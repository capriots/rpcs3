#include "stdafx.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/Cell/lv2/sys_mutex.h"
#include "Emu/Cell/lv2/sys_cond.h"
#include "Emu/Cell/lv2/sys_sync.h"
#include "Emu/Cell/lv2/sys_ppu_thread.h"
#include "Emu/savestate_utils.hpp"
#include "Emu/System.h"
#include "sysPrxForUser.h"
#include "util/asm.hpp"

#include <bitset>
#include <ranges>
#include "cellDmux.h"
#include "cellDmuxPamf.h"

vm::gvar<CellDmuxCoreOps> g_cell_dmux_core_ops_pamf;
vm::gvar<CellDmuxCoreOps> g_cell_dmux_core_ops_raw_es;

LOG_CHANNEL(cellDmuxPamf)

template <>
void fmt_class_string<CellDmuxPamfError>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](CellDmuxPamfError value)
	{
		switch (value)
		{
			STR_CASE(CELL_DMUX_PAMF_ERROR_BUSY);
			STR_CASE(CELL_DMUX_PAMF_ERROR_ARG);
			STR_CASE(CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM);
			STR_CASE(CELL_DMUX_PAMF_ERROR_NO_MEMORY);
			STR_CASE(CELL_DMUX_PAMF_ERROR_FATAL);
		}

		return unknown;
	});
}

inline std::pair<DmuxPamfStreamTypeIndex, u32> dmuxPamfStreamIdToTypeChannel(u8 stream_id, u8 private_stream_id)
{
	if ((stream_id & 0xf0) == 0xe0)
	{
		return { DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO, stream_id & 0x0f };
	}

	if (stream_id != 0xbd)
	{
		return { DMUX_PAMF_STREAM_TYPE_INDEX_INVALID, 0 };
	}

	switch (private_stream_id & 0xf0)
	{
	case 0x40: return { DMUX_PAMF_STREAM_TYPE_INDEX_LPCM, private_stream_id & 0x0f };
	case 0x30: return { DMUX_PAMF_STREAM_TYPE_INDEX_AC3, private_stream_id & 0x0f };
	case 0x00: return { DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX, private_stream_id & 0x0f };
	case 0x20: return { DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA, private_stream_id & 0x0f };
	default:   return { DMUX_PAMF_STREAM_TYPE_INDEX_INVALID, 0 };
	}
}

//----------------------------------------------------------------------------
// SPU
//----------------------------------------------------------------------------

// TODO comments everywhere

void dmux_pamf_context::elementary_stream::access_unit_queue::wrap()
{
	wrap_pos = back;
	back = buffer.begin();
}

void dmux_pamf_context::elementary_stream::access_unit_queue::pop_back(u32 au_size)
{
	back -= au_size;
}

void dmux_pamf_context::elementary_stream::access_unit_queue::pop_back(std::span<std::byte>::iterator au_pos)
{
	if (wrap_pos != buffer.begin() && au_pos > back)
	{
		wrap_pos = buffer.begin();
	}

	back = au_pos;
}

void dmux_pamf_context::elementary_stream::access_unit_queue::pop_front(u32 au_size)
{
	front += au_size;

	if (wrap_pos != buffer.begin() && wrap_pos <= front)
	{
		ensure(wrap_pos == front);
		front =
		wrap_pos = buffer.begin();
	}
}

void dmux_pamf_context::elementary_stream::access_unit_queue::push(const access_unit_chunk& au_chunk)
{
	std::ranges::copy(au_chunk.cached_data, back);
	std::ranges::copy(au_chunk.data, back + au_chunk.cached_data.size());
	back += au_chunk.data.size() + au_chunk.cached_data.size();
}

void dmux_pamf_context::elementary_stream::flush_es()
{
	if (current_au.accumulated_size != 0)
	{
		au_queue.push({ {}, cache });

		current_au.accumulated_size += cache.size();

		ctx.event_queue_too_full = !ctx.send_event(DmuxPamfEventType::au_found, stream_id, private_stream_id, vm::get_addr(std::to_address(au_queue.back) - current_au.accumulated_size), std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(current_au.pts)),
			std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(current_au.dts)), 0, current_au.accumulated_size, au_specific_info_size, current_au.au_specific_info_buf, es_id, current_au.rap);
	}

	reset();

	while (!ctx.send_event(DmuxPamfEventType::flush_done, stream_id, private_stream_id, es_id)) {}
}

void dmux_pamf_context::elementary_stream::reset_es(std::byte* au_addr)
{
	if (!au_addr)
	{
		reset();
		au_queue.front = au_queue.back = au_queue.wrap_pos = au_queue.buffer.begin();
	}
	else
	{
		au_queue.pop_back(std::span<std::byte>::iterator(au_addr));
	}
}

void dmux_pamf_context::elementary_stream::discard_access_unit()
{
	au_queue.pop_back(current_au.accumulated_size - (au_chunk.data.size() + au_chunk.cached_data.size()));
	reset();
	cache.clear();
}

template <u32 unk_size_mask>
u32 dmux_pamf_context::elementary_stream::parse_audio_stream_header(const std::span<const std::byte>& elementary_stream)
{
	u32 unk_size = 0;

	if (au_size_unk == 0)
	{
		// Not checked on LLE
		if (elementary_stream.size() < sizeof(u32))
		{
			return umax;
		}

		unk_size = read_from_ptr<be_t<u32>>(elementary_stream) & unk_size_mask;
		au_size_unk = 1;
	}

	return unk_size + 4;
}


bool dmux_pamf_context::elementary_stream::process_stream()
{
	for (;;)
	{
		switch (state)
		{
		case state::initial:
			if (current_stream_chunk.empty())
			{
				ctx.au_queue_full = false;
				return true;
			}

			current_stream_chunk = current_stream_chunk.subspan(parse_stream(current_stream_chunk));

			ensure(current_au.state != access_unit::state::size_mismatch, "Size mismatch between distance of sync words and au-size specified in stream");

			current_au.accumulated_size += au_chunk.data.size() + au_chunk.cached_data.size();

			if (!current_au.timestamps_rap_set && (current_au.state == access_unit::state::commenced || current_au.state == access_unit::state::m2v_sequence || (current_au.state == access_unit::state::complete && au_chunk.cached_data.empty())))
			{
				set_au_timestamps_rap();
			}

			state = state::pushing_au_queue;
			[[fallthrough]];

		case state::pushing_au_queue:
			if (!au_chunk.data.empty() || !au_chunk.cached_data.empty())
			{
				if (au_queue.wrap_pos == au_queue.buffer.begin())
				{
					if (au_queue.back + au_chunk.data.size() + au_chunk.cached_data.size() > au_queue.buffer.end())
					{
						cellDmuxPamf.error("Access unit larger than specified maximum access unit size");
						ctx.send_event(DmuxPamfEventType::fatal_error);
						ctx.au_queue_full = true;
						return false;
					}
				}
				else if (static_cast<s64>(au_chunk.data.size() + au_chunk.cached_data.size() + 0x10) > au_queue.front - au_queue.back) // + sizeof(v128) because of SPU shenanigans probably
				{
					ctx.au_queue_full = true;
					return false;
				}

				au_queue.push(au_chunk);
				au_chunk = {};
			}

			state = state::notifying_au_found;
			[[fallthrough]];

		case state::notifying_au_found:
			if (current_au.state == access_unit::state::complete && !ctx.send_event(DmuxPamfEventType::au_found, stream_id, private_stream_id, vm::get_addr(std::to_address(au_queue.back) - current_au.accumulated_size),
				std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(current_au.pts)), std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(current_au.dts)), 0, current_au.accumulated_size, au_specific_info_size, current_au.au_specific_info_buf, es_id, current_au.rap))
			{
				ctx.event_queue_too_full = true;
				ctx.au_queue_full = false;
				return false;
			}

			state = state::waiting_for_au_released;
			[[fallthrough]];

		case state::waiting_for_au_released:
			if (current_au.state == access_unit::state::complete && au_queue.back + au_max_size > au_queue.buffer.end())
			{
				if (au_queue.wrap_pos != au_queue.buffer.begin())
				{
					ctx.au_queue_full = true;
					return false;
				}

				au_queue.wrap();
			}

			state = state::initial;

			if (current_au.state == access_unit::state::complete)
			{
				current_au = {};
			}
		}
	}
}

template <bool avc>
u32 dmux_pamf_context::video_stream<avc>::parse_stream(const std::span<const std::byte>& stream)
{
	if (current_au.state != access_unit::state::none && (avc || current_au.state != access_unit::state::m2v_sequence))
	{
		current_au.state = access_unit::state::incomplete;
	}

	std::array<std::byte, 8> buf{};
	std::ranges::copy(cache, buf.begin());
	std::copy_n(stream.begin(), std::min(sizeof(u32) - 1, stream.size()), buf.begin() + cache.size());

	auto partial_au_begin = stream.begin();
	s64 cache_idx = 0;
	auto stream_it = stream.begin();
	[&]
	{
		for (; cache_idx <= static_cast<s64>(cache.size() + std::min(sizeof(u32) - 1, stream.size()) - sizeof(u32)); cache_idx++)
		{
			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(buf.data() + cache_idx); (avc && code == AVC_AU_DELIMITER) || (!avc && (code == M2V_PIC_START || code == M2V_SEQUENCE_HEADER || code == M2V_SEQUENCE_END)))
			{
				if (current_au.state != access_unit::state::none && (avc || current_au.state != access_unit::state::m2v_sequence))
				{
					stream_it += !avc && code == M2V_SEQUENCE_END ? sizeof(u32) : 0;
					current_au.state = access_unit::state::complete;
					return;
				}

				// TODO fist code in cache ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

				current_au.state = avc || code == M2V_PIC_START ? access_unit::state::commenced : access_unit::state::m2v_sequence;
			}
		}

		for (; stream_it <= stream.end() - sizeof(u32); stream_it++)
		{
			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(stream_it); (avc && code == AVC_AU_DELIMITER) || (!avc && (code == M2V_PIC_START || code == M2V_SEQUENCE_HEADER || code == M2V_SEQUENCE_END)))
			{
				if (current_au.state != access_unit::state::none && (avc || current_au.state != access_unit::state::m2v_sequence))
				{
					stream_it += !avc && code == M2V_SEQUENCE_END ? sizeof(u32) : 0;
					current_au.state = access_unit::state::complete;
					return;
				}

				partial_au_begin = avc || current_au.state == access_unit::state::none ? stream_it : partial_au_begin;
				current_au.state = avc || code == M2V_PIC_START ? access_unit::state::commenced : access_unit::state::m2v_sequence;
			}
		}
	}();

	au_chunk.data = { partial_au_begin, stream_it };

	if (current_au.state != access_unit::state::none)
	{
		std::copy_n(cache.begin(), cache_idx, std::back_inserter(au_chunk.cached_data));
		cache.erase(cache.begin(), cache.begin() + cache_idx);
	}

	if (current_au.state != access_unit::state::complete)
	{
		std::copy(stream_it, stream.end(), std::back_inserter(cache));
	}

	return (current_au.state != access_unit::state::complete ? stream.end() : stream_it) - stream.begin();
}
u32 dmux_pamf_context::lpcm_stream::parse_stream_header(const std::span<const std::byte>& elementary_stream, [[maybe_unused]] s8 pts_dts_flag)
{
	// Not checked on LLE
	if (elementary_stream.size() < 1 + 0x10)
	{
		return umax;
	}

	std::memcpy(au_specific_info_buf.data(), &elementary_stream[1], au_specific_info_buf.size());
	return parse_audio_stream_header<0x7ff>(elementary_stream);
}

u32 dmux_pamf_context::lpcm_stream::parse_stream(const std::span<const std::byte>& stream)
{
	if (current_au.state == access_unit::state::none)
	{
		current_au.au_specific_info_buf = au_specific_info_buf;
	}

	if (au_max_size - current_au.accumulated_size > stream.size())
	{
		au_chunk.data = stream;
		current_au.state = current_au.state == access_unit::state::none ? access_unit::state::commenced : access_unit::state::incomplete;
	}
	else
	{
		au_chunk.data = stream.first(au_max_size - current_au.accumulated_size);
		current_au.state = access_unit::state::complete;
	}

	return au_chunk.data.size();
}

template <bool ac3>
u32 dmux_pamf_context::audio_stream<ac3>::parse_stream(const std::span<const std::byte>& stream)
{
	const auto parse_au_size = [](be_t<u16> data) -> u16
	{
		if constexpr (ac3)
		{
			if (const u8 fscod = data >> 14, frmsizecod = (data >> 8) & 0x3f; fscod < 3 && frmsizecod < 38)
			{
				return AC3_FRMSIZE_TABLE[fscod][frmsizecod] * sizeof(s16);
			}
		}
		else if ((data & 0x3ff) < 0x200)
		{
			return ((data & 0x3ff) + 1) * 8 + ATRACX_ATS_HEADER_SIZE;
		}

		return 0;
	};

	if (current_au.state != access_unit::state::none)
	{
		current_au.state = access_unit::state::incomplete;
	}

	std::array<std::byte, 8> buf{};
	std::ranges::copy(cache, buf.begin());
	std::copy_n(stream.begin(), std::min(sizeof(u16) - 1, stream.size()), buf.begin() + cache.size());

	auto partial_au_begin = stream.begin();
	s64 cache_idx = 0;
	auto stream_it = stream.begin();
	[&]
	{
		for (; cache_idx <= static_cast<s64>(cache.size() + std::min(sizeof(u16) - 1, stream.size()) - sizeof(u16)); cache_idx++)
		{
			if (const be_t<u16> tmp = read_from_ptr<be_t<u16>>(buf.data() + cache_idx); current_au.size_info_offset != 0)
			{
				if (--current_au.size_info_offset == 0)
				{
					current_au.parsed_size = parse_au_size(tmp);
				}
			}
			else if ((ac3 && tmp == AC3_SYNCWORD) || (!ac3 && tmp == ATRACX_SYNCWORD))
			{
				if (current_au.state == access_unit::state::none)
				{
					current_au.size_info_offset = ac3 ? sizeof(u16) * 2 : sizeof(u16);
					current_au.state = access_unit::state::commenced;
				}
				else if (const u32 au_size = current_au.accumulated_size + cache_idx; au_size >= current_au.parsed_size)
				{
					current_au.state = au_size == current_au.parsed_size ? access_unit::state::complete : access_unit::state::size_mismatch;
					return;
				}
			}
		}

		for (; stream_it <= stream.end() - sizeof(u32); stream_it++) // LLE uses sizeof(u32), even though we only read sizeof(u16) bytes
		{
			if (const be_t<u16> tmp = read_from_ptr<be_t<u16>>(stream_it); current_au.size_info_offset != 0)
			{
				if (--current_au.size_info_offset == 0)
				{
					current_au.parsed_size = parse_au_size(tmp);
				}
			}
			else if ((ac3 && tmp == AC3_SYNCWORD) || (!ac3 && tmp == ATRACX_SYNCWORD))
			{
				if (current_au.state == access_unit::state::none)
				{
					partial_au_begin = stream_it;
					current_au.size_info_offset = ac3 ? sizeof(u16) * 2 : sizeof(u16);
					current_au.state = access_unit::state::commenced;
				}
				else if (const u32 au_size = current_au.accumulated_size + stream_it - partial_au_begin + cache.size(); au_size >= current_au.parsed_size)
				{
					current_au.state = au_size == current_au.parsed_size ? access_unit::state::complete : access_unit::state::size_mismatch;
					return;
				}
			}
		}
	}();

	au_chunk.data = { partial_au_begin, stream_it };

	if (current_au.state != access_unit::state::none)
	{
		std::copy_n(cache.begin(), cache_idx, std::back_inserter(au_chunk.cached_data));
		cache.erase(cache.begin(), cache.begin() + cache_idx);
	}

	if (current_au.state != access_unit::state::complete)
	{
		std::copy(stream_it, stream.end(), std::back_inserter(cache));
	}

	return (current_au.state != access_unit::state::complete ? stream.end() : stream_it) - stream.begin();
}

u32 dmux_pamf_context::user_data_stream::parse_stream_header(const std::span<const std::byte>& elementary_stream, s8 pts_dts_flag)
{
	if (pts_dts_flag < 0) // PTS field exists
	{
		// Not checked on LLE
		if (2 + sizeof(u32) > elementary_stream.size())
		{
			return umax;
		}

		au_size_unk = read_from_ptr<be_t<u32>>(elementary_stream.begin() + 2) - sizeof(u32);
		return 10;
	}

	return 2;
}

u32 dmux_pamf_context::user_data_stream::parse_stream(const std::span<const std::byte>& stream)
{
	if (au_size_unk > stream.size())
	{
		au_chunk.data = stream;
		au_size_unk -= stream.size();
		current_au.state = access_unit::state::commenced; // User data always uses commenced
	}
	else
	{
		au_chunk.data = stream.first(au_size_unk);
		au_size_unk = 0;
		current_au.state = access_unit::state::complete;
	}

	return stream.size();
}

void dmux_pamf_context::demuxer::enable_es(u32 stream_id, u32 private_stream_id, bool is_avc, u32 au_queue_buffer_size, std::byte* au_queue_buffer, u32 au_max_size, bool raw_es, u32 es_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return;
	}

	this->raw_es = raw_es;
	es_type_idx = type_idx;

	switch (type_idx)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		elementary_streams[0][channel] = is_avc ? static_cast<std::unique_ptr<elementary_stream>>(std::make_unique<video_stream<true>>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size))
			: std::make_unique<video_stream<false>>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		return;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:      elementary_streams[1][channel] = std::make_unique<lpcm_stream>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size); return;
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:       elementary_streams[2][channel] = std::make_unique<audio_stream<true>>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size); return;
	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:    elementary_streams[3][channel] = std::make_unique<audio_stream<false>>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size); return;
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA: elementary_streams[4][channel] = std::make_unique<user_data_stream>(ctx, stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size); return;
	default: fmt::throw_exception("Unreachable");
	}
}

void dmux_pamf_context::demuxer::disable_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return;
	}

	elementary_streams[type_idx][channel].reset();
}

void dmux_pamf_context::demuxer::release_au(u32 au_size, u32 stream_id, u32 private_stream_id) const
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return;
	}

	elementary_streams[type_idx][channel]->release_au(au_size);
}

void dmux_pamf_context::demuxer::flush_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return;
	}

	state = state::initial;
	elementary_streams[type_idx][channel]->flush_es();
}

void dmux_pamf_context::demuxer::reset_stream()
{
	std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), &elementary_stream::discard_access_unit);
	state = state::initial;
	input_stream = {};
}

void dmux_pamf_context::demuxer::reset_es(u32 stream_id, u32 private_stream_id, std::byte* au_addr)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return;
	}

	if (!au_addr)
	{
		state = state::initial;
	}

	elementary_streams[type_idx][channel]->reset_es(au_addr);
}

inline bool dmux_pamf_context::demuxer::reset_state_and_check_demux_done()
{
	state = state::initial;
	ctx.au_queue_full = false;
	return check_demux_done();
}

inline bool dmux_pamf_context::demuxer::check_demux_done()
{
	if (!demux_done || demux_done_was_notified)
	{
		return true;
	}

	if (!ctx.send_event(DmuxPamfEventType::demux_done))
	{
		ctx.event_queue_too_full = true;
		return false;
	}

	demux_done_was_notified = true;
	return true;
}

// TODO logging
bool dmux_pamf_context::demuxer::demux(const DmuxPamfStreamInfo* stream_info)
{
	if (demux_done)
	{
		if (stream_info)
		{
			if (!demux_done_was_notified)
			{
				ctx.au_queue_full = false;

				if (!ctx.send_event(DmuxPamfEventType::demux_done))
				{
					ctx.event_queue_too_full = true;
					return false;
				}

				demux_done_was_notified = true;
				return true;
			}

			if (!stream_info->continuity)
			{
				std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), &elementary_stream::discard_access_unit );
			}

			state = state::initial;

			// Not checked on LLE, it would parse old memory contents or uninitialized memory if the size of the input stream set by the user is not a multiple of 0x800.
			// Valid PAMF streams are always a multiple of 0x800 bytes large.
			if (stream_info->stream_size & 0x7ff)
			{
				cellDmuxPamf.error("Invalid input stream size");
				return reset_state_and_check_demux_done();
			}

			input_stream.set_stream({ stream_info->stream_addr.get_ptr(), stream_info->stream_size });
			demux_done = false;
			demux_done_was_notified = false;
		}
		else
		{
			return reset_state_and_check_demux_done();
		}
	}

	switch (state)
	{
	case state::initial:
	{
		for (;;)
		{
			if (input_stream.eof())
			{
				demux_done = true;
				return reset_state_and_check_demux_done();
			}

			current_pack.stream = input_stream.get_next_pack();

			// If the input stream is a raw elementary stream, skip everything MPEG-PS related and go straight to elementary stream parsing
			if (raw_es)
			{
				current_pack.es_type_idx = es_type_idx;
				current_pack.es_channel = 0;

				if (elementary_stream::is_enabled(elementary_streams[es_type_idx][0]))
				{
					elementary_streams[es_type_idx][0]->set_stream(current_pack.stream);
				}

				state = state::elementary_stream;
				ctx.au_queue_full = false;
				return check_demux_done();
			}

			// While LLE is actually searching the entire input stream for a pack start code or program end code,
			// it doesn't set its current stream position/index to the found code, so it would bug out if there isn't one at the current position
			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(current_pack.stream); code == PACK_START)
			{
				break;
			}
			else if (code == PROG_END)
			{
				if (!ctx.send_event(DmuxPamfEventType::prog_end_code))
				{
					state = state::prog_end;
					ctx.event_queue_too_full = true;
					ctx.au_queue_full = false;
					return check_demux_done();
				}

				return reset_state_and_check_demux_done();
			}

			cellDmuxPamf.warning("No start code found at the beginning of the current section");
		}

		// Skip over pack header
		// No need to check the remaining stream size here since the maximum size of a pack header is 0xa8 bytes, while we already made sure that the remaining stream is at least 0x800 bytes large
		const u8 pack_stuffing_length = read_from_ptr<u8>(current_pack.stream.begin() + PACK_STUFFING_LENGTH_OFFSET) & 0x7;
		auto current_pes_packet = current_pack.stream.subspan(PACK_STUFFING_LENGTH_OFFSET + sizeof(u8) + pack_stuffing_length);

		if (read_from_ptr<be_t<u32>>(current_pes_packet) >> 8 != PACKET_START_CODE_PREFIX)
		{
			cellDmuxPamf.error("Invalid start code after pack header");
			return reset_state_and_check_demux_done();
		}

		// Skip over system header if present
		if (read_from_ptr<be_t<u32>>(current_pes_packet) == SYSTEM_HEADER)
		{
			const u32 system_header_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);

			// Not checked on LLE, the SPU task would just increment the reading position and bug out
			if (system_header_length + sizeof(u32) + sizeof(u16) > current_pes_packet.size())
			{
				cellDmuxPamf.error("Invalid system header length");
				return reset_state_and_check_demux_done();
			}

			current_pes_packet = current_pes_packet.subspan(system_header_length);

			// The SPU thread isn't doing load + rotate here for 4-byte loading (for valid PAMF streams, the next start code after a system header is always aligned to 0x10 bytes)
			const uptr addr_low = reinterpret_cast<uptr>(current_pes_packet.data()) & 0xf;
			current_pes_packet = { current_pes_packet.begin() - addr_low, current_pes_packet.end() };

			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(current_pes_packet); code >> 8 != PACKET_START_CODE_PREFIX)
			{
				cellDmuxPamf.error("Invalid start code after system header");
				return reset_state_and_check_demux_done();
			}

			// A system header is optionally followed by a private stream 2
			// The first two bytes of the stream are the stream id of a video stream. The next access unit of that stream is a random access point/keyframe
			else if (code == PRIVATE_STREAM_2)
			{
				const u16 pes_packet_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);

				// Not checked on LLE, the SPU task would just increment the reading position and bug out
				if (pes_packet_length + PES_HEADER_DATA_LENGTH_OFFSET + sizeof(u8) > current_pes_packet.size() || pes_packet_length < 8)
				{
					cellDmuxPamf.error("Invalid private stream 2 length");
					return reset_state_and_check_demux_done();
				}

				if (const u8 channel = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET + sizeof(u16)) & 0xf; elementary_stream::is_enabled(elementary_streams[0][channel]))
				{
					elementary_streams[0][channel]->set_rap();
				}

				current_pes_packet = current_pes_packet.subspan(pes_packet_length);
			}
		}

		// PES packet
		const u32 pes_packet_start_code = read_from_ptr<be_t<u32>>(current_pes_packet);

		if (pes_packet_start_code >> 8 != PACKET_START_CODE_PREFIX)
		{
			return reset_state_and_check_demux_done();
		}

		const u16 pes_packet_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);
		const u8 pes_header_data_length = read_from_ptr<u8>(current_pes_packet.begin() + PES_HEADER_DATA_LENGTH_OFFSET) + PES_HEADER_DATA_LENGTH_OFFSET + sizeof(u8);

		// Not checked on LLE, the SPU task would just increment the reading position and bug out
		if (pes_packet_length > current_pes_packet.size()
			|| pes_packet_length <= pes_header_data_length)
		{
			cellDmuxPamf.error("Invalid pes packet length");
			return reset_state_and_check_demux_done();
		}

		const auto elementary_stream = current_pes_packet.subspan(pes_header_data_length, pes_packet_length - pes_header_data_length);

		const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(pes_packet_start_code, read_from_ptr<u8>(elementary_stream));

		if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
		{
			return reset_state_and_check_demux_done();
		}

		current_pack.es_type_idx = type_idx;
		current_pack.es_channel = channel;

		if (elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
		{
			const s8 pts_dts_flag = read_from_ptr<s8>(current_pes_packet.begin() + PTS_DTS_FLAG_OFFSET);

			if (pts_dts_flag < 0)
			{
				const s32 PTS_32_30 = read_from_ptr<u8>(current_pes_packet.begin() + 9) >> 1;
				const s32 PTS_29_15 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 10) >> 1;
				const s32 PTS_14_0 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 12) >> 1;

				elementary_streams[type_idx][channel]->set_pts(PTS_32_30 << 30 | PTS_29_15 << 15 | PTS_14_0); // Bit 32 is discarded
			}

			if (pts_dts_flag & 0x40)
			{
				const s32 DTS_32_30 = read_from_ptr<u8>(current_pes_packet.begin() + 14) >> 1;
				const s32 DTS_29_15 = read_from_ptr<u16>(current_pes_packet.begin() + 15) >> 1;
				const s32 DTS_14_0 = read_from_ptr<u16>(current_pes_packet.begin() + 17) >> 1;

				elementary_streams[type_idx][channel]->set_dts(DTS_32_30 << 30 | DTS_29_15 << 15 | DTS_14_0); // Bit 32 is discarded
			}

			const usz stream_header_size = elementary_streams[type_idx][channel]->parse_stream_header(elementary_stream, pts_dts_flag);

			// Not checked on LLE, the SPU task would just increment the reading position and bug out
			if (stream_header_size > elementary_stream.size())
			{
				cellDmuxPamf.error("Invalid stream header size");
				return reset_state_and_check_demux_done();
			}

			elementary_streams[type_idx][channel]->set_stream(elementary_stream.subspan(stream_header_size));
		}

		state = state::elementary_stream;
		[[fallthrough]];
	}
	case state::elementary_stream:
	{
		if (!elementary_stream::is_enabled(elementary_streams[current_pack.es_type_idx][current_pack.es_channel]))
		{
			return reset_state_and_check_demux_done();
		}

		if (elementary_streams[current_pack.es_type_idx][current_pack.es_channel]->process_stream())
		{
			state = state::initial;
		}

		return check_demux_done();
	}
	case state::prog_end:
	{
		if (!ctx.send_event(DmuxPamfEventType::prog_end_code))
		{
			ctx.event_queue_too_full = true;
			ctx.au_queue_full = false;
			return check_demux_done();
		}

		return reset_state_and_check_demux_done();
	}
	default:
		fmt::throw_exception("Unreachable");
	}
}

bool dmux_pamf_context::get_next_cmd(DmuxPamfCommand& lhs, bool new_stream) const
{
	cellDmuxPamf.trace("Getting next command");

	if (cmd_queue->pop(lhs))
	{
		cellDmuxPamf.trace("Command type: %d", static_cast<u32>(lhs.type.get()));
		return true;
	}

	if ((new_stream || demuxer.has_work()) && !au_queue_full && !event_queue_too_full)
	{
		cellDmuxPamf.trace("No new command, continuing demuxing");
		return false;
	}

	cellDmuxPamf.trace("No new command and nothing to do, waiting...");

	cmd_queue->wait();

	if (thread_ctrl::state() == thread_state::aborting)
	{
		return false;
	}

	ensure(cmd_queue->pop(lhs));

	cellDmuxPamf.trace("Command type: %d", static_cast<u32>(lhs.type.get()));
	return true;
}

bool dmux_pamf_context::send_event(auto&&... args) const
{
	if (event_queue->size() >= max_enqueued_events)
	{
		return false;
	}

	return ensure(event_queue->emplace(std::forward<decltype(args)>(args)..., event_queue_was_too_full));
}

void dmux_pamf_context::operator()() // cellSpursMain()
{
	while (!Emu.IsRunning())
	{
		thread_ctrl::wait_for(5'000);

		if (thread_ctrl::state() == thread_state::aborting)
		{
			return;
		}
	}

	DmuxPamfCommand cmd;

	while (thread_ctrl::state() != thread_state::aborting)
	{
		if (get_next_cmd(cmd, new_stream))
		{
			event_queue_was_too_full = event_queue_too_full;
			event_queue_too_full = false;

			switch (cmd.type)
			{
			case DmuxPamfCommandType::enable_es:
				max_enqueued_events += 2;
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::enable_es) + 1));
				demuxer.enable_es(cmd.enable_es.stream_id, cmd.enable_es.private_stream_id, cmd.enable_es.is_avc, cmd.enable_es.au_queue_buffer_size, cmd.enable_es.au_queue_buffer.get_ptr(), cmd.enable_es.au_max_size, cmd.enable_es.is_raw_es, cmd.enable_es.es_id);
				break;

			case DmuxPamfCommandType::disable_es:
				demuxer.disable_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::disable_es) + 1));
				max_enqueued_events -= 2;
				break;

			case DmuxPamfCommandType::set_stream:
				new_stream = true;
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::set_stream) + 1));
				break;

			case DmuxPamfCommandType::release_au:
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::release_au) + 1));
				demuxer.release_au(cmd.release_au.au_size, cmd.release_au.stream_id, cmd.release_au.private_stream_id);
				break;

			case DmuxPamfCommandType::flush_es:
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::flush_es) + 1));
				demuxer.flush_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				break;

			case DmuxPamfCommandType::close:
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::close) + 1));
				while (!send_event(DmuxPamfEventType::close)){}
				return;

			case DmuxPamfCommandType::reset_stream:
				demuxer.reset_stream();
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::reset_stream) + 1));
				break;

			case DmuxPamfCommandType::reset_es:
				demuxer.reset_es(cmd.reset_es.stream_id, cmd.reset_es.private_stream_id, cmd.reset_es.au_addr.get_ptr());
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::reset_es) + 1));
				break;

			case DmuxPamfCommandType::resume:
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::resume) + 1));
				break;

			default:
				cmd_result_queue->emplace(1000);
			}
		}
		else if (thread_ctrl::state() == thread_state::aborting)
		{
			return;
		}

		if (new_stream && !demuxer.has_work())
		{
			new_stream = false;

			DmuxPamfStreamInfo stream_info;
			ensure(stream_info_queue->pop(stream_info));

			demuxer.demux(&stream_info);
		}
		else
		{
			demuxer.demux(nullptr);
		}
	}
}

void dmux_pamf_context::elementary_stream::save(utils::serial& ar)
{
	ar(state, es_id, vm::get_addr(current_stream_chunk.data()), static_cast<u32>(current_stream_chunk.size()), pts, dts, rap, vm::get_addr(au_queue.buffer.data()), static_cast<u32>(au_queue.buffer.size()),
		vm::get_addr(std::to_address(au_queue.back)), vm::get_addr(std::to_address(au_queue.front)), vm::get_addr(std::to_address(au_queue.wrap_pos)), au_max_size, au_size_unk, current_au);

	if (state == state::pushing_au_queue)
	{
		ar(au_chunk.cached_data, vm::get_addr(au_chunk.data.data()), static_cast<u32>(au_chunk.data.size()));
	}

	if (current_au.state != access_unit::state::complete)
	{
		ar(cache);
	}
}

void dmux_pamf_context::demuxer::save(utils::serial& ar)
{
	ar(state, demux_done, demux_done_was_notified, raw_es, es_type_idx);

	if (!demux_done)
	{
		input_stream.save(ar);
	}

	if (state == state::elementary_stream)
	{
		if (ar.is_writing())
		{
			ar(vm::get_addr(current_pack.stream.data()), current_pack.es_type_idx, current_pack.es_channel);
		}
		else
		{
			current_pack.stream = { vm::_ptr<const std::byte>(ar.pop<vm::addr_t>()), 0x800 };
			ar(current_pack.es_type_idx, current_pack.es_channel);
		}
	}

	std::array<bool, 0x10> video_streams_bitset;
	std::array<bool, 0x10> video_streams_avc_bitset;
	std::array<bool, 0x10> lpcm_streams_bitset;
	std::array<bool, 0x10> ac3_streams_bitset;
	std::array<bool, 0x10> atracx_streams_bitset;
	std::array<bool, 0x10> user_data_streams_bitset;

	if (ar.is_writing())
	{
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO], video_streams_bitset.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO], video_streams_avc_bitset.begin(), [](const auto& es){ return !dynamic_cast<video_stream<false>*>(es.get()); });
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_LPCM], lpcm_streams_bitset.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_AC3], ac3_streams_bitset.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX], atracx_streams_bitset.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA], user_data_streams_bitset.begin(), elementary_stream::is_enabled);
	}

	ar(video_streams_bitset, video_streams_avc_bitset, lpcm_streams_bitset, ac3_streams_bitset, atracx_streams_bitset, user_data_streams_bitset);

	if (ar.is_writing())
	{
		std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), [&ar](const auto& es){ es->save(ar); });
	}
	else
	{
		for (u32 ch = 0; ch < 0x10; ch++)
			if (video_streams_bitset[ch])
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO][ch] = video_streams_avc_bitset[ch]
					? static_cast<std::unique_ptr<elementary_stream>>(std::make_unique<video_stream<true>>(ctx, ar, 0xe0 | ch))
					: std::make_unique<video_stream<false>>(ctx, ar, 0xe0 | ch);

		for (u32 ch = 0; ch < 0x10; ch++)
			if (lpcm_streams_bitset[ch])
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_LPCM][ch] = std::make_unique<lpcm_stream>(ctx, ar, 0x40 | ch);

		for (u32 ch = 0; ch < 0x10; ch++)
			if (ac3_streams_bitset[ch])
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_AC3][ch] = std::make_unique<audio_stream<true>>(ctx, ar, 0x30 | ch);

		for (u32 ch = 0; ch < 0x10; ch++)
			if (atracx_streams_bitset[ch])
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX][ch] = std::make_unique<audio_stream<false>>(ctx, ar, 0x00 | ch);

		for (u32 ch = 0; ch < 0x10; ch++)
			if (user_data_streams_bitset[ch])
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA][ch] = std::make_unique<user_data_stream>(ctx, ar, 0x20 | ch);
	}
}

void dmux_pamf_context::save(utils::serial& ar)
{
	USING_SERIALIZATION_VERSION(cellDmuxPamf);
	ar(cmd_queue); // The queues are contiguous in guest memory, so we only need to save the address of the first one
	demuxer.save(ar);
	ar(new_stream);
}


//----------------------------------------------------------------------------
// PPU
//----------------------------------------------------------------------------

template <DmuxPamfCommandType type>
void DmuxPamfContext::send_spu_command_and_wait(ppu_thread& ppu, bool waiting_for_spu_state, auto&&... cmd_params)
{
	if (!waiting_for_spu_state)
	{
		// The caller is supposed to own the mutex until the SPU thread has consumed the command, so the queue should always be empty here
		ensure(cmd_queue.emplace(type, std::forward<decltype(cmd_params)>(cmd_params)...), "The command queue wasn't empty");
	}

	lv2_obj::sleep(ppu);

	// Block until the SPU thread has consumed the command
	cmd_result_queue.wait();

	if (ppu.check_state())
	{
		ppu.state += cpu_flag::again;
		return;
	}

	be_t<u32> result{};
	ensure(cmd_result_queue.pop(result), "The result queue was empty");
	ensure(result == static_cast<u32>(type) + 1, "The HLE SPU thread sent an invalid result");
}

DmuxPamfElementaryStream* DmuxPamfContext::find_es(u16 stream_id, u16 private_stream_id)
{
	const auto it = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first == DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO
		? std::ranges::find_if(elementary_streams | std::views::reverse, [&](const auto& es){ return es && es->stream_id == stream_id; })
		: std::ranges::find_if(elementary_streams | std::views::reverse, [&](const auto& es){ return es && es->stream_id == stream_id && es->private_stream_id == private_stream_id; });

	return it != std::ranges::rend(elementary_streams) ? it->get_ptr() : nullptr;
}

error_code DmuxPamfContext::wait_au_released_or_stream_reset(ppu_thread& ppu, u64 au_queue_full_bitset, b8& stream_reset_started, dmux_pamf_state& savestate)
{
	if (savestate == dmux_pamf_state::waiting_for_au_released)
	{
		goto label1_waiting_for_au_released_state;
	}

	if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		return {};
	}

	if (au_queue_full_bitset)
	{
		cellDmuxPamf.trace("AU queue of elementary stream %d is full. Waiting for AU to be released...", std::countr_zero(au_queue_full_bitset));

		while (!(au_queue_full_bitset & au_released_bitset) && !stream_reset_requested)
		{
			savestate = dmux_pamf_state::waiting_for_au_released;
			label1_waiting_for_au_released_state:

			if (sys_cond_wait(ppu, cond, 0) != CELL_OK)
			{
				sys_mutex_unlock(ppu, mutex);
				return CELL_DMUX_PAMF_ERROR_FATAL;
			}

			if (ppu.state & cpu_flag::again)
			{
				return {};
			}
		}

		cellDmuxPamf.trace("AU released");
	}

	stream_reset_started = stream_reset_requested;
	stream_reset_requested = false;

	au_released_bitset = 0;

	return sys_mutex_unlock(ppu, mutex) != CELL_OK ? static_cast<error_code>(CELL_DMUX_PAMF_ERROR_FATAL) : CELL_OK;
}

template <bool reset>
error_code DmuxPamfContext::set_au_reset(ppu_thread& ppu)
{
	if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		return {};
	}

	std::ranges::for_each(elementary_streams, [](auto& es){ if (es) es->reset_next_au = reset; });

	return sys_mutex_unlock(ppu, mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

template <typename F>
error_code DmuxPamfContext::callback(ppu_thread& ppu, DmuxCb<F> cb, auto&&... args)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	return cb.cbFunc(ppu, std::forward<decltype(args)>(args)..., cb.cbArg);
}

// This is repeated a lot in DmuxPamfContext::exec()
#define SEND_FATAL_ERR_AND_CONTINUE()\
	savestate = dmux_pamf_state::sending_fatal_err;\
	callback(ppu, notify_fatal_err, _this, CELL_OK); /* LLE uses CELL_OK as error code */\
	if (ppu.state & cpu_flag::again)\
	{\
		return;\
	}\
	continue;

void DmuxPamfContext::exec(ppu_thread& ppu)
{
	switch (savestate)
	{
	case dmux_pamf_state::initial: break;
	case dmux_pamf_state::waiting_for_au_released: goto label1_waiting_for_au_released_state;
	case dmux_pamf_state::waiting_for_au_released_error: goto label2_waiting_for_au_released_error_state;
	case dmux_pamf_state::checking_event_queue: goto label3_checking_event_queue_state;
	case dmux_pamf_state::waiting_for_event: goto label4_waiting_for_event_state;
	case dmux_pamf_state::starting_demux_done: goto label5_starting_demux_done_state;
	case dmux_pamf_state::starting_demux_done_mutex_lock_error: goto label6_starting_demux_done_mutex_lock_error_state;
	case dmux_pamf_state::starting_demux_done_mutex_unlock_error: goto label7_starting_demux_done_mutex_unlock_error_state;
	case dmux_pamf_state::starting_demux_done_checking_stream_reset: goto label8_starting_demux_done_check_stream_reset_state;
	case dmux_pamf_state::starting_demux_done_checking_stream_reset_error: goto label9_start_demux_done_check_stream_reset_error_state;
	case dmux_pamf_state::setting_au_reset: goto label10_setting_au_reset_state;
	case dmux_pamf_state::setting_au_reset_error: goto label11_setting_au_reset_error_state;
	case dmux_pamf_state::processing_event: goto label12_processing_event_state;
	case dmux_pamf_state::au_found_waiting_for_spu: goto label13_au_found_waiting_for_spu_state;
	case dmux_pamf_state::unsetting_au_cancel: goto label14_unsetting_au_cancel_state;
	case dmux_pamf_state::demux_done_notifying: goto label15_demux_done_notifying_state;
	case dmux_pamf_state::demux_done_mutex_lock: goto label16_demux_done_mutex_lock_state;
	case dmux_pamf_state::demux_done_cond_signal: goto label17_demux_done_cond_signal_state;
	case dmux_pamf_state::resuming_demux_mutex_lock: goto label18_resuming_demux_mutex_lock_state;
	case dmux_pamf_state::resuming_demux_waiting_for_spu: goto label19_resuming_demux_waiting_for_spu_state;
	case dmux_pamf_state::sending_fatal_err:
		callback(ppu, notify_fatal_err, _this, CELL_OK);

		if (ppu.state & cpu_flag::again)
		{
			return;
		}
	}

	for (;;)
	{
		savestate = dmux_pamf_state::initial;

		stream_reset_started = false;

		// If the access unit queue of an enabled elementary stream is full, wait until the user releases an AU or requests a stream reset before processing the next event
		label1_waiting_for_au_released_state:

		if (wait_au_released_or_stream_reset(ppu, au_queue_full_bitset, stream_reset_started, savestate) != CELL_OK)
		{
			savestate = dmux_pamf_state::waiting_for_au_released_error;
			label2_waiting_for_au_released_error_state:

			callback(ppu, notify_fatal_err, _this, CELL_OK);
		}

		if (ppu.state & cpu_flag::again)
		{
			return;
		}

		// Check the event queue and wait for the next event if it is empty
		savestate = dmux_pamf_state::checking_event_queue;
		label3_checking_event_queue_state:

		// TODO Sometimes the signal flag is set here, which would cause subsequent lv2 mutex/cond syscalls to fail. Calling cpu_thread::check_state() here fixes that
		// However, this appears to cause ppu_thread::loaded_from_savestate not being set correctly when loading from a savestate
		if (ppu.check_state())
		{
			ppu.state += cpu_flag::again;
			return;
		}

		if (!event_queue.peek(event))
		{
			savestate = dmux_pamf_state::waiting_for_event;
			label4_waiting_for_event_state:

			cellDmuxPamf.trace("Waiting for the next event...");

			lv2_obj::sleep(ppu);
			event_queue.wait();

			if (ppu.check_state())
			{
				ppu.state += cpu_flag::again;
				return;
			}

			ensure(event_queue.peek(event));
		}

		cellDmuxPamf.trace("Event type: %d", static_cast<u32>(event.type.get()));

		// If the event is a demux done event, set the sequence state to resetting and check for a potential stream reset request again
		if (event.type == DmuxPamfEventType::demux_done)
		{
			savestate = dmux_pamf_state::starting_demux_done;
			label5_starting_demux_done_state:

			if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
			{
				savestate = dmux_pamf_state::starting_demux_done_mutex_lock_error;
				label6_starting_demux_done_mutex_lock_error_state:

				callback(ppu, notify_fatal_err, _this, CELL_OK);
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			sequence_state = DmuxPamfSequenceState::resetting;

			if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
			{
				savestate = dmux_pamf_state::starting_demux_done_mutex_unlock_error;
				label7_starting_demux_done_mutex_unlock_error_state:

				callback(ppu, notify_fatal_err, _this, CELL_OK);

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}

			if (!stream_reset_started)
			{
				savestate = dmux_pamf_state::starting_demux_done_checking_stream_reset;
				label8_starting_demux_done_check_stream_reset_state:

				if (wait_au_released_or_stream_reset(ppu, 0, stream_reset_started, savestate) != CELL_OK)
				{
					savestate = dmux_pamf_state::starting_demux_done_checking_stream_reset_error;
					label9_start_demux_done_check_stream_reset_error_state:

					callback(ppu, notify_fatal_err, _this, CELL_OK);
				}

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}
		}

		// If the user requested a stream reset, set the reset flag for every enabled elementary stream
		if (stream_reset_started)
		{
			stream_reset_in_progress = true;

			savestate = dmux_pamf_state::setting_au_reset;
			label10_setting_au_reset_state:

			if (set_au_reset<true>(ppu) != CELL_OK)
			{
				savestate = dmux_pamf_state::setting_au_reset_error;
				label11_setting_au_reset_error_state:

				callback(ppu, notify_fatal_err, _this, CELL_OK);
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}
		}

		savestate = dmux_pamf_state::processing_event;
		label12_processing_event_state:

		switch (event.type)
		{
		case DmuxPamfEventType::au_found:
		{
			if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			label13_au_found_waiting_for_spu_state:

			DmuxPamfElementaryStream* const es = find_es(event.au_found.stream_id, event.au_found.private_stream_id);

			// If the elementary stream of the found access unit is not enabled, don't do anything
			if (!es || es->_this.get_ptr() != es || es->es_id != event.au_found.es_id)
			{
				if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
				{
					SEND_FATAL_ERR_AND_CONTINUE()
				}

				break;
			}

			// If a stream reset was requested, don't notify the user of any found access units that are still in the event queue
			if (!stream_reset_in_progress)
			{
				const vm::var<DmuxPamfAuInfo> au_info;
				au_info->addr = std::bit_cast<vm::bptr<void, u64>>(event.au_found.au_addr);
				au_info->size = event.au_found.au_size;
				au_info->pts = event.au_found.pts;
				au_info->dts = event.au_found.dts;
				au_info->user_data = user_data;
				au_info->specific_info = es->_this.ptr(&DmuxPamfElementaryStream::au_specific_info);
				au_info->specific_info_size = es->au_specific_info_size;
				au_info->is_rap = static_cast<b8>(event.au_found.is_rap);

				if (!is_raw_es)
				{
					if (dmuxPamfStreamIdToTypeChannel(event.au_found.stream_id, event.au_found.private_stream_id).first == DMUX_PAMF_STREAM_TYPE_INDEX_LPCM)
					{
						es->au_specific_info[0] = read_from_ptr<u8>(event.au_found.stream_header_buf) >> 4;
						es->au_specific_info[1] = read_from_ptr<u8>(event.au_found.stream_header_buf) & 0xf;
						es->au_specific_info[2] = read_from_ptr<u8>(&event.au_found.stream_header_buf[1]) >> 6;
					}
				}

				if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
				{
					SEND_FATAL_ERR_AND_CONTINUE()
				}

				if (callback(ppu, es->notify_au_found, es->_this, au_info) != CELL_OK)
				{
					// If the callback returns an error, the access unit queue for this elementary stream is full
					au_queue_full_bitset |= 1ull << es->this_index;
					continue;
				}

				if (ppu.state & cpu_flag::again)
				{
					return;
				}

				break;
			}

			// After a stream reset request, since the SPU thread doesn't know which access units were actually received by the user,
			// we need to send it the start address of the first found access unit for each elementary stream still in the event queue,
			// so that it can reset its internal pointers to the state just before the stream reset request.
			if (es->reset_next_au)
			{
				send_spu_command_and_wait<DmuxPamfCommandType::reset_es>(ppu, savestate == dmux_pamf_state::au_found_waiting_for_spu, event.au_found.stream_id, event.au_found.private_stream_id, event.au_found.au_addr);

				if (ppu.state & cpu_flag::again)
				{
					savestate = dmux_pamf_state::au_found_waiting_for_spu;
					return;
				}

				es->reset_next_au = false;
			}

			if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			break;
		}
		case DmuxPamfEventType::demux_done:
		{
			if (stream_reset_in_progress)
			{
				stream_reset_in_progress = false;

				savestate = dmux_pamf_state::unsetting_au_cancel;
				label14_unsetting_au_cancel_state:

				if (set_au_reset<false>(ppu) != CELL_OK)
				{
					SEND_FATAL_ERR_AND_CONTINUE()
				}

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}

			savestate = dmux_pamf_state::demux_done_notifying;
			label15_demux_done_notifying_state:

			callback(ppu, notify_demux_done, _this, CELL_OK);

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			savestate = dmux_pamf_state::demux_done_mutex_lock;
			label16_demux_done_mutex_lock_state:

			if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			if (sequence_state == DmuxPamfSequenceState::resetting)
			{
				sequence_state = DmuxPamfSequenceState::dormant;

				savestate = dmux_pamf_state::demux_done_cond_signal;
				label17_demux_done_cond_signal_state:

				if (sys_cond_signal_all(ppu, cond) != CELL_OK)
				{
					sys_mutex_unlock(ppu, mutex);
					SEND_FATAL_ERR_AND_CONTINUE()
				}

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}

			if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			break;
		}
		case DmuxPamfEventType::close:
		{
			while (event_queue.pop()){} // Empty the event queue
			return;
		}
		case DmuxPamfEventType::flush_done:
		{
			if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			const auto es = find_es(event.flush_done.stream_id, event.flush_done.private_stream_id);
			const bool valid = es && es->_this.get_ptr() == es && es->es_id == event.flush_done.es_id;

			if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			if (valid)
			{
				callback(ppu, es->notify_flush_done, es->_this);

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}

			break;
		}
		case DmuxPamfEventType::prog_end_code:
		{
			callback(ppu, notify_prog_end_code, _this);

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			break;
		}
		case DmuxPamfEventType::fatal_error:
		{
			ensure(event_queue.pop());

			SEND_FATAL_ERR_AND_CONTINUE()
		}
		default:
			fmt::throw_exception("Invalid event");
		}

		ensure(event_queue.pop());

		// If there are too many events enqueued, the SPU thread will stop demuxing until it receives a new command.
		// Once the event queue size is reduced to two, send a resume command
		if (enabled_es_num >= 0 && event_queue.size() == 2)
		{
			savestate = dmux_pamf_state::resuming_demux_mutex_lock;
			label18_resuming_demux_mutex_lock_state:

			if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}

			if (ppu.state & cpu_flag::again)
			{
				return;
			}

			if (enabled_es_num >= 0)
			{
				ensure(cmd_queue.emplace(DmuxPamfCommandType::resume));

				savestate = dmux_pamf_state::resuming_demux_waiting_for_spu;
				label19_resuming_demux_waiting_for_spu_state:

				lv2_obj::sleep(ppu);
				cmd_result_queue.wait();

				if (ppu.check_state())
				{
					ppu.state += cpu_flag::again;
					return;
				}

				ensure(cmd_result_queue.pop());
			}

			if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
			{
				SEND_FATAL_ERR_AND_CONTINUE()
			}
		}

		au_queue_full_bitset = 0;
	}
}

void dmuxPamfEntry(ppu_thread& ppu, vm::ptr<DmuxPamfContext> dmux)
{
	dmux->exec(ppu);

	if (ppu.state & cpu_flag::again)
	{
		ppu.syscall_args[0] = dmux.addr();
		return;
	}

	ppu_execute<&sys_ppu_thread_exit>(ppu, CELL_OK);
}

error_code dmuxPamfVerifyEsSpecificInfo(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info)
{
	// The meaning of error code value 5 in here is inconsistent with how it's used elsewhere for some reason

	if (!es_specific_info)
	{
		return CELL_OK;
	}

	switch (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		if (is_avc)
		{
			if (const u32 level = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoAvc>(es_specific_info)->level;
				level != CELL_DMUX_PAMF_AVC_LEVEL_2P1 && level != CELL_DMUX_PAMF_AVC_LEVEL_3P0 && level != CELL_DMUX_PAMF_AVC_LEVEL_3P1 && level != CELL_DMUX_PAMF_AVC_LEVEL_3P2 && level != CELL_DMUX_PAMF_AVC_LEVEL_4P1 && level != CELL_DMUX_PAMF_AVC_LEVEL_4P2)
			{
				return 5;
			}
		}
		else if (vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoM2v>(es_specific_info)->profileLevel > CELL_DMUX_PAMF_M2V_MP_HL)
		{
			return 5;
		}

		return CELL_OK;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
	{
		if (const auto [sampling_freq, nch, bps] = *vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info);
			sampling_freq != CELL_DMUX_PAMF_FS_48K || (nch != 1u && nch != 2u && nch != 6u && nch != 8u) || (bps != CELL_DMUX_PAMF_BITS_PER_SAMPLE_16 && bps != CELL_DMUX_PAMF_BITS_PER_SAMPLE_24))
		{
			return 5;
		}

		return CELL_OK;
	}
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
		return CELL_OK;

	default:
		return 5;
	}
}

template <bool raw_es>
u32 dmuxPamfGetAuSpecificInfoSize(u16 stream_id, u16 private_stream_id, bool is_avc)
{
	if constexpr (raw_es)
	{
		return 0;
	}

	switch (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		if (is_avc)
		{
			return 4;
		}

		return 0;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
		return 3;

	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
	default:
		return 0;
	}
}

u32 dmuxPamfGetAuQueueMaxSize(u16 stream_id, u16 private_stream_id)
{
	switch (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
		return 0x100;

	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
		return 0x40;

	default:
		return 0;
	}
}

u32 dmuxPamfGetLpcmAuSize(vm::cptr<CellDmuxPamfEsSpecificInfoLpcm> lpcm_info)
{
	return lpcm_info->samplingFreq * lpcm_info->bitsPerSample * (lpcm_info->numOfChannels + (lpcm_info->numOfChannels & 1) /* streams with an odd number of channels contain an empty dummy channel */) / 1600;
}

u32 dmuxPamfGetAuQueueBufferSize(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info)
{
	switch (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		if (is_avc)
		{
			if (!es_specific_info)
			{
				return 0x46a870;
			}

			switch (vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoAvc>(es_specific_info)->level)
			{
			case CELL_DMUX_PAMF_AVC_LEVEL_2P1: return 0xb00c0;
			case CELL_DMUX_PAMF_AVC_LEVEL_3P0: return 0x19f2e0;
			case CELL_DMUX_PAMF_AVC_LEVEL_3P1: return 0x260120;
			case CELL_DMUX_PAMF_AVC_LEVEL_3P2: return 0x35f6c0;
			case CELL_DMUX_PAMF_AVC_LEVEL_4P1: return 0x45e870;
			case CELL_DMUX_PAMF_AVC_LEVEL_4P2: // Same as below
			default:                           return 0x46a870;
			}
		}

		if (es_specific_info && vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoM2v>(es_specific_info)->profileLevel > CELL_DMUX_PAMF_M2V_MP_ML)
		{
			return 0x255000;
		}

		return 0x70000;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
	{
		if (!es_specific_info)
		{
			return 0x104380;
		}

		const u32 nch = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info)->numOfChannels;

		if (vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info)->samplingFreq <= 96000)
		{
			if (nch > 0 && nch <= 2)
			{
				return 0x20000 + dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
			}

			if (nch <= 6)
			{
				return 0x60000 + dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
			}

			if (nch <= 8)
			{
				return 0x80000 + dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
			}

			return dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
		}

		if (nch > 0 && nch <= 2)
		{
			return 0x60000 + dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
		}

		if (nch <= 6)
		{
			return 0x100000 + dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
		}

		return dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
	}
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
		return 0xa000;

	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
		return 0x6400;

	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
		return 0x160000;

	default:
		return 0;
	}
}

template <bool raw_es>
u32 dmuxPamfGetEsMemSize(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info)
{
	return dmuxPamfGetAuSpecificInfoSize<raw_es>(stream_id, private_stream_id, is_avc) * dmuxPamfGetAuQueueMaxSize(stream_id, private_stream_id)
		+ dmuxPamfGetAuQueueBufferSize(stream_id, private_stream_id, is_avc, es_specific_info) + 0x7f + static_cast<u32>(sizeof(DmuxPamfElementaryStream)) + 0xf;
}

error_code dmuxPamfNotifyDemuxDone(ppu_thread& ppu, [[maybe_unused]] vm::ptr<void> core_handle, error_code error, vm::ptr<CellDmuxPamfHandle> handle)
{
	handle->notify_demux_done.cbFunc(ppu, handle, error, handle->notify_demux_done.cbArg);
	return CELL_OK;
}

error_code dmuxPamfNotifyProgEndCode(ppu_thread& ppu, [[maybe_unused]] vm::ptr<void> core_handle, vm::ptr<CellDmuxPamfHandle> handle)
{
	if (handle->notify_prog_end_code.cbFunc)
	{
		handle->notify_prog_end_code.cbFunc(ppu, handle, handle->notify_prog_end_code.cbArg);
	}

	return CELL_OK;
}

error_code dmuxPamfNotifyFatalErr(ppu_thread& ppu, [[maybe_unused]] vm::ptr<void> core_handle, error_code error, vm::ptr<CellDmuxPamfHandle> handle)
{
	handle->notify_fatal_err.cbFunc(ppu, handle, error, handle->notify_fatal_err.cbArg);
	return CELL_OK;
}

error_code dmuxPamfEsNotifyAuFound(ppu_thread& ppu, [[maybe_unused]] vm::ptr<void> core_handle, vm::cptr<DmuxPamfAuInfo> au_info, vm::ptr<CellDmuxPamfEsHandle> handle)
{
	const vm::var<DmuxAuInfo> _au_info;
	_au_info->info.auAddr = au_info->addr;
	_au_info->info.auSize = au_info->size;
	_au_info->info.isRap = au_info->is_rap;
	_au_info->info.userData = au_info->user_data;
	_au_info->info.pts = au_info->pts;
	_au_info->info.dts = au_info->dts;
	_au_info->specific_info = au_info->specific_info;
	_au_info->specific_info_size = au_info->specific_info_size;

	return handle->notify_au_found.cbFunc(ppu, handle, _au_info, handle->notify_au_found.cbArg);
}

error_code dmuxPamfEsNotifyFlushDone(ppu_thread& ppu, [[maybe_unused]] vm::ptr<void> core_handle, vm::ptr<CellDmuxPamfEsHandle> handle)
{
	return handle->notify_flush_done.cbFunc(ppu, handle, handle->notify_flush_done.cbArg);
}

error_code _CellDmuxCoreOpQueryAttr(vm::cptr<CellDmuxPamfSpecificInfo> pamfSpecificInfo, vm::ptr<CellDmuxPamfAttr> pamfAttr)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpQueryAttr(pamfSpecificInfo=*0x%x, pamfAttr=*0x%x)", pamfSpecificInfo, pamfAttr);

	if (!pamfAttr || (pamfSpecificInfo && pamfSpecificInfo->thisSize != sizeof(CellDmuxPamfSpecificInfo)))
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	pamfAttr->maxEnabledEsNum = DMUX_PAMF_MAX_ENABLED_ES_NUM;
	pamfAttr->version = DMUX_PAMF_VERSION;
	pamfAttr->memSize = sizeof(CellDmuxPamfHandle) + sizeof(DmuxPamfContext) + 0xe7b;

	return CELL_OK;
}

error_code DmuxPamfContext::open(ppu_thread& ppu, const CellDmuxPamfResource& res, const DmuxCb<DmuxNotifyDemuxDone>& notify_dmux_done, const DmuxCb<DmuxNotifyProgEndCode>& notify_prog_end_code, const DmuxCb<DmuxNotifyFatalErr>& notify_fatal_err, vm::bptr<DmuxPamfContext>& handle)
{
	if (res.ppuThreadPriority >= 0xc00u || res.ppuThreadStackSize < 0x1000u || res.spuThreadPriority >= 0x100u || res.numOfSpus != 1u || !res.memAddr || res.memSize < sizeof(DmuxPamfContext) + 0xe7b)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	const auto _this = vm::ptr<DmuxPamfContext>::make(utils::align(+res.memAddr.addr(), 0x80));

	_this->_this = _this;
	_this->this_size = res.memSize;
	_this->version = DMUX_PAMF_VERSION;
	_this->notify_demux_done = notify_dmux_done;
	_this->notify_prog_end_code = notify_prog_end_code;
	_this->notify_fatal_err = notify_fatal_err;
	_this->resource = res;
	_this->unk = 0;
	_this->ppu_thread_stack_size = res.ppuThreadStackSize;
	_this->au_released_bitset = 0;
	_this->stream_reset_requested = false;
	_this->sequence_state = DmuxPamfSequenceState::dormant;
	_this->max_enabled_es_num = DMUX_PAMF_MAX_ENABLED_ES_NUM;
	_this->enabled_es_num = 0;
	std::ranges::fill(_this->elementary_streams, vm::null);
	_this->next_es_id = 0;

	const vm::var<sys_mutex_attribute_t> mutex_attr = {{ SYS_SYNC_PRIORITY, SYS_SYNC_NOT_RECURSIVE, SYS_SYNC_NOT_PROCESS_SHARED, SYS_SYNC_NOT_ADAPTIVE, 0, 0, 0, { "_dxpmtx"_u64 } }};
	const vm::var<sys_cond_attribute_t> cond_attr = {{ SYS_SYNC_NOT_PROCESS_SHARED, 0, 0, { "_dxpcnd"_u64 } }};

	if (sys_mutex_create(ppu, _this.ptr(&DmuxPamfContext::mutex), mutex_attr) != CELL_OK
		|| sys_cond_create(ppu, _this.ptr(&DmuxPamfContext::cond), _this->mutex, cond_attr) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	_this->spurs_context_addr = _this.ptr(&DmuxPamfContext::spurs_context);
	_this->cmd_queue_addr_ = _this.ptr(&DmuxPamfContext::cmd_queue);
	_this->cmd_queue_buffer_addr_ = _this.ptr(&DmuxPamfContext::cmd_queue_buffer);
	_this->cmd_queue_addr = _this.ptr(&DmuxPamfContext::cmd_queue);
	_this->cmd_result_queue_addr = _this.ptr(&DmuxPamfContext::cmd_result_queue);
	_this->stream_info_queue_addr = _this.ptr(&DmuxPamfContext::stream_info_queue);
	_this->event_queue_addr = _this.ptr(&DmuxPamfContext::event_queue);
	_this->cmd_queue_buffer_addr = _this.ptr(&DmuxPamfContext::cmd_queue_buffer);
	_this->cmd_result_queue_buffer_addr = _this.ptr(&DmuxPamfContext::cmd_result_queue_buffer);
	_this->event_queue_buffer_addr = _this.ptr(&DmuxPamfContext::event_queue_buffer);
	_this->stream_info_queue_buffer_addr = _this.ptr(&DmuxPamfContext::stream_info_queue_buffer);
	_this->cmd_queue_addr__ = _this.ptr(&DmuxPamfContext::cmd_queue);

	ensure(std::snprintf(_this->spurs_taskset_name, sizeof(_this->spurs_taskset_name), "_libdmux_pamf_%08x", _this.addr()) == 22);

	_this->cmd_queue.init(_this->cmd_queue_buffer);
	_this->cmd_result_queue.init(_this->cmd_result_queue_buffer);
	_this->stream_info_queue.init(_this->stream_info_queue_buffer);
	_this->event_queue.init(_this->event_queue_buffer);

	// HLE exclusive
	_this->savestate = {};
	_this->au_queue_full_bitset = 0;
	_this->stream_reset_started = false;
	_this->stream_reset_in_progress = false;

	_this->run_spu_thread();

	handle = _this;
	return _this->create_thread(ppu);
}

error_code _CellDmuxCoreOpOpen(ppu_thread& ppu, vm::cptr<CellDmuxPamfSpecificInfo> pamfSpecificInfo, vm::cptr<CellDmuxResource> demuxerResource, vm::cptr<CellDmuxResourceSpurs> demuxerResourceSpurs, vm::cptr<DmuxCb<DmuxNotifyDemuxDone>> notifyDemuxDone,
	vm::cptr<DmuxCb<DmuxNotifyProgEndCode>> notifyProgEndCode, vm::cptr<DmuxCb<DmuxNotifyFatalErr>> notifyFatalErr, vm::pptr<CellDmuxPamfHandle> handle)
{
	// Block savestates during ppu_execute
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpOpen(pamfSpecificInfo=*0x%x, demuxerResource=*0x%x, demuxerResourceSpurs=*0x%x, notifyDemuxDone=*0x%x, notifyProgEndCode=*0x%x, notifyFatalErr=*0x%x, handle=**0x%x)",
		pamfSpecificInfo, demuxerResource, demuxerResourceSpurs, notifyDemuxDone, notifyProgEndCode, notifyFatalErr, handle);

	if ((pamfSpecificInfo && pamfSpecificInfo->thisSize != sizeof(CellDmuxPamfSpecificInfo))
		|| !demuxerResource
		|| (demuxerResourceSpurs && !demuxerResourceSpurs->spurs)
		|| !notifyDemuxDone || !notifyDemuxDone->cbFunc || !notifyDemuxDone->cbArg
		|| !notifyProgEndCode
		|| !notifyFatalErr || !notifyFatalErr->cbFunc || !notifyFatalErr->cbArg
		|| !handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(demuxerResource->memAddr.aligned(0x10)); // Not checked on LLE

	const auto _handle = vm::static_ptr_cast<CellDmuxPamfHandle>(demuxerResource->memAddr);

	_handle->notify_demux_done = *notifyDemuxDone;
	_handle->notify_fatal_err = *notifyFatalErr;
	_handle->notify_prog_end_code = *notifyProgEndCode;

	if (!pamfSpecificInfo || !pamfSpecificInfo->programEndCodeCb)
	{
		_handle->notify_prog_end_code.cbFunc = vm::null;
	}

	const CellDmuxPamfResource res{ demuxerResource->ppuThreadPriority, demuxerResource->ppuThreadStackSize, demuxerResource->numOfSpus, demuxerResource->spuThreadPriority,
		vm::bptr<void>::make(demuxerResource->memAddr.addr() + sizeof(CellDmuxPamfHandle)), demuxerResource->memSize - sizeof(CellDmuxPamfHandle) };

	const auto demux_done_func = vm::bptr<DmuxNotifyDemuxDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyDemuxDone)));
	const auto prog_end_code_func = vm::bptr<DmuxNotifyProgEndCode>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyProgEndCode)));
	const auto fatal_err_func = vm::bptr<DmuxNotifyFatalErr>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyFatalErr)));

	const error_code ret = DmuxPamfContext::open(ppu, res, { demux_done_func, _handle }, { prog_end_code_func, _handle }, { fatal_err_func, _handle }, _handle->demuxer);

	*handle = _handle;

	return ret;
}

error_code DmuxPamfContext::close(ppu_thread& ppu)
{
	if (join_thread(ppu) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	ensure(idm::remove<dmux_pamf_thread>(hle_spu_thread_id));

	return CELL_OK;
}

error_code _CellDmuxCoreOpClose(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	// The PPU thread is going to use ppu_execute
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpClose(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return handle->demuxer->close(ppu);
}

error_code DmuxPamfContext::reset_stream(ppu_thread& ppu)
{
	auto& ar = *ppu.optional_savestate_state;
	const u8 savestate = ar.try_read<u8>().second;
	ar.clear();

	switch (savestate)
	{
	case 0: break;
	case 1: goto label1_wait_for_spu_state;
	case 2: goto label2_cond_signal_state;
	default: fmt::throw_exception("Unexpected savestate value: 0x%x", savestate);
	}

	if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(0);
		return {};
	}

	if (sequence_state != DmuxPamfSequenceState::running)
	{
		return sys_mutex_unlock(ppu, mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
	}

	label1_wait_for_spu_state:
	send_spu_command_and_wait<DmuxPamfCommandType::reset_stream>(ppu, savestate);

	if (ppu.state & cpu_flag::again)
	{
		ar(1);
		return {};
	}

	stream_reset_requested = true;

	label2_cond_signal_state:
	if (error_code ret = sys_cond_signal_to(ppu, cond, static_cast<u32>(thread_id)); ret != CELL_OK && ret != static_cast<s32>(CELL_EPERM))
	{
		sys_mutex_unlock(ppu, mutex);
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(2);
		return {};
	}

	return sys_mutex_unlock(ppu, mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpResetStream(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpResetStream(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return handle->demuxer->reset_stream(ppu);
}

error_code DmuxPamfContext::create_thread(ppu_thread& ppu)
{
	const vm::var<char[]> name = vm::make_str("HLE PAMF demuxer");
	const auto entry = g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEntry));

	if (ppu_execute<&sys_ppu_thread_create>(ppu, _this.ptr(&DmuxPamfContext::thread_id), entry, +_this.addr(), +resource.ppuThreadPriority, +resource.ppuThreadStackSize, SYS_PPU_THREAD_CREATE_JOINABLE, +name) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	return CELL_OK;
}

error_code _CellDmuxCoreOpCreateThread(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	// Block savestates during ppu_execute
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpCreateThread(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return handle->demuxer->create_thread(ppu);
}

error_code DmuxPamfContext::join_thread(ppu_thread& ppu)
{
	if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	std::ranges::fill_n(elementary_streams, enabled_es_num, vm::null);

	enabled_es_num = -1;

	send_spu_command_and_wait<DmuxPamfCommandType::close>(ppu, false);

	if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	return sys_ppu_thread_join(ppu, static_cast<u32>(thread_id), +vm::var<u64>{}) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpJoinThread(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	// The PPU thread is going to use ppu_execute
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpJoinThread(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return handle->demuxer->join_thread(ppu);
}

template <bool raw_es>
error_code DmuxPamfContext::set_stream(ppu_thread& ppu, vm::cptr<std::byte> stream_address, u32 stream_size, b8 discontinuity, u32 user_data)
{
	auto& ar = *ppu.optional_savestate_state;
	const bool waiting_for_spu_state = ar.try_read<bool>().second;
	ar.clear();

	if (!waiting_for_spu_state)
	{
		if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
		{
			return CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (ppu.state & cpu_flag::again)
		{
			ar(false);
			return {};
		}

		this->user_data = user_data;

		if (!stream_info_queue.emplace(stream_address, stream_size, user_data, !discontinuity, raw_es))
		{
			return sys_mutex_unlock(ppu, mutex) == CELL_OK ? CELL_DMUX_PAMF_ERROR_BUSY : CELL_DMUX_PAMF_ERROR_FATAL;
		}
	}

	send_spu_command_and_wait<DmuxPamfCommandType::set_stream>(ppu, waiting_for_spu_state);

	if (ppu.state & cpu_flag::again)
	{
		ar(true);
		return {};
	}

	sequence_state = DmuxPamfSequenceState::running;

	return sys_mutex_unlock(ppu, mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

template <bool raw_es>
error_code _CellDmuxCoreOpSetStream(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle, vm::cptr<std::byte> streamAddress, u32 streamSize, b8 discontinuity, u64 userData)
{
	cellDmuxPamf.trace("_CellDmuxCoreOpSetStream<raw_es=%d>(handle=*0x%x, streamAddress=*0x%x, streamSize=0x%x, discontinuity=%d, userData=0x%llx)", raw_es, handle, streamAddress, streamSize, +discontinuity, userData);

	if (!streamAddress || streamSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!handle); // Not checked on LLE

	return handle->demuxer->set_stream<raw_es>(ppu, streamAddress, streamSize, discontinuity, static_cast<u32>(userData));
}

error_code DmuxPamfElementaryStream::release_au(ppu_thread& ppu, vm::ptr<std::byte> au_addr, u32 au_size) const
{
	auto& ar = *ppu.optional_savestate_state;
	const u8 savestate = ar.try_read<u8>().second;
	ar.clear();

	switch (savestate)
	{
	case 0: break;
	case 1: goto label1_wait_for_spu_state;
	case 2: goto label2_cond_signal_state;
	default: fmt::throw_exception("Unexpected savestate value: 0x%x", savestate);
	}

	if (sys_mutex_lock(ppu, demuxer->mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(0);
		return {};
	}

	label1_wait_for_spu_state:
	demuxer->send_spu_command_and_wait<DmuxPamfCommandType::release_au>(ppu, savestate, au_addr, au_size, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id));

	if (ppu.state & cpu_flag::again)
	{
		ar(1);
		return {};
	}

	demuxer->au_released_bitset |= 1ull << this_index;

	label2_cond_signal_state:
	if (error_code ret = sys_cond_signal_to(ppu, demuxer->cond, static_cast<u32>(demuxer->thread_id)); ret != CELL_OK && ret != static_cast<s32>(CELL_EPERM))
	{
		sys_mutex_unlock(ppu, demuxer->mutex);
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(2);
		return {};
	}

	return sys_mutex_unlock(ppu, demuxer->mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpReleaseAu(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle, vm::ptr<std::byte> auAddr, u32 auSize)
{
	cellDmuxPamf.trace("_CellDmuxCoreOpFreeMemory(esHandle=*0x%x, auAddr=*0x%x, auSize=0x%x)", esHandle, auAddr, auSize);

	if (!auAddr || auSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!esHandle); // Not checked on LLE

	return esHandle->es->release_au(ppu, auAddr, auSize);
}

template <bool raw_es>
error_code dmuxPamfGetEsAttr(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, CellDmuxPamfEsAttr& attr)
{
	if (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM;
	}

	if (dmuxPamfVerifyEsSpecificInfo(stream_id, private_stream_id, is_avc, es_specific_info) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	attr.auQueueMaxSize = dmuxPamfGetAuQueueMaxSize(stream_id, private_stream_id);
	attr.memSize = dmuxPamfGetEsMemSize<raw_es>(stream_id, private_stream_id, is_avc, es_specific_info);
	attr.specificInfoSize = dmuxPamfGetAuSpecificInfoSize<raw_es>(stream_id, private_stream_id, is_avc);

	return CELL_OK;
}

template <bool raw_es>
static inline std::tuple<u16, u16, bool> get_stream_ids(vm::cptr<void> esFilterId)
{
	if constexpr (raw_es)
	{
		const auto filter_id = vm::static_ptr_cast<const u8>(esFilterId);
		return { filter_id[2], filter_id[3], filter_id[8] >> 7 };
	}

	const auto filter_id = vm::static_ptr_cast<const CellCodecEsFilterId>(esFilterId);
	return { filter_id->filterIdMajor, filter_id->filterIdMinor, filter_id->supplementalInfo1 };
}

template <bool raw_es>
error_code _CellDmuxCoreOpQueryEsAttr(vm::cptr<void> esFilterId, vm::cptr<void> esSpecificInfo, vm::ptr<CellDmuxPamfEsAttr> attr)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpQueryEsAttr<raw_es=%d>(esFilterId=*0x%x, esSpecificInfo=*0x%x, attr=*0x%x)", raw_es, esFilterId, esSpecificInfo, attr);

	if (!esFilterId || !attr)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	const auto [stream_id, private_stream_id, is_avc] = get_stream_ids<raw_es>(esFilterId);

	CellDmuxPamfEsAttr es_attr;

	const error_code ret = dmuxPamfGetEsAttr<raw_es>(stream_id, private_stream_id, is_avc, esSpecificInfo, es_attr);

	*attr = es_attr;
	attr->memSize += static_cast<u32>(sizeof(CellDmuxPamfEsHandle));

	return ret;
}

template <bool raw_es>
error_code DmuxPamfContext::enable_es(ppu_thread& ppu, u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, vm::ptr<void> mem_addr, u32 mem_size, const DmuxCb<DmuxEsNotifyAuFound>& notify_au_found,
	const DmuxCb<DmuxEsNotifyFlushDone>& notify_flush_done, vm::bptr<DmuxPamfElementaryStream>& es)
{
	auto& ar = *ppu.optional_savestate_state;
	const bool waiting_for_spu_state = ar.try_read<bool>().second;
	ar.clear();

	if (mem_size < dmuxPamfGetEsMemSize<raw_es>(stream_id, private_stream_id, is_avc, es_specific_info))
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	const auto stream_type = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first;

	if (!waiting_for_spu_state)
	{
		if (stream_type == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
		{
			return CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM;
		}

		if (dmuxPamfVerifyEsSpecificInfo(stream_id, private_stream_id, is_avc, es_specific_info) != CELL_OK)
		{
			return CELL_DMUX_PAMF_ERROR_ARG;
		}

		if (error_code ret = sys_mutex_lock(ppu, mutex, 0); ret != CELL_OK)
		{
			return CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (ppu.state & cpu_flag::again)
		{
			ar(false);
			return {};
		}

		this->is_raw_es = raw_es;

		if (enabled_es_num == max_enabled_es_num)
		{
			return sys_mutex_unlock(ppu, mutex) == CELL_OK ? CELL_DMUX_PAMF_ERROR_NO_MEMORY : CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (find_es(stream_id, private_stream_id))
		{
			// Elementary stream is already enabled
			return sys_mutex_unlock(ppu, mutex) == CELL_OK ? CELL_DMUX_PAMF_ERROR_ARG : CELL_DMUX_PAMF_ERROR_FATAL;
		}
	}

	const be_t<u32> au_max_size = [&]() -> be_t<u32>
	{
		switch (stream_type)
		{
		case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
			if (is_avc)
			{
				if (!es_specific_info || vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoAvc>(es_specific_info)->level == CELL_DMUX_PAMF_AVC_LEVEL_4P2)
				{
					return 0xcc000u;
				}

				switch (vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoAvc>(es_specific_info)->level)
				{
				case CELL_DMUX_PAMF_AVC_LEVEL_2P1: return 0x12900u;
				case CELL_DMUX_PAMF_AVC_LEVEL_3P0: return 0x25f80u;
				case CELL_DMUX_PAMF_AVC_LEVEL_3P1: return 0x54600u;
				case CELL_DMUX_PAMF_AVC_LEVEL_3P2: return 0x78000u;
				case CELL_DMUX_PAMF_AVC_LEVEL_4P1: return 0xc0000u;
				default: fmt::throw_exception("Unreachable"); // es_specific_info was already checked for invalid values in dmuxPamfVerifyEsSpecificInfo()
				}
			}

			if (!es_specific_info || vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoM2v>(es_specific_info)->profileLevel > CELL_DMUX_PAMF_M2V_MP_ML)
			{
				return 0x12a800u;
			}

			return 0x38000u;

		case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:      return dmuxPamfGetLpcmAuSize(vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info));
		case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:       return 0xf00u;
		case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:    return 0x1008u;
		case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA: return 0xa0000u;
		default: fmt::throw_exception("Unreachable"); // stream_type was already checked
		}
	}();

	const auto _es = vm::bptr<DmuxPamfElementaryStream>::make(utils::align(mem_addr.addr(), 0x10));

	const auto au_queue_buffer = vm::bptr<std::byte>::make(utils::align(_es.addr() + static_cast<u32>(sizeof(DmuxPamfElementaryStream)), 0x80));
	const be_t<u32> au_specific_info_size = dmuxPamfGetAuSpecificInfoSize<raw_es>(stream_id, private_stream_id, is_avc);

	send_spu_command_and_wait<DmuxPamfCommandType::enable_es>(ppu, waiting_for_spu_state, stream_id, private_stream_id, is_avc, au_queue_buffer,
		dmuxPamfGetAuQueueBufferSize(stream_id, private_stream_id, is_avc, es_specific_info), au_max_size, au_specific_info_size, raw_es, next_es_id);

	if (ppu.state & cpu_flag::again)
	{
		ar(true);
		return {};
	}

	u32 es_idx = umax;
	while (elementary_streams[++es_idx]){} // There is guaranteed to be an empty slot, this was already checked above

	_es->_this = _es;
	_es->this_size = mem_size;
	_es->this_index = es_idx;
	_es->demuxer = _this;
	_es->notify_au_found = notify_au_found;
	_es->notify_flush_done = notify_flush_done;
	_es->stream_id = stream_id;
	_es->private_stream_id = private_stream_id;
	_es->is_avc = is_avc;
	_es->au_queue_buffer = au_queue_buffer;
	_es->au_max_size = au_max_size;
	_es->au_specific_info_size = au_specific_info_size;
	_es->reset_next_au = false;
	_es->es_id = next_es_id++;

	elementary_streams[es_idx] = _es;

	enabled_es_num++;

	if (sys_mutex_unlock(ppu, mutex) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	es = _es;
	return CELL_OK;
}

template <bool raw_es>
error_code _CellDmuxCoreOpEnableEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle, vm::cptr<void> esFilterId, vm::cptr<CellDmuxEsResource> esResource, vm::cptr<DmuxCb<DmuxEsNotifyAuFound>> notifyAuFound,
	vm::cptr<DmuxCb<DmuxEsNotifyFlushDone>> notifyFlushDone, vm::cptr<void> esSpecificInfo, vm::pptr<CellDmuxPamfEsHandle> esHandle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpEnableEs<raw_es=%d>(handle=*0x%x, esFilterId=*0x%x, esResource=*0x%x, notifyAuFound=*0x%x, notifyFlushDone=*0x%x, esSpecificInfo=*0x%x, esHandle)",
		raw_es, handle, esFilterId, esResource, notifyAuFound, notifyFlushDone, esSpecificInfo, esHandle);

	if (!handle || !esFilterId || !esResource || !esResource->memAddr || esResource->memSize == 0u || !notifyAuFound || !notifyAuFound->cbFunc || !notifyAuFound->cbArg || !notifyFlushDone || !notifyFlushDone->cbFunc || !notifyFlushDone->cbArg)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!esHandle && esResource->memAddr.aligned(0x10)); // Not checked on LLE

	const auto es_handle = vm::static_ptr_cast<CellDmuxPamfEsHandle>(esResource->memAddr);

	es_handle->notify_au_found = *notifyAuFound;
	es_handle->notify_flush_done = *notifyFlushDone;

	const auto au_found_func = vm::bptr<DmuxEsNotifyAuFound>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEsNotifyAuFound)));
	const auto flush_done_func = vm::bptr<DmuxEsNotifyFlushDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEsNotifyFlushDone)));

	const auto [stream_id, private_stream_id, is_avc] = get_stream_ids<raw_es>(esFilterId);

	const error_code ret = handle->demuxer->enable_es<raw_es>(ppu, stream_id, private_stream_id, is_avc, esSpecificInfo, vm::ptr<void>::make(esResource->memAddr.addr() + sizeof(CellDmuxPamfEsHandle)),
		esResource->memSize - sizeof(CellDmuxPamfEsHandle), { au_found_func, es_handle }, { flush_done_func, es_handle }, es_handle->es);

	*esHandle = es_handle;

	return ret;
}

error_code DmuxPamfElementaryStream::disable_es(ppu_thread& ppu)
{
	const auto dmux = demuxer.get_ptr();

	auto& ar = *ppu.optional_savestate_state;
	const u8 savestate = ar.try_read<u8>().second;
	ar.clear();

	switch (savestate)
	{
	case 0: break;
	case 1: goto label1_wait_for_spu_state;
	case 2: goto label2_cond_signal_state;
	default: fmt::throw_exception("Unexpected savestate value: 0x%x", savestate);
	}

	if (sys_mutex_lock(ppu, dmux->mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(0);
		return {};
	}

	if (!dmux->find_es(stream_id, private_stream_id))
	{
		// Elementary stream is already disabled
		return sys_mutex_unlock(ppu, dmux->mutex) == CELL_OK ? CELL_DMUX_PAMF_ERROR_ARG : CELL_DMUX_PAMF_ERROR_FATAL;
	}

	label1_wait_for_spu_state:
	dmux->send_spu_command_and_wait<DmuxPamfCommandType::disable_es>(ppu, savestate, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id));

	if (ppu.state & cpu_flag::again)
	{
		ar(1);
		return {};
	}

	_this = vm::null;
	this_size = 0;
	demuxer = vm::null;
	notify_au_found = {};
	au_queue_buffer = vm::null;
	unk = 0;
	au_max_size = 0;

	dmux->elementary_streams[this_index] = vm::null;
	dmux->enabled_es_num--;

	dmux->au_released_bitset |= 1ull << this_index;

	this_index = 0;

	label2_cond_signal_state:
	if (error_code ret = sys_cond_signal_to(ppu, dmux->cond, static_cast<u32>(dmux->thread_id)); ret != CELL_OK && ret != static_cast<s32>(CELL_EPERM))
	{
		sys_mutex_unlock(ppu, dmux->mutex);
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		ar(2);
		return {};
	}

	return sys_mutex_unlock(ppu, dmux->mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpDisableEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpDisableEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return esHandle->es->disable_es(ppu);
}

error_code DmuxPamfElementaryStream::flush_es(ppu_thread& ppu) const
{
	auto& ar = *ppu.optional_savestate_state;
	const bool waiting_for_spu_state = ar.try_read<bool>().second;
	ar.clear();

	if (!waiting_for_spu_state)
	{
		if (sys_mutex_lock(ppu, demuxer->mutex, 0) != CELL_OK)
		{
			return CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (ppu.state & cpu_flag::again)
		{
			ar(false);
			return {};
		}
	}

	demuxer->send_spu_command_and_wait<DmuxPamfCommandType::flush_es>(ppu, waiting_for_spu_state, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id));

	if (ppu.state & cpu_flag::again)
	{
		ar(true);
		return {};
	}

	return sys_mutex_unlock(ppu, demuxer->mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpFlushEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpFlushEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return esHandle->es->flush_es(ppu);
}

error_code DmuxPamfElementaryStream::reset_es(ppu_thread& ppu) const
{
	auto& ar = *ppu.optional_savestate_state;
	const bool waiting_for_spu_state = ar.try_read<bool>().second;
	ar.clear();

	if (!waiting_for_spu_state)
	{
		if (sys_mutex_lock(ppu, demuxer->mutex, 0) != CELL_OK)
		{
			return CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (ppu.state & cpu_flag::again)
		{
			ar(false);
			return {};
		}
	}

	demuxer->send_spu_command_and_wait<DmuxPamfCommandType::reset_es>(ppu, waiting_for_spu_state, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id), vm::null);

	if (ppu.state & cpu_flag::again)
	{
		ar(true);
		return {};
	}

	return sys_mutex_unlock(ppu, demuxer->mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpResetEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpResetEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return esHandle->es->reset_es(ppu);
}

error_code DmuxPamfContext::reset_stream_and_wait_done(ppu_thread& ppu)
{
	// Both sys_cond_wait() and DmuxPamfContext::reset_stream() are already using ppu_thread::optional_savestate_state, so we can't save this function currently
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	if (reset_stream(ppu) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		return {};
	}

	if (sys_mutex_lock(ppu, mutex, 0) != CELL_OK)
	{
		return CELL_DMUX_PAMF_ERROR_FATAL;
	}

	if (ppu.state & cpu_flag::again)
	{
		return {};
	}

	while (sequence_state != DmuxPamfSequenceState::dormant)
	{
		if (sys_cond_wait(ppu, cond, 0) != CELL_OK)
		{
			sys_mutex_unlock(ppu, mutex);
			return CELL_DMUX_PAMF_ERROR_FATAL;
		}

		if (ppu.state & cpu_flag::again)
		{
			return {};
		}
	}

	return sys_mutex_unlock(ppu, mutex) == CELL_OK ? static_cast<error_code>(CELL_OK) : CELL_DMUX_PAMF_ERROR_FATAL;
}

error_code _CellDmuxCoreOpResetStreamAndWaitDone(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpResetStreamAndWaitDone(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return handle->demuxer->reset_stream_and_wait_done(ppu);
}

static void init_gvar(const vm::gvar<CellDmuxCoreOps>& var)
{
	var->queryAttr.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpQueryAttr)));
	var->open.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpOpen)));
	var->close.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpClose)));
	var->resetStream.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetStream)));
	var->createThread.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpCreateThread)));
	var->joinThread.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpJoinThread)));
	var->freeMemory.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpReleaseAu)));
	var->disableEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpDisableEs)));
	var->flushEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpFlushEs)));
	var->resetEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetEs)));
	var->resetStreamAndWaitDone.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetStreamAndWaitDone)));
}

DECLARE(ppu_module_manager::cellDmuxPamf)("cellDmuxPamf", []
{
	REG_VNID(cellDmuxPamf, 0x28b2b7b2, g_cell_dmux_core_ops_pamf).init = []
	{
		g_cell_dmux_core_ops_pamf->setStream.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpSetStream<false>)));
		g_cell_dmux_core_ops_pamf->queryEsAttr.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpQueryEsAttr<false>)));
		g_cell_dmux_core_ops_pamf->enableEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpEnableEs<false>)));
		init_gvar(g_cell_dmux_core_ops_pamf);
	};

	REG_VNID(cellDmuxPamf, 0x9728a0e9, g_cell_dmux_core_ops_raw_es).init = []
	{
		g_cell_dmux_core_ops_raw_es->setStream.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpSetStream<true>)));
		g_cell_dmux_core_ops_raw_es->queryEsAttr.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpQueryEsAttr<true>)));
		g_cell_dmux_core_ops_raw_es->enableEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpEnableEs<true>)));
		init_gvar(g_cell_dmux_core_ops_raw_es);
	};

	REG_HIDDEN_FUNC(_CellDmuxCoreOpQueryAttr);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpOpen);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpClose);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpResetStream);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpCreateThread);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpJoinThread);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpSetStream<false>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpSetStream<true>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpReleaseAu);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpQueryEsAttr<false>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpQueryEsAttr<true>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpEnableEs<false>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpEnableEs<true>);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpDisableEs);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpFlushEs);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpResetEs);
	REG_HIDDEN_FUNC(_CellDmuxCoreOpResetStreamAndWaitDone);

	REG_HIDDEN_FUNC(dmuxPamfNotifyDemuxDone);
	REG_HIDDEN_FUNC(dmuxPamfNotifyProgEndCode);
	REG_HIDDEN_FUNC(dmuxPamfNotifyFatalErr);
	REG_HIDDEN_FUNC(dmuxPamfEsNotifyAuFound);
	REG_HIDDEN_FUNC(dmuxPamfEsNotifyFlushDone);

	REG_HIDDEN_FUNC(dmuxPamfEntry);
});
