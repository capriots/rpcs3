#include "stdafx.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/Cell/lv2/sys_sync.h"

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

inline std::pair<DmuxPamfStreamTypeIndex, u32> dmuxPamfStreamIdToTypeChannel(u16 stream_id, u16 private_stream_id)
{
	if ((stream_id & 0xf0) == 0xe0)
	{
		return { DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO, stream_id & 0x0f };
	}

	if ((stream_id & 0xff) != 0xbd)
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
// SPU thread
//----------------------------------------------------------------------------

void dmux_pamf_base::output_queue::pop_back(u32 size)
{
	ensure(size <= back - buffer.data(), "Invalid au_size");
	back -= size;
}

void dmux_pamf_base::output_queue::pop_back(u8* addr)
{
	ensure(addr >= buffer.data() && addr < std::to_address(buffer.end()), "Invalid au_begin");

	// If au_begin is in front of the back pointer, unwrap the back pointer (there are no more access units behind the back pointer)
	if (addr > back)
	{
		wrap_pos = buffer.data();
	}

	back = addr;
}

void dmux_pamf_base::output_queue::pop_front(u32 size)
{
	front += size;

	// When "front" reaches "wrap_pos", unwrap the queue
	if (wrap_pos != buffer.data() && wrap_pos <= front)
	{
		ensure(wrap_pos == front, "Invalid au_size");
		front = buffer.data();
		wrap_pos = buffer.data();
	}
}

void dmux_pamf_base::output_queue::push(const access_unit_chunk& au_chunk)
{
	std::ranges::copy(au_chunk.cached_data, back);
	std::ranges::copy(au_chunk.data, back + au_chunk.cached_data.size());
	back += au_chunk.data.size() + au_chunk.cached_data.size();
}

void dmux_pamf_base::elementary_stream::flush_es()
{
	if (current_au.accumulated_size != 0)
	{
		au_queue.push({ {}, cache });

		current_au.accumulated_size += static_cast<u32>(cache.size());

		interface.on_au_found(stream_id, private_stream_id, user_data, { au_queue.peek_back(current_au.accumulated_size), current_au.accumulated_size }, current_au.pts, current_au.dts, current_au.rap, au_specific_info_size, current_au.au_specific_info_buf);
	}

	reset();

	while (!interface.on_flush_done(stream_id, private_stream_id, user_data)) {}
}

void dmux_pamf_base::elementary_stream::reset_es(u8* au_addr)
{
	if (!au_addr)
	{
		reset();
		au_queue.clear();
	}
	else
	{
		au_queue.pop_back(au_addr);
	}
}

void dmux_pamf_base::elementary_stream::discard_access_unit()
{
	au_queue.pop_back(current_au.accumulated_size - static_cast<u32>(au_chunk.data.size() + au_chunk.cached_data.size()));
	reset();
	cache.clear();
}

template <u32 unk_size_mask>
u32 dmux_pamf_base::elementary_stream::parse_audio_stream_header(std::span<const u8> elementary_stream)
{
	// No clue what this is, I have not found a single instance in any PAMF stream where it is something other than zero
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

	return unk_size + sizeof(u32);
}

bool dmux_pamf_base::elementary_stream::process_stream()
{
	ensure(pes_packet_data && pes_packet_data_begin, "set_pes_packet_data() should be called before process_stream()");

	for (;;)
	{
		switch (state)
		{
		case state::initial:
			if (pes_packet_data->empty())
			{
				pes_packet_data.reset();
				pes_packet_data_begin.reset();
				return true;
			}

			// Parse the current stream and increment the reading position by the amount that was consumed
			pes_packet_data = pes_packet_data->subspan(parse_stream(*pes_packet_data));

			ensure(current_au.state != access_unit::state::size_mismatch, "Size mismatch between distance of sync words and au-size specified in stream"); // TODO

			current_au.accumulated_size += static_cast<u32>(au_chunk.data.size() + au_chunk.cached_data.size());

			// If the beginning of a new access unit was found, set the current timestamps and rap indicator
			if (!current_au.timestamps_rap_set && (current_au.state == access_unit::state::commenced || current_au.state == access_unit::state::m2v_sequence || (current_au.state == access_unit::state::complete && au_chunk.cached_data.empty())))
			{
				set_au_timestamps_rap();
			}

			state = state::pushing_au_queue;
			[[fallthrough]];

		case state::pushing_au_queue:
			if (!au_chunk.data.empty() || !au_chunk.cached_data.empty())
			{
				if (!au_queue.is_wrapped_around())
				{
					// Since it was already checked if there is enough space for au_max_size, this can only occur if the current access unit is larger than au_max_size
					if (au_chunk.data.size() + au_chunk.cached_data.size() > au_queue.get_remaining_capacity())
					{
						cellDmuxPamf.error("Access unit larger than specified maximum access unit size");
						interface.on_fatal_error();
						interface.on_au_queue_full();
						return false;
					}
				}
				else if (au_chunk.data.size() + au_chunk.cached_data.size() + 0x10 > au_queue.get_remaining_capacity()) // + sizeof(v128) because of SPU shenanigans probably
				{
					interface.on_au_queue_full();
					return false;
				}

				au_queue.push(au_chunk);
				au_chunk = {};
			}

			state = state::notifying_au_found;
			[[fallthrough]];

		case state::notifying_au_found:
			if (current_au.state == access_unit::state::complete && !interface.on_au_found(stream_id, private_stream_id, user_data, { au_queue.peek_back(current_au.accumulated_size), current_au.accumulated_size },
				current_au.pts, current_au.dts, current_au.rap, au_specific_info_size, current_au.au_specific_info_buf))
			{
				return false;
			}

			state = state::waiting_for_au_released;
			[[fallthrough]];

		case state::waiting_for_au_released:
			// Check if there is enough space for the next access unit
			if (current_au.state == access_unit::state::complete && au_max_size > au_queue.get_remaining_capacity())
			{
				// If there are unconsumed access units behind the back pointer, wait until they are released by the user
				if (au_queue.is_wrapped_around())
				{
					interface.on_au_queue_full();
					return false;
				}

				au_queue.wrap_around();
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
u32 dmux_pamf_base::video_stream<avc>::parse_stream(std::span<const u8> stream)
{
	if (current_au.state != access_unit::state::none && (avc || current_au.state != access_unit::state::m2v_sequence))
	{
		current_au.state = access_unit::state::incomplete;
	}

	// Concatenate the cache of the previous stream section and the beginning of the current section
	std::array<u8, 8> buf{};
	std::ranges::copy(cache, buf.begin());
	std::copy_n(stream.begin(), std::min(sizeof(u32) - 1, stream.size()), buf.begin() + cache.size());

	auto au_chunk_begin = stream.begin();
	s32 cache_idx = 0;
	auto stream_it = stream.begin();
	[&]
	{
		// Search for delimiter in cache
		for (; cache_idx <= static_cast<s32>(cache.size() + std::min(sizeof(u32) - 1, stream.size()) - sizeof(u32)); cache_idx++)
		{
			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(buf.data() + cache_idx); (avc && code == AVC_AU_DELIMITER) || (!avc && (code == M2V_PIC_START || code == M2V_SEQUENCE_HEADER || code == M2V_SEQUENCE_END)))
			{
				if (current_au.state != access_unit::state::none && (avc || current_au.state != access_unit::state::m2v_sequence))
				{
					stream_it += !avc && code == M2V_SEQUENCE_END ? sizeof(u32) : 0;
					current_au.state = access_unit::state::complete;
					return;
				}

				// If current_au.state is none and there was a delimiter found here, then LLE outputs the entire cache, even if the access unit starts at cache_idx > 0

				current_au.state = avc || code == M2V_PIC_START ? access_unit::state::commenced : access_unit::state::m2v_sequence;
			}
		}

		// Search for delimiter in stream
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

				au_chunk_begin = avc || current_au.state == access_unit::state::none ? stream_it : au_chunk_begin;
				current_au.state = avc || code == M2V_PIC_START ? access_unit::state::commenced : access_unit::state::m2v_sequence;
			}
		}
	}();

	if (current_au.state != access_unit::state::none)
	{
		au_chunk.data = { au_chunk_begin, stream_it };
		std::copy_n(cache.begin(), cache_idx, std::back_inserter(au_chunk.cached_data));
		cache.erase(cache.begin(), cache.begin() + cache_idx);
	}

	// Cache the end of the stream if an access unit wasn't completed. There could be the beginning of a delimiter in the last three bytes
	if (current_au.state != access_unit::state::complete)
	{
		std::copy(stream_it, stream.end(), std::back_inserter(cache));
	}

	return static_cast<u32>((current_au.state != access_unit::state::complete ? stream.end() : stream_it) - stream.begin());
}

u32 dmux_pamf_base::lpcm_stream::parse_stream_header(std::span<const u8> elementary_stream, [[maybe_unused]] s8 pts_dts_flag)
{
	// Not checked on LLE
	if (elementary_stream.size() < sizeof(u8) + 0x10)
	{
		return umax;
	}

	std::memcpy(au_specific_info_buf.data(), &elementary_stream[1], au_specific_info_buf.size());
	return parse_audio_stream_header<0x7ff>(elementary_stream);
}

u32 dmux_pamf_base::lpcm_stream::parse_stream(std::span<const u8> stream)
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

	return static_cast<u32>(au_chunk.data.size());
}

template <bool ac3>
u32 dmux_pamf_base::audio_stream<ac3>::parse_stream(std::span<const u8> stream)
{
	const auto parse_au_size = [](be_t<u16> data) -> u16
	{
		if constexpr (ac3)
		{
			if (const u8 fscod = data >> 14, frmsizecod = data >> 8 & 0x3f; fscod < 3 && frmsizecod < 38)
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

	// Concatenate the cache of the previous stream section and the beginning of the current section
	std::array<u8, 8> buf{};
	std::ranges::copy(cache, buf.begin());
	std::copy_n(stream.begin(), std::min(sizeof(u16) - 1, stream.size()), buf.begin() + cache.size());

	auto au_chunk_begin = stream.begin();
	s32 cache_idx = 0;
	auto stream_it = stream.begin();
	[&]
	{
		// Search for delimiter in cache
		for (; cache_idx <= static_cast<s32>(cache.size() + std::min(sizeof(u16) - 1, stream.size()) - sizeof(u16)); cache_idx++)
		{
			if (const be_t<u16> tmp = read_from_ptr<be_t<u16>>(buf.data() + cache_idx); current_au.size_info_offset != 0)
			{
				if (--current_au.size_info_offset == 0)
				{
					current_au.parsed_size = parse_au_size(tmp);
				}
			}
			else if (tmp == SYNC_WORD)
			{
				if (current_au.state == access_unit::state::none)
				{
					// If current_au.state is none and there was a delimiter found here, then LLE outputs the entire cache, even if the access unit starts at cache_idx > 0

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

		// Search for delimiter in stream
		for (; stream_it <= stream.end() - sizeof(u32); stream_it++) // LLE uses sizeof(u32), even though we only read sizeof(u16) bytes
		{
			if (const be_t<u16> tmp = read_from_ptr<be_t<u16>>(stream_it); current_au.size_info_offset != 0)
			{
				if (--current_au.size_info_offset == 0)
				{
					current_au.parsed_size = parse_au_size(tmp);
				}
			}
			else if (tmp == SYNC_WORD)
			{
				if (current_au.state == access_unit::state::none)
				{
					au_chunk_begin = stream_it;
					current_au.size_info_offset = ac3 ? sizeof(u16) * 2 : sizeof(u16);
					current_au.state = access_unit::state::commenced;
				}
				else if (const u32 au_size = static_cast<u32>(current_au.accumulated_size + stream_it - au_chunk_begin + cache.size()); au_size >= current_au.parsed_size)
				{
					current_au.state = au_size == current_au.parsed_size ? access_unit::state::complete : access_unit::state::size_mismatch;
					return;
				}
			}
		}
	}();

	if (current_au.state != access_unit::state::none)
	{
		au_chunk.data = { au_chunk_begin, stream_it };
		std::copy_n(cache.begin(), cache_idx, std::back_inserter(au_chunk.cached_data));
		cache.erase(cache.begin(), cache.begin() + cache_idx);
	}

	// Cache the end of the stream if an access unit wasn't completed. There could be the beginning of a delimiter in the last three bytes
	if (current_au.state != access_unit::state::complete)
	{
		std::copy(stream_it, stream.end(), std::back_inserter(cache));
	}

	return static_cast<u32>((current_au.state != access_unit::state::complete ? stream.end() : stream_it) - stream.begin());
}

u32 dmux_pamf_base::user_data_stream::parse_stream_header(std::span<const u8> elementary_stream, s8 pts_dts_flag)
{
	if (pts_dts_flag < 0) // PTS field exists
	{
		// Not checked on LLE
		if (elementary_stream.size() < 2 + sizeof(u32))
		{
			return umax;
		}

		au_size_unk = read_from_ptr<be_t<u32>>(elementary_stream.begin() + 2) - sizeof(u32);
		return 10;
	}

	return 2;
}

u32 dmux_pamf_base::user_data_stream::parse_stream(std::span<const u8> stream)
{
	if (au_size_unk > stream.size())
	{
		au_chunk.data = stream;
		au_size_unk -= static_cast<u32>(stream.size());
		current_au.state = access_unit::state::commenced; // User data streams always use commenced
	}
	else
	{
		au_chunk.data = stream.first(au_size_unk);
		au_size_unk = 0;
		current_au.state = access_unit::state::complete;
	}

	return static_cast<u32>(stream.size()); // Always consume the entire stream
}

bool dmux_pamf_base::enable_es(u32 stream_id, u32 private_stream_id, bool is_avc, std::span<u8> au_queue_buffer, u32 au_max_size, bool raw_es, u32 user_data)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return false;
	}

	this->raw_es = raw_es;
	pack_es_type_idx = type_idx;

	switch (type_idx)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		elementary_streams[0][channel] = is_avc ? static_cast<std::unique_ptr<elementary_stream>>(std::make_unique<video_stream<true>>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size))
			: std::make_unique<video_stream<false>>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size);
		return true;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:      elementary_streams[1][channel] = std::make_unique<lpcm_stream>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size); return true;
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:       elementary_streams[2][channel] = std::make_unique<audio_stream<true>>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size); return true;
	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:    elementary_streams[3][channel] = std::make_unique<audio_stream<false>>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size); return true;
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA: elementary_streams[4][channel] = std::make_unique<user_data_stream>(*this, stream_id, private_stream_id, user_data, au_queue_buffer, au_max_size); return true;
	default: fmt::throw_exception("Unreachable");
	}
}

bool dmux_pamf_base::disable_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return false;
	}

	elementary_streams[type_idx][channel] = nullptr;
	return true;
}

bool dmux_pamf_base::release_au(u32 stream_id, u32 private_stream_id, u32 au_size) const
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return false;
	}

	elementary_streams[type_idx][channel]->release_au(au_size);
	return true;
}

bool dmux_pamf_base::flush_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return false;
	}

	state = state::initial;
	elementary_streams[type_idx][channel]->flush_es();
	return true;
}

void dmux_pamf_base::set_stream(std::span<const u8> stream, bool continuity)
{
	if (!continuity)
	{
		std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), &elementary_stream::discard_access_unit);
	}

	state = state::initial;

	// Not checked on LLE, it would parse old memory contents or uninitialized memory if the size of the input stream set by the user is not a multiple of 0x800.
	// Valid PAMF streams are always a multiple of 0x800 bytes large.
	if ((stream.size() & 0x7ff) != 0)
	{
		cellDmuxPamf.warning("Invalid input stream size");
	}

	this->stream.emplace(stream);
	demux_done_notified = false;
}

void dmux_pamf_base::reset_stream()
{
	std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), &elementary_stream::discard_access_unit);
	state = state::initial;
	stream.reset();
}

bool dmux_pamf_base::reset_es(u32 stream_id, u32 private_stream_id, u8* au_addr)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID || !elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
	{
		return false;
	}

	if (!au_addr)
	{
		state = state::initial;
	}

	elementary_streams[type_idx][channel]->reset_es(au_addr);
	return true;
}

bool dmux_pamf_base::process_next_pack()
{
	if (!stream)
	{
		demux_done_notified = demux_done_notified || on_demux_done();
		return true;
	}

	switch (state)
	{
	case state::initial:
	{
		// Search for the next pack start code or prog end code
		std::span<const u8, PACK_SIZE> pack{ static_cast<const u8*>(nullptr), PACK_SIZE }; // This initial value is not used, can't be default constructed
		for (;; pack = stream->get_next_pack())
		{
			if (stream->eof())
			{
				stream.reset();
				demux_done_notified = on_demux_done();
				return true;
			}

			pack = stream->get_next_pack();

			// If the input stream is a raw elementary stream, skip everything MPEG-PS related and go straight to elementary stream parsing
			if (raw_es)
			{
				if (elementary_stream::is_enabled(elementary_streams[pack_es_type_idx][0]))
				{
					elementary_streams[pack_es_type_idx][0]->set_pes_packet_data(pack);
				}

				state = state::elementary_stream;
				return true;
			}

			// While LLE is actually searching the entire section for a pack start code or program end code,
			// it doesn't set its current stream position to the address of the found code, so it would bug out if there isn't one at the start of the section
			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(pack); code == PACK_START)
			{
				break;
			}
			else if (code == PROG_END)
			{
				if (!on_prog_end())
				{
					state = state::prog_end;
				}

				return true;
			}

			cellDmuxPamf.warning("No start code found at the beginning of the current section");
		}

		// Skip over pack header
		const u8 pack_stuffing_length = read_from_ptr<u8>(pack.subspan<PACK_STUFFING_LENGTH_OFFSET>()) & 0x7;
		std::span<const u8> current_pes_packet = pack.subspan(PACK_STUFFING_LENGTH_OFFSET + sizeof(u8) + pack_stuffing_length);

		if (read_from_ptr<be_t<u32>>(current_pes_packet) >> 8 != PACKET_START_CODE_PREFIX)
		{
			cellDmuxPamf.error("Invalid start code after pack header");
			return false;
		}

		// Skip over system header if present
		if (read_from_ptr<be_t<u32>>(current_pes_packet) == SYSTEM_HEADER)
		{
			const u32 system_header_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);

			// Not checked on LLE, the SPU task would just increment the reading position and read random data in the local store
			if (system_header_length + sizeof(u32) + sizeof(u16) > current_pes_packet.size())
			{
				cellDmuxPamf.error("Invalid system header length");
				return false;
			}

			current_pes_packet = current_pes_packet.subspan(system_header_length);

			// The SPU thread isn't doing load + rotate here for 4-byte loading (for valid PAMF streams, the next start code after a system header is always aligned to 0x10 bytes)
			const u32 offset_low = (current_pes_packet.data() - pack.data()) & 0xf;
			current_pes_packet = { current_pes_packet.begin() - offset_low, current_pes_packet.end() };

			if (const be_t<u32> code = read_from_ptr<be_t<u32>>(current_pes_packet); code >> 8 != PACKET_START_CODE_PREFIX)
			{
				cellDmuxPamf.error("Invalid start code after system header");
				return false;
			}

			// A system header is optionally followed by a private stream 2
			// The first two bytes of the stream are the stream id of a video stream. The next access unit of that stream is a random access point/keyframe
			else if (code == PRIVATE_STREAM_2)
			{
				const u16 pes_packet_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);

				// Not checked on LLE, the SPU task would just increment the reading position and read random data in the local store
				if (pes_packet_length + PES_HEADER_DATA_LENGTH_OFFSET + sizeof(u8) > current_pes_packet.size())
				{
					cellDmuxPamf.error("Invalid private stream 2 length");
					return false;
				}

				if (const u8 channel = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET + sizeof(u16)) & 0xf; elementary_stream::is_enabled(elementary_streams[0][channel]))
				{
					elementary_streams[0][channel]->set_rap();
				}

				current_pes_packet = current_pes_packet.subspan(pes_packet_length);
			}
		}

		// Parse PES packet
		// Valid PAMF streams only have one PES packet per pack (not including the system header + private stream 2), so only one is parsed

		const u32 pes_packet_start_code = read_from_ptr<be_t<u32>>(current_pes_packet);

		if (pes_packet_start_code >> 8 != PACKET_START_CODE_PREFIX)
		{
			cellDmuxPamf.error("Invalid start code");
			return false;
		}

		const u16 pes_packet_length = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + PES_PACKET_LENGTH_OFFSET) + PES_PACKET_LENGTH_OFFSET + sizeof(u16);
		const u8 pes_header_data_length = read_from_ptr<u8>(current_pes_packet.begin() + PES_HEADER_DATA_LENGTH_OFFSET) + PES_HEADER_DATA_LENGTH_OFFSET + sizeof(u8);

		// Not checked on LLE, the SPU task would just increment the reading position and read random data in the local store
		if (pes_packet_length > current_pes_packet.size() || pes_packet_length <= pes_header_data_length)
		{
			cellDmuxPamf.error("Invalid pes packet length");
			return false;
		}

		const std::span<const u8> elementary_stream = current_pes_packet.subspan(pes_header_data_length, pes_packet_length - pes_header_data_length);

		const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(pes_packet_start_code, read_from_ptr<u8>(elementary_stream));

		if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
		{
			cellDmuxPamf.error("Invalid stream type");
			return false;
		}

		pack_es_type_idx = type_idx;
		pack_es_channel = channel;

		if (elementary_stream::is_enabled(elementary_streams[type_idx][channel]))
		{
			const s8 pts_dts_flag = read_from_ptr<s8>(current_pes_packet.begin() + PTS_DTS_FLAG_OFFSET);

			if (pts_dts_flag < 0)
			{
				const s32 PTS_32_30 = read_from_ptr<u8>(current_pes_packet.begin() + 9) >> 1;
				const s32 PTS_29_15 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 10) >> 1;
				const s32 PTS_14_0 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 12) >> 1;

				elementary_streams[type_idx][channel]->set_pts(PTS_32_30 << 30 | PTS_29_15 << 15 | PTS_14_0); // Bit 32 is discarded, bit 31 is sign-extended
			}

			if (pts_dts_flag & 0x40)
			{
				const s32 DTS_32_30 = read_from_ptr<u8>(current_pes_packet.begin() + 14) >> 1;
				const s32 DTS_29_15 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 15) >> 1;
				const s32 DTS_14_0 = read_from_ptr<be_t<u16>>(current_pes_packet.begin() + 17) >> 1;

				elementary_streams[type_idx][channel]->set_dts(DTS_32_30 << 30 | DTS_29_15 << 15 | DTS_14_0); // Bit 32 is discarded, bit 31 is sign-extended
			}

			const usz stream_header_size = elementary_streams[type_idx][channel]->parse_stream_header(elementary_stream, pts_dts_flag);

			// Not checked on LLE, the SPU task would just increment the reading position and read random data in the local store
			if (stream_header_size > elementary_stream.size())
			{
				cellDmuxPamf.error("Invalid stream header size");
				return false;
			}

			elementary_streams[type_idx][channel]->set_pes_packet_data(elementary_stream.subspan(stream_header_size));
		}

		state = state::elementary_stream;
		[[fallthrough]];
	}
	case state::elementary_stream:
	{
		if (!elementary_stream::is_enabled(elementary_streams[pack_es_type_idx][pack_es_channel]) || elementary_streams[pack_es_type_idx][pack_es_channel]->process_stream())
		{
			state = state::initial;
		}

		return true;
	}
	case state::prog_end:
	{
		if (on_prog_end())
		{
			state = state::initial;
		}

		return true;
	}
	default:
		fmt::throw_exception("Unreachable");
	}
}

bool dmux_pamf_spu_context::get_next_cmd(DmuxPamfCommand& lhs, bool new_stream) const
{
	cellDmuxPamf.trace("Getting next command");

	if (cmd_queue->pop(lhs))
	{
		cellDmuxPamf.trace("Command type: %d", static_cast<u32>(lhs.type.get()));
		return true;
	}

	if ((new_stream || has_work()) && !wait_for_au_queue && !wait_for_event_queue)
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

bool dmux_pamf_spu_context::send_event(auto&&... args) const
{
	if (event_queue->size() >= max_enqueued_events)
	{
		return false;
	}

	return ensure(event_queue->emplace(std::forward<decltype(args)>(args)..., event_queue_was_too_full));
}

void dmux_pamf_spu_context::operator()() // cellSpursMain()
{
	DmuxPamfCommand cmd;

	while (thread_ctrl::state() != thread_state::aborting)
	{
		if (get_next_cmd(cmd, new_stream))
		{
			event_queue_was_too_full = wait_for_event_queue;
			wait_for_event_queue = false;
			wait_for_au_queue = false;

			ensure(cmd_result_queue->emplace(static_cast<u32>(cmd.type.value()) + 1));

			switch (cmd.type)
			{
			case DmuxPamfCommandType::enable_es:
				max_enqueued_events += 2;
				enable_es(cmd.enable_es.stream_id, cmd.enable_es.private_stream_id, cmd.enable_es.is_avc, { cmd.enable_es.au_queue_buffer.get_ptr(), cmd.enable_es.au_queue_buffer_size },
					cmd.enable_es.au_max_size, cmd.enable_es.is_raw_es, cmd.enable_es.user_data);
				break;

			case DmuxPamfCommandType::disable_es:
				disable_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				max_enqueued_events -= 2;
				break;

			case DmuxPamfCommandType::set_stream:
				new_stream = true;
				break;

			case DmuxPamfCommandType::release_au:
				release_au(cmd.release_au.stream_id, cmd.release_au.private_stream_id, cmd.release_au.au_size);
				break;

			case DmuxPamfCommandType::flush_es:
				flush_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				break;

			case DmuxPamfCommandType::close:
				while (!send_event(DmuxPamfEventType::close)) {}
				return;

			case DmuxPamfCommandType::reset_stream:
				reset_stream();
				break;

			case DmuxPamfCommandType::reset_es:
				reset_es(cmd.reset_es.stream_id, cmd.reset_es.private_stream_id, cmd.reset_es.au_addr ? cmd.reset_es.au_addr.get_ptr() : nullptr);
				break;

			case DmuxPamfCommandType::resume:
				break;

			default:
				cellDmuxPamf.error("Invalid command");
				return;
			}
		}
		else if (thread_ctrl::state() == thread_state::aborting)
		{
			return;
		}

		// Only set the new stream once the previous one has been entirely consumed
		if (new_stream && !has_work())
		{
			new_stream = false;

			DmuxPamfStreamInfo stream_info;
			ensure(stream_info_queue->pop(stream_info));

			set_stream({ stream_info.stream_addr.get_ptr(), stream_info.stream_size }, stream_info.continuity);
		}

		process_next_pack();
	}
}

void dmux_pamf_base::elementary_stream::save_base(utils::serial& ar, bool save_stream, u32 stream_id, u32 private_stream_id) const
{
	ar(state, user_data, pts, dts, rap);
	au_queue.save(ar);
	ar(au_max_size, au_size_unk, au_specific_info_buf, current_au);

	if (state == state::pushing_au_queue)
	{
		ar(au_chunk.cached_data, vm::get_addr(au_chunk.data.data()), static_cast<u16>(au_chunk.data.size()));
	}

	if (current_au.state != access_unit::state::complete)
	{
		ar(cache);
	}

	if (save_stream && this->stream_id == stream_id && this->private_stream_id == private_stream_id)
	{
		ensure(pes_packet_data && pes_packet_data_begin && pes_packet_data->data() >= pes_packet_data_begin->data());
		ar(true, vm::get_addr(pes_packet_data_begin->data()), static_cast<u16>(pes_packet_data_begin->size()), static_cast<u16>(pes_packet_data->data() - pes_packet_data_begin->data()));
	}
}

void dmux_pamf_base::save_base(utils::serial& ar)
{
	bool stream_not_consumed = stream.has_value();

	ar(state, stream_not_consumed, demux_done_notified, pack_es_type_idx, raw_es);

	if (stream_not_consumed)
	{
		ar.is_writing() ? stream->save(ar) : static_cast<void>(stream.emplace(ar));
	}

	if (state == state::elementary_stream)
	{
		ar(pack_es_channel);
	}

	std::array<bool, 0x10> enabled_video_streams;
	std::array<bool, 0x10> avc_video_streams;
	std::array<bool, 0x10> enabled_lpcm_streams;
	std::array<bool, 0x10> enabled_ac3_streams;
	std::array<bool, 0x10> enabled_atracx_streams;
	std::array<bool, 0x10> enabled_user_data_streams;

	if (ar.is_writing())
	{
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO], enabled_video_streams.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO], avc_video_streams.begin(), [](auto& es){ return !!dynamic_cast<video_stream<true>*>(es.get()); });
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_LPCM], enabled_lpcm_streams.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_AC3], enabled_ac3_streams.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX], enabled_atracx_streams.begin(), elementary_stream::is_enabled);
		std::ranges::transform(elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA], enabled_user_data_streams.begin(), elementary_stream::is_enabled);
	}

	ar(enabled_video_streams, avc_video_streams, enabled_lpcm_streams, enabled_ac3_streams, enabled_atracx_streams, enabled_user_data_streams);

	if (ar.is_writing())
	{
		std::ranges::for_each(elementary_streams | std::views::join | std::views::filter(elementary_stream::is_enabled), [&](const auto& es){ es->save(ar, state == state::elementary_stream, pack_es_channel); });
	}
	else
	{
		for (u32 ch = 0; ch < 0x10; ch++)
		{
			if (enabled_video_streams[ch])
			{
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO][ch] = avc_video_streams[ch]
					? static_cast<std::unique_ptr<elementary_stream>>(std::make_unique<video_stream<true>>(*this, ar, 0xe0 | ch))
					: std::make_unique<video_stream<false>>(*this, ar, 0xe0 | ch);
			}
		}

		for (u32 ch = 0; ch < 0x10; ch++)
		{
			if (enabled_lpcm_streams[ch])
			{
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_LPCM][ch] = std::make_unique<lpcm_stream>(*this, ar, 0x40 | ch);
			}
		}

		for (u32 ch = 0; ch < 0x10; ch++)
		{
			if (enabled_ac3_streams[ch])
			{
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_AC3][ch] = std::make_unique<audio_stream<true>>(*this, ar, 0x30 | ch);
			}
		}

		for (u32 ch = 0; ch < 0x10; ch++)
		{
			if (enabled_atracx_streams[ch])
			{
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX][ch] = std::make_unique<audio_stream<false>>(*this, ar, 0x00 | ch);
			}
		}

		for (u32 ch = 0; ch < 0x10; ch++)
		{
			if (enabled_user_data_streams[ch])
			{
				elementary_streams[DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA][ch] = std::make_unique<user_data_stream>(*this, ar, 0x20 | ch);
			}
		}
	}
}

void dmux_pamf_spu_context::save(utils::serial& ar)
{
	USING_SERIALIZATION_VERSION(cellDmuxPamf);
	ar(cmd_queue, new_stream); // The queues are contiguous in guest memory, so we only need to save the address of the first one
	save_base(ar);
}


//----------------------------------------------------------------------------
// PPU thread
//----------------------------------------------------------------------------

error_code _CellDmuxCoreOpQueryAttr(vm::cptr<CellDmuxPamfSpecificInfo> pamfSpecificInfo, vm::ptr<CellDmuxPamfAttr> pamfAttr)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpQueryAttr(pamfSpecificInfo=*0x%x, pamfAttr=*0x%x)", pamfSpecificInfo, pamfAttr);

	return CELL_OK;
}

error_code _CellDmuxCoreOpOpen(vm::cptr<CellDmuxPamfSpecificInfo> pamfSpecificInfo, vm::cptr<CellDmuxResource> demuxerResource, vm::cptr<CellDmuxResourceSpurs> demuxerResourceSpurs, vm::cptr<DmuxCb<DmuxNotifyDemuxDone>> notifyDemuxDone,
	vm::cptr<DmuxCb<DmuxNotifyProgEndCode>> notifyProgEndCode, vm::cptr<DmuxCb<DmuxNotifyFatalErr>> notifyFatalErr, vm::pptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpOpen(pamfSpecificInfo=*0x%x, demuxerResource=*0x%x, demuxerResourceSpurs=*0x%x, notifyDemuxDone=*0x%x, notifyProgEndCode=*0x%x, notifyFatalErr=*0x%x, handle=**0x%x)",
		pamfSpecificInfo, demuxerResource, demuxerResourceSpurs, notifyDemuxDone, notifyProgEndCode, notifyFatalErr, handle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpClose(vm::ptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpClose(handle=*0x%x)", handle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpResetStream(vm::ptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpResetStream(handle=*0x%x)", handle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpCreateThread(vm::ptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpCreateThread(handle=*0x%x)", handle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpJoinThread(vm::ptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpJoinThread(handle=*0x%x)", handle);

	return CELL_OK;
}

template <bool raw_es>
error_code _CellDmuxCoreOpSetStream(vm::ptr<void> handle, vm::cptr<void> streamAddress, u32 streamSize, b8 discontinuity, u64 userData)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpSetStream<raw_es=%d>(handle=*0x%x, streamAddress=*0x%x, streamSize=0x%x, discontinuity=%d, userData=0x%llx)", raw_es, handle, streamAddress, streamSize, +discontinuity, userData);

	return CELL_OK;
}

error_code _CellDmuxCoreOpReleaseAu(vm::ptr<void> esHandle, vm::ptr<void> memAddr, u32 memSize)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpReleaseAu(esHandle=*0x%x, memAddr=*0x%x, memSize=0x%x)", esHandle, memAddr, memSize);

	return CELL_OK;
}

template <bool raw_es>
error_code _CellDmuxCoreOpQueryEsAttr(vm::cptr<void> esFilterId, vm::cptr<void> esSpecificInfo, vm::ptr<CellDmuxPamfEsAttr> attr)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpQueryEsAttr<raw_es=%d>(esFilterId=*0x%x, esSpecificInfo=*0x%x, attr=*0x%x)", raw_es, esFilterId, esSpecificInfo, attr);

	return CELL_OK;
}

template <bool raw_es>
error_code _CellDmuxCoreOpEnableEs(vm::ptr<void> handle, vm::cptr<void> esFilterId, vm::cptr<CellDmuxEsResource> esResource, vm::cptr<DmuxCb<DmuxEsNotifyAuFound>> notifyAuFound,
	vm::cptr<DmuxCb<DmuxEsNotifyFlushDone>> notifyFlushDone, vm::cptr<void> esSpecificInfo, vm::pptr<void> esHandle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpEnableEs<raw_es=%d>(handle=*0x%x, esFilterId=*0x%x, esResource=*0x%x, notifyAuFound=*0x%x, notifyFlushDone=*0x%x, esSpecificInfo=*0x%x, esHandle)",
		raw_es, handle, esFilterId, esResource, notifyAuFound, notifyFlushDone, esSpecificInfo, esHandle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpDisableEs(vm::ptr<void> esHandle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpDisableEs(esHandle=*0x%x)", esHandle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpFlushEs(vm::ptr<void> esHandle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpFlushEs(esHandle=*0x%x)", esHandle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpResetEs(vm::ptr<void> esHandle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpResetEs(esHandle=*0x%x)", esHandle);

	return CELL_OK;
}

error_code _CellDmuxCoreOpResetStreamAndWaitDone(vm::ptr<void> handle)
{
	cellDmuxPamf.todo("_CellDmuxCoreOpResetStreamAndWaitDone(handle=*0x%x)", handle);

	return CELL_OK;
}

template <bool raw_es>
static void init_gvar(const vm::gvar<CellDmuxCoreOps>& var)
{
	var->queryAttr.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpQueryAttr)));
	var->open.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpOpen)));
	var->close.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpClose)));
	var->resetStream.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetStream)));
	var->createThread.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpCreateThread)));
	var->joinThread.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpJoinThread)));
	var->setStream.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpSetStream<raw_es>)));
	var->releaseAu.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpReleaseAu)));
	var->queryEsAttr.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpQueryEsAttr<raw_es>)));
	var->enableEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpEnableEs<raw_es>)));
	var->disableEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpDisableEs)));
	var->flushEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpFlushEs)));
	var->resetEs.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetEs)));
	var->resetStreamAndWaitDone.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpResetStreamAndWaitDone)));
}

DECLARE(ppu_module_manager::cellDmuxPamf)("cellDmuxPamf", []
{
	REG_VNID(cellDmuxPamf, 0x28b2b7b2, g_cell_dmux_core_ops_pamf).init = []{ init_gvar<false>(g_cell_dmux_core_ops_pamf); };
	REG_VNID(cellDmuxPamf, 0x9728a0e9, g_cell_dmux_core_ops_raw_es).init = []{ init_gvar<true>(g_cell_dmux_core_ops_raw_es); };

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
});
