#include "stdafx.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/Cell/lv2/sys_mutex.h"
#include "Emu/Cell/lv2/sys_cond.h"
#include "Emu/Cell/lv2/sys_sync.h"
#include "Emu/Cell/lv2/sys_ppu_thread.h"
#include "Emu/savestate_utils.hpp"
#include "Emu/System.h"
#include "sysPrxForUser.h"
#include "util/simd.hpp"

#include "cellDmux.h"
#include "cellDmuxPamf.h"

#include <chrono> // TODO remove

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

void dmux_pamf_context::elementary_stream::access_unit_queue::write(const es_0x10& unk2)
{
	std::memcpy(addr.get_ptr() + pos, &unk2.prev_bytes, unk2.prev_bytes_size);
	std::memcpy(addr.get_ptr() + pos + unk2.prev_bytes_size, unk2.addr, unk2.au_size);

	pos += unk2.prev_bytes_size + unk2.au_size;
}

template <bool is_avc>
u32 dmux_pamf_context::video_stream<is_avc>::parse_stream(const u8* input_addr, u32 input_size, es_0x10& unk4)
{
	const u8* cutout_start_addr = nullptr;

	if (unk0xc0.first_au_delimiter_found)
	{
		cutout_start_addr = input_addr;
		unk0xc0.au_cut_status = 5;
	}

	if (unk0xc0.cache_start_idx != 0)
	{
		const v128 buf[2] = { unk0xc0.prev_packet_cache, read_from_ptr<v128>(input_addr) };

		for (u32 i = unk0xc0.cache_start_idx; i < sizeof(v128); i++)
		{
			if (be_t<u32> code = read_from_ptr<be_t<u32>>(&buf->_bytes[i]); (is_avc && code == AVC_AU_DELIMITER) || (!is_avc && (code == M2V_PIC_START || code == M2V_SEQUENCE_HEADER || code == M2V_SEQUENCE_END)))
			{
				if (unk0xc0.first_au_delimiter_found)
				{
					unk4.addr = cutout_start_addr;
					unk4.processed_size = 0;
					unk4.remaining_bytes_to_parse = 0;
					unk4.au_size = 0;
					unk4.prev_bytes_size = i - unk0xc0.cache_start_idx;
					std::memcpy(&unk4.prev_bytes, &unk0xc0.prev_packet_cache._bytes[unk0xc0.cache_start_idx], sizeof(v128) - unk0xc0.cache_start_idx);
					unk0xc0.cache_start_idx = i;

					if (!is_avc && code == M2V_SEQUENCE_END)
					{
						unk4.au_size += sizeof(u32);
						unk4.processed_size += sizeof(u32);
					}

					unk0xc0.current_au_size += unk4.prev_bytes_size + unk4.au_size;
					unk0xc0.au_cut_status = 3;

					return 3;
				}

				cutout_start_addr = input_addr;
				unk0xc0.au_cut_status = 1;
				unk0xc0.first_au_delimiter_found = is_avc || code == M2V_PIC_START;
			}
		}
	}

	s64 i = 0;
	for (; i <= static_cast<s64>(input_size - sizeof(u32)); i++)
	{
		if (be_t<u32> code = read_from_ptr<be_t<u32>>(&input_addr[i]); (is_avc && code == AVC_AU_DELIMITER) || (!is_avc && (code == M2V_PIC_START || code == M2V_SEQUENCE_HEADER || code == M2V_SEQUENCE_END)))
		{
			if (unk0xc0.first_au_delimiter_found)
			{
				if (!is_avc && code == M2V_SEQUENCE_END)
				{
					i += sizeof(u32);
				}

				unk0xc0.au_cut_status = 3;
				break;
			}

			if (is_avc || !cutout_start_addr)
			{
				cutout_start_addr = input_addr + i;
				unk0xc0.au_cut_status = 1;
			}

			unk0xc0.first_au_delimiter_found = is_avc || code == M2V_PIC_START;
		}
	}

	unk4.addr = cutout_start_addr;
	unk4.remaining_bytes_to_parse = unk0xc0.au_cut_status == 3 ? 0 : sizeof(u32) - 1;

	if (!cutout_start_addr)
	{
		unk4.au_size = 0;
	}
	else
	{
		unk4.au_size = static_cast<u32>(input_addr + i - cutout_start_addr);
		unk4.processed_size = static_cast<u32>(input_addr + i - cutout_start_addr) + unk4.remaining_bytes_to_parse;
		unk4.prev_bytes_size = unk0xc0.cache_start_idx ? sizeof(v128) - unk0xc0.cache_start_idx : 0;
		std::memcpy(&unk4.prev_bytes, &unk0xc0.prev_packet_cache._bytes[unk0xc0.cache_start_idx], sizeof(v128) - unk0xc0.cache_start_idx);

		unk0xc0.current_au_size += unk4.prev_bytes_size + unk4.au_size;
	}

	unk0xc0.cache_start_idx = unk4.remaining_bytes_to_parse == 0 ? 0 : sizeof(v128) - unk4.remaining_bytes_to_parse;
	std::memcpy(&unk0xc0.prev_packet_cache, input_addr + input_size - sizeof(v128), sizeof(v128));

	return unk0xc0.au_cut_status == 3 ? 3 : 4;
}

// TODO: names
#pragma message(": warning: TODO")
u32 dmux_pamf_context::lpcm_stream::parse_stream(const u8* input_addr, u32 input_size, es_0x10& unk4)
{
	unk4.addr = input_addr;

	if (unk0xc0.au_cut_status == 0)
	{
		unk0xc0.stream_header_buf = stream_header_buf;
	}
	else
	{
		unk4.prev_bytes_size = 0;
	}

	const u32 remaining_au_bytes = access_unit_queue.au_max_size - unk0xc0.current_au_size;

	if (remaining_au_bytes > input_size)
	{
		unk4.au_size = input_size;
		unk4.processed_size = input_size;

		unk0xc0.current_au_size += input_size;

		if (unk0xc0.au_cut_status == 0)
		{
			unk0xc0.au_cut_status = 1; // TODO make enum

			return 4;
		}

		unk0xc0.au_cut_status = 5; // TODO make enum

		return 4;
	}

	unk4.au_size = remaining_au_bytes;
	unk4.processed_size = remaining_au_bytes;

	unk0xc0.current_au_size += remaining_au_bytes;
	unk0xc0.au_cut_status = 3; // TODO make enum

	return 5;
}

// TODO: SIMPLIFY, VARIABLE NAMES
#pragma message(": warning: TODO")
template <bool is_ac3>
u32 dmux_pamf_context::audio_stream<is_ac3>::parse_stream(const u8* input_addr, u32 input_size, es_0x10& unk4)
{
	const u8* cutout_start_addr = nullptr;

	if (unk0xc0.first_au_delimiter_found)
	{
		cutout_start_addr = input_addr;
		unk0xc0.au_cut_status = 5;
	}

	const u32 old_cache_start_idx = unk0xc0.cache_start_idx;
	v128 buf[2]{ unk0xc0.prev_packet_cache, read_from_ptr<v128>(input_addr) };

	u32 buf_size = sizeof(v128);

	for (s32 input_pos = old_cache_start_idx != 0 ? -static_cast<s32>(sizeof(v128)) : 0; input_pos <= static_cast<s64>(input_size - sizeof(u32)); input_pos += sizeof(v128), unk0xc0.cache_start_idx = 0)
	{
		if (input_pos >= 0)
		{
			buf[0] = buf[1];

			if (input_size - input_pos > sizeof(v128))
			{
				buf[1] = read_from_ptr<v128>(input_addr + input_pos + sizeof(v128));
			}
			else
			{
				buf_size = input_size - input_pos - 3;
			}
		}

		for (u32 i = unk0xc0.cache_start_idx; i < buf_size; i++)
		{
			if (be_t<u16> tmp = read_from_ptr<be_t<u16>>(&buf->_bytes[i]); unk0xc0.au_info_offset != 0)
			{
				unk0xc0.au_info_offset--;

				if (unk0xc0.au_info_offset == 0)
				{
					unk0xc0.parsed_au_size = 0;

					if constexpr (is_ac3)
					{
						if (u8 fscod = tmp >> 14, frmsizecod = (tmp >> 8) & 0x3f; fscod < 3 && frmsizecod < 38)
						{
							unk0xc0.parsed_au_size = AC3_FRMSIZE_TABLE[fscod][frmsizecod] * sizeof(s16);
						}
					}
					else
					{
						if ((tmp & 0x3ff) < 0x200)
						{
							unk0xc0.parsed_au_size = ((tmp & 0x3ff) + 1) * 8 + 8 /* TODO sizeof ats header*/;
						}
					}
				}
			}
			else if ((is_ac3 && tmp == AC3_SYNCWORD) || (!is_ac3 && tmp == ATRACX_SYNCWORD))
			{
				if (!unk0xc0.first_au_delimiter_found)
				{
					cutout_start_addr = input_pos < 0 ? input_addr : input_addr + input_pos + i;

					unk0xc0.first_au_delimiter_found = true;
					unk0xc0.au_info_offset = is_ac3 ? sizeof(u16) * 2 : sizeof(u16);
					unk0xc0.au_cut_status = 1;
					continue;
				}

				if (input_pos >= 0)
				{
					unk4.addr = cutout_start_addr;
					unk4.processed_size = static_cast<u32>(input_addr + input_pos + i - cutout_start_addr);
					unk4.au_size = static_cast<u32>(input_addr + input_pos + i - cutout_start_addr);
					unk4.remaining_bytes_to_parse = 0;
					unk4.prev_bytes_size = old_cache_start_idx ? sizeof(v128) - old_cache_start_idx : 0;
					std::memcpy(&unk4.prev_bytes, &unk0xc0.prev_packet_cache._bytes[old_cache_start_idx], sizeof(v128) - old_cache_start_idx);
				}
				else
				{
					unk4.addr = cutout_start_addr;
					unk4.processed_size = 0;
					unk4.remaining_bytes_to_parse = 0;
					unk4.au_size = 0;
					unk4.prev_bytes_size = i - unk0xc0.cache_start_idx;
					std::memcpy(&unk4.prev_bytes, &unk0xc0.prev_packet_cache._bytes[old_cache_start_idx], sizeof(v128) - old_cache_start_idx);
					unk0xc0.cache_start_idx = i;
				}

				if (u32 current_au_size = unk0xc0.current_au_size + unk4.prev_bytes_size + unk4.au_size; current_au_size >= unk0xc0.parsed_au_size)
				{
					unk0xc0.au_cut_status = current_au_size == unk0xc0.parsed_au_size ? 3 : 2;
					unk0xc0.current_au_size += unk4.prev_bytes_size + unk4.au_size;
					return unk0xc0.au_cut_status;
				}
			}
		}
	}

	unk4.addr = cutout_start_addr;

	if (!cutout_start_addr)
	{
		unk4.au_size = 0;
	}
	else
	{
		unk4.processed_size = static_cast<u32>(input_addr + input_size - cutout_start_addr);
		unk4.au_size = static_cast<u32>(input_addr + input_size - cutout_start_addr - 3);
		unk4.remaining_bytes_to_parse = 3;
		unk4.prev_bytes_size = old_cache_start_idx ? sizeof(v128) - old_cache_start_idx : 0;
		std::memcpy(&unk4.prev_bytes, &unk0xc0.prev_packet_cache._bytes[old_cache_start_idx], sizeof(v128) - old_cache_start_idx);

		unk0xc0.current_au_size += unk4.prev_bytes_size + unk4.au_size;
	}

	unk0xc0.cache_start_idx = 0xd;
	unk0xc0.prev_packet_cache = read_from_ptr<v128>(input_addr + input_size - sizeof(v128));

	return 4;
}

u32 dmux_pamf_context::user_data_stream::parse_stream(const u8* addr, u32 size, es_0x10& unk4)
{
	unk4.addr = addr;
	unk4.prev_bytes_size = 0;
	unk4.processed_size = size;

	if (au_size > size)
	{
		unk4.au_size = size;
		unk0xc0.current_au_size += size;
		au_size -= size;

		if (unk0xc0.au_cut_status == 0)
		{
			unk0xc0.au_cut_status = 1; // TODO make enum
		}

		return 4;
	}

	unk4.au_size = au_size;
	unk0xc0.current_au_size += au_size;
	au_size = 0;

	unk0xc0.au_cut_status = 3; // TODO make enum

	return 5;
}

bool dmux_pamf_context::enable_es(u32 stream_id, u32 private_stream_id, bool is_avc, u32 au_queue_buffer_size, vm::ptr<u8> au_queue_buffer, u32 au_max_size, bool is_raw_es, u32 es_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (elementary_streams[type_idx][channel]) // Already enabled
	{
		return false;
	}

	raw_es = is_raw_es;

	if (is_raw_es)
	{
		es_type_idx = type_idx;
	}

	switch (type_idx)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		if (is_avc)
		{
			elementary_streams[0][channel] = std::make_unique<video_stream<true>>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		}
		else
		{
			elementary_streams[0][channel] = std::make_unique<video_stream<false>>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		}
		return true;

	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
		elementary_streams[1][channel] = std::make_unique<lpcm_stream>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		return true;

	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
		elementary_streams[2][channel] = std::make_unique<audio_stream<true>>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		return true;

	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
		elementary_streams[3][channel] = std::make_unique<audio_stream<false>>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		return true;

	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
		elementary_streams[4][channel] = std::make_unique<user_data_stream>(stream_id, private_stream_id, es_id, au_queue_buffer, au_queue_buffer_size, au_max_size);
		return true;

	default:
		fmt::throw_exception("Unreachable");
	}
}

bool dmux_pamf_context::disable_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (!elementary_streams[type_idx][channel]) // Already disabled
	{
		return false;
	}

	elementary_streams[type_idx][channel].reset();
	return true;
}

bool dmux_pamf_context::free_memory(u32 mem_size, u32 stream_id, u32 private_stream_id) const
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (!elementary_streams[type_idx][channel]) // Not enabled
	{
		return false;
	}

	elementary_stream& es = *elementary_streams[type_idx][channel];

	if (es.access_unit_queue.end_pos != 0)
	{
		if (es.access_unit_queue.end_pos <= es.access_unit_queue.freed_mem_size + mem_size)
		{
			es.access_unit_queue.freed_mem_size = es.access_unit_queue.freed_mem_size + mem_size - es.access_unit_queue.end_pos;
			es.access_unit_queue.end_pos = 0;

			return true;
		}
	}

	es.access_unit_queue.freed_mem_size += mem_size;

	return true;
}

bool dmux_pamf_context::flush_es(u32 stream_id, u32 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (!elementary_streams[type_idx][channel]) // Not enabled
	{
		return false;
	}

	elementary_stream& es = *elementary_streams[type_idx][channel];

	if (es.unk0xc0.current_au_size != 0)
	{
		es.unk0x10.processed_size = 0;
		es.unk0x10.au_size = 0;
		es.unk0x10.addr = nullptr;

		const u32 prev_bytes_size = sizeof(v128) - es.unk0xc0.cache_start_idx;
		es.unk0x10.prev_bytes_size = prev_bytes_size;

		es.unk0x10.prev_bytes.clear();
		std::memcpy(&es.unk0x10.prev_bytes, &es.unk0xc0.prev_packet_cache._bytes[es.unk0xc0.cache_start_idx], prev_bytes_size);

		es.access_unit_queue.write(es.unk0x10);

		const u32 au_size = es.unk0xc0.current_au_size - es.unk0xc0.cache_start_idx + 0x10;
		const u32 au_start_offset = es.access_unit_queue.pos - au_size;

		es.unk0xc0.current_au_size = au_size;

		send_au_found(es.stream_id, es.private_stream_id, es.access_unit_queue.addr + au_start_offset, es.unk0xc0.pts, es.unk0xc0.dts, au_size, es.au_specific_info_size, es.au_params.stream_header_buf, es.es_id, es.unk0xc0.is_rap);
	}

	es.au_size = 0;
	es.start_of_au = false;

	es.reset();
	es.reset_timestamps();
	es.unk0x10.reset();
	es.unk0xc0.reset();

	unk0x30.state = demuxer::state::initial;

	while (!send_event(DmuxPamfEventType::flush_done, stream_id, private_stream_id, es.es_id)){}

	return true;
}

void dmux_pamf_context::reset_stream()
{
	for (std::unique_ptr<elementary_stream> (&types)[0x10] : elementary_streams)
	{
		for (std::unique_ptr<elementary_stream>& es : types)
		{
			if (!es) // Not enabled
			{
				continue;
			}

			// TODO
#pragma message(": warning: TODO")
			const u32 sizeUNK_1_add_0x1c = es->unk0x10.au_size + es->unk0x10.prev_bytes_size;
			const u32 unk_0xc8_sub = es->unk0xc0.current_au_size - sizeUNK_1_add_0x1c;
			const u32 unk_0xac_sub = es->access_unit_queue.pos - unk_0xc8_sub;

			es->access_unit_queue.pos = unk_0xac_sub;

			es->au_size = 0;
			es->start_of_au = false;

			es->reset();
			es->reset_timestamps();
			es->unk0x10.reset();
			es->unk0xc0.reset();

			es->unk0xc0.cache_start_idx = 0;
		}
	}

	unk0x30.reset();
	input_stream.init(vm::null, 0);
}

bool dmux_pamf_context::reset_es(u32 stream_id, u32 private_stream_id, vm::ptr<u8> au_addr)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (!elementary_streams[type_idx][channel]) // Not enabled
	{
		return false;
	}

	elementary_stream& es = *elementary_streams[type_idx][channel];

	if (!au_addr)
	{
		es.au_size = 0;
		es.start_of_au = false;

		es.reset();
		es.reset_timestamps();
		es.unk0x10.reset();
		es.unk0xc0.reset();

		es.access_unit_queue.reset();

		unk0x30.state = demuxer::state::initial;
	}
	else
	{
		const u32 au_offset = au_addr - es.access_unit_queue.addr;

		if (es.access_unit_queue.end_pos != 0)
		{
			if (au_offset > es.access_unit_queue.pos)
			{
				es.access_unit_queue.end_pos = 0;
			}
		}

		es.access_unit_queue.pos = au_offset;
	}

	return true;
}

void dmux_pamf_context::discard_access_units()
{
	for (std::unique_ptr<elementary_stream> (&types)[0x10] : elementary_streams)
	{
		for (std::unique_ptr<elementary_stream>& es : types)
		{
			if (!es) // Not enabled
			{
				continue;
			}

			if (es->unk0xc0.current_au_size != 0)
			{
				es->access_unit_queue.pos -= es->unk0xc0.current_au_size;
			}

			es->au_size = 0;
			es->start_of_au = false;

			es->reset();
			es->reset_timestamps();
			es->unk0x10.reset();
			es->unk0xc0.reset();

			es->unk0xc0.cache_start_idx = 0;
		}
	}
}

inline bool dmux_pamf_context::set_demux_done()
{
	demux_done = true;
	au_queue_full = false;
	return check_demux_done_was_notified();
}

template <bool set_au_queue_full>
inline bool dmux_pamf_context::check_demux_done()
{
	au_queue_full = set_au_queue_full;
	return demux_done ? check_demux_done_was_notified() : true;
}

inline bool dmux_pamf_context::check_demux_done_was_notified()
{
	if (demux_done_was_notified)
	{
		return true;
	}

	if (!send_event(DmuxPamfEventType::demux_done))
	{
		event_queue_too_full = true;
		return false;
	}

	demux_done_was_notified = true;
	return true;
}

bool dmux_pamf_context::demux(const DmuxPamfStreamInfo* stream_info)
{
	if (demux_done)
	{
		if (stream_info)
		{
			if (!demux_done_was_notified)
			{
				au_queue_full = false;

				if (!send_event(DmuxPamfEventType::demux_done))
				{
					event_queue_too_full = true;
					return false;
				}

				demux_done_was_notified = true;
				return true;
			}

			if (!stream_info->continuity)
			{
				discard_access_units();
			}

			unk0x30.reset();

			input_stream.init(stream_info->stream_addr, stream_info->stream_size);

			demux_done = false;
			demux_done_was_notified = false;
		}
		else
		{
			au_queue_full = false;
			return check_demux_done_was_notified();
		}
	}

	s32 end_of_pack_stuffing_bytes_offset = 0;

	switch (unk0x30.state)
	{
	case demuxer::state::initial:
	{
		unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;
		[[fallthrough]];
	}
	case demuxer::state::refreshing_buffer_and_finding_start_code:
	{
		for (;;)
		{
			if (static_cast<s64>(input_stream.pos) > static_cast<s64>(input_stream.size) - static_cast<s64>(sizeof(v128)))
			{
				unk0x30.current_addr = nullptr;

				return set_demux_done();
			}

			// PAMF streams are always a multiple of 0x800 bytes large.
			// Should this actually occur, LLE would end up parsing old memory contents since it doesn't consistently check the stream size before reading from it.
			// To emulate this properly, we would need to copy the input stream into an internal buffer like LLE
			ensure(!(input_stream.size & 0x7ff));

			input_stream.bytes_to_process = PACK_SIZE;

			unk0x30.current_addr = input_stream.addr.get_ptr() + input_stream.pos;
			input_stream.pos += 0x800;

			if (raw_es)
			{
				unk0x30.channel = 0;
				unk0x30.state = demuxer::state::parsing_elementary_stream;
				unk0x30.type_idx = es_type_idx;
				unk0x30.pes_packet_remaining_size = input_stream.bytes_to_process;
				unk0x30.end_of_pes_packet = unk0x30.current_addr + input_stream.bytes_to_process;

				return check_demux_done<false>();
			}

			// LLE is actually searching the entire input stream for a pack start code/program end code.
			// However, if the code is not at the start of the current segment, then parsing the rest of the pack
			// would almost always fail, because the input address is not adjusted to where the code was found and old memory contents are parsed

			if (be_t<u32> code = read_from_ptr<be_t<u32>>(unk0x30.current_addr); code == PACK_START)
			{
				unk0x30.state = demuxer::state::parsing_pack_header;
				break;
			}
			else if (code == PROG_END)
			{
				if (!send_event(DmuxPamfEventType::prog_end_code))
				{
					unk0x30.state = demuxer::state::found_prog_end_code;

					event_queue_too_full = true;
					return check_demux_done<false>();
				}

				unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;

				return check_demux_done<false>();
			}

			cellDmuxPamf.warning("No start code found at the beginning of current segment");
		}

		[[fallthrough]];
	}
	case demuxer::state::parsing_pack_header:
	{
		if (PACK_STUFFING_LENGTH_OFFSET > input_stream.bytes_to_process)
		{
			unk0x30.state = demuxer::state::parsing_pack_header;

			input_stream.bytes_to_process = 0;

			return set_demux_done();
		}

		const s32 pack_stuffing_length = read_from_ptr<u8>(unk0x30.current_addr + PACK_STUFFING_LENGTH_OFFSET) & 0x7;

		end_of_pack_stuffing_bytes_offset = pack_stuffing_length + PACK_STUFFING_LENGTH_OFFSET + sizeof(u8);

		unk0x30.state = demuxer::state::verifying_start_of_pes_packet;
		[[fallthrough]];
	}
	case demuxer::state::verifying_start_of_pes_packet:
	{
		if (end_of_pack_stuffing_bytes_offset > input_stream.bytes_to_process)
		{
			unk0x30.state = demuxer::state::verifying_start_of_pes_packet;

			input_stream.bytes_to_process = 0;

			return set_demux_done();
		}

		unk0x30.current_addr += end_of_pack_stuffing_bytes_offset;

		// Not accurate, but we need to keep track of the size of the buffer, since we're reading directly from the input stream unlike LLE
		input_stream.bytes_to_process -= end_of_pack_stuffing_bytes_offset;

		ensure(input_stream.bytes_to_process > static_cast<s32>(sizeof(u32))); // Not checked on LLE

		if (read_from_ptr<be_t<u32>>(unk0x30.current_addr) >> 8 != PACKET_START_CODE_PREFIX)
		{
			unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;

			return check_demux_done<false>();
		}

		unk0x30.state = demuxer::state::parsing_system_header;
		[[fallthrough]];
	}
	case demuxer::state::parsing_system_header:
	{
		if (read_from_ptr<be_t<u32>>(unk0x30.current_addr) == SYSTEM_HEADER)
		{
			ensure(input_stream.bytes_to_process > 4 + static_cast<s32>(sizeof(u16))); // Not checked on LLE

			const u16 header_length = read_from_ptr<be_t<u16>>(&unk0x30.current_addr[4]);
			unk0x30.current_addr += header_length + 6;

			// Not accurate, but we need to keep track of the size of the buffer, since we're reading directly from the input stream unlike LLE
			input_stream.bytes_to_process -= header_length + 6;

			// LLE isn't adjusting its loading for unaligned addresses here // TODO asidufghapidfgpaiudfhaeufhpaosdfiunhfaivundfhfpundfhpgusfpahefpgvhaeprighvaperuhgbsperihvnaperasidufghapidfgpaiudfhaeufhpaosdfiunhfaivundfhfpundfhpgusfpahefpgvhaeprighvaperuhgbsperihvnaper
			const usz input_addr_low = reinterpret_cast<usz>(unk0x30.current_addr) & 0xf;
			const u8* const input_addr_aligned = unk0x30.current_addr - input_addr_low;

			ensure(input_stream.bytes_to_process > static_cast<s32>(sizeof(u32) - input_addr_low)); // Not checked on LLE

			if (be_t<u32> code = read_from_ptr<be_t<u32>>(input_addr_aligned); code >> 8 != PACKET_START_CODE_PREFIX)
			{
				unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;

				return check_demux_done<false>();
			}
			else if (code == PRIVATE_STREAM_2)
			{
				// TODO asidufghapidfgpaiudfhaeufhpaosdfiunhfaivundfhfpundfhpgusfpahefpgvhaeprighvaperuhgbsperihvnaperasidufghapidfgpaiudfhaeufhpaosdfiunhfaivundfhfpundfhpgusfpahefpgvhaeprighvaperuhgbsperihvnaper

				const u16 PES_packet_length = read_from_ptr<be_t<u16>>(&input_addr_aligned[4]);
				unk0x30.current_addr += PES_packet_length + 6;

				/*if (u32 channel = input_addr_aligned[7] & 0xf; elementary_streams[0][channel].is_enabled)
				{
					elementary_streams[0][channel].is_rap = true;
				}*/

				if (u32 channel = input_addr_aligned[7] & 0xf; elementary_streams[0][channel])
				{
					elementary_streams[0][channel]->is_rap = true;
				}
			}
		}

		unk0x30.pes_packet_header = unk0x30.current_addr;

		if (read_from_ptr<be_t<u32>>(unk0x30.pes_packet_header) >> 8 != PACKET_START_CODE_PREFIX)
		{
			unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;

			return check_demux_done<false>();
		}

		unk0x30.state = demuxer::state::parsing_pes_packet_header;
		[[fallthrough]];
	}
	case demuxer::state::parsing_pes_packet_header:
	{
		const u16 PES_packet_length = read_from_ptr<be_t<u16>>(&unk0x30.pes_packet_header[4]);
		const u8 PES_header_data_length = unk0x30.pes_packet_header[8];

		const u32 total_pes_packet_length = PES_packet_length + 6;
		const u32 total_pes_header_length = PES_header_data_length + 9;

		unk0x30.pes_packet_remaining_size = total_pes_packet_length - total_pes_header_length;
		unk0x30.current_addr += total_pes_header_length;

		unk0x30.state = demuxer::state::parsing_elementary_stream_header;
		[[fallthrough]];
	}
	case demuxer::state::parsing_elementary_stream_header:
	{
		const u32 PES_packet_start_code = read_from_ptr<be_t<u32>>(unk0x30.pes_packet_header);

		const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(PES_packet_start_code, unk0x30.current_addr[0]);

		s32 unk_size = 0;

		switch (type_idx)
		{
		case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
			// Don't do anything
			break;

		case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:
			if (elementary_streams[1][channel])
			{
				elementary_streams[1][channel]->stream_header_buf = v128::loadu(&unk0x30.current_addr[1]);

				if (elementary_streams[1][channel]->au_size == 0)
				{
					unk_size = std::bit_cast<be_t<u32>>(elementary_streams[1][channel]->stream_header_buf._u32[0]) >> 8 & 0x7ff;

					ensure(unk_size != 0x7ff); // This case is bugged on LLE, likely never happens with valid streams

					elementary_streams[1][channel]->au_size = 1;
				}
			}

			unk0x30.current_addr += unk_size + 4;
			unk0x30.pes_packet_remaining_size -= unk_size + 4;
			break;

		case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:
			if (elementary_streams[2][channel])
			{
				elementary_streams[2][channel]->stream_header_buf = v128::loadu(&unk0x30.current_addr[1]);

				if (elementary_streams[2][channel]->au_size == 0)
				{
					unk_size = std::bit_cast<be_t<u32>>(elementary_streams[2][channel]->stream_header_buf._u32[0]) >> 8 & 0xffff;

					ensure(unk_size != 0xffff); // This case is bugged on LLE, likely never happens with valid streams

					elementary_streams[2][channel]->au_size = 1;
				}
			}

			unk0x30.current_addr += unk_size + 4;
			unk0x30.pes_packet_remaining_size -= unk_size + 4;
			break;

		case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:
			if (elementary_streams[3][channel])
			{
				elementary_streams[3][channel]->stream_header_buf = v128::loadu(&unk0x30.current_addr[1]);

				if (elementary_streams[3][channel]->au_size == 0)
				{
					unk_size = std::bit_cast<be_t<u32>>(elementary_streams[3][channel]->stream_header_buf._u32[0]) >> 8 & 0xffff;

					ensure(unk_size != 0xffff); // This case is bugged on LLE, likely never happens with valid streams

					elementary_streams[3][channel]->au_size = 1;
				}
			}

			unk0x30.current_addr += unk_size + 4;
			unk0x30.pes_packet_remaining_size -= unk_size + 4;
			break;

		case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA:
			if (elementary_streams[4][channel])
			{
				unk0x30.current_addr -= 2;
				unk0x30.pes_packet_remaining_size += 2;

				if (static_cast<s8>(unk0x30.pes_packet_header[7]) < 0) // PTS field exists
				{
					elementary_streams[4][channel]->stream_header_buf = v128::loadu(&unk0x30.current_addr[2]);
					elementary_streams[4][channel]->au_size = std::bit_cast<be_t<u32>>(elementary_streams[4][channel]->stream_header_buf._u32[0]) - 4; // user data au size

					unk0x30.current_addr += 8;
					unk0x30.pes_packet_remaining_size -= 8;

					const s32 PTS_32_30 = unk0x30.pes_packet_header[9] >> 1;
					const s32 PTS_29_15 = read_from_ptr<be_t<u16>>(&unk0x30.pes_packet_header[10]) >> 1;
					const s32 PTS_14_0 = read_from_ptr<be_t<u16>>(&unk0x30.pes_packet_header[12]) >> 1;

					elementary_streams[4][channel]->dts = umax;
					elementary_streams[4][channel]->pts = PTS_32_30 << 30 | PTS_29_15 << 15 | PTS_14_0; // Bit 32 is discarded
				}
			}

			unk0x30.current_addr += unk_size + 4;
			unk0x30.pes_packet_remaining_size -= unk_size + 4;
			break;

		case DMUX_PAMF_STREAM_TYPE_INDEX_INVALID:
			fmt::throw_exception("Invalid stream id: %d", PES_packet_start_code & 0xff);
		}

		unk0x30.type_idx = type_idx;
		unk0x30.channel = channel;
		unk0x30.pes_packet_data = unk0x30.current_addr;
		unk0x30.pes_packet_data_size = unk0x30.pes_packet_remaining_size;
		unk0x30.end_of_pes_packet = unk0x30.current_addr + unk0x30.pes_packet_remaining_size;

		if (elementary_streams[unk0x30.type_idx][unk0x30.channel])
		{
			const s8 pts_dts_flag = unk0x30.pes_packet_header[7];

			if (pts_dts_flag < 0)
			{
				const s32 PTS_32_30 = unk0x30.pes_packet_header[9] >> 1;
				const s32 PTS_29_15 = read_from_ptr<be_t<u16>>(&unk0x30.pes_packet_header[10]) >> 1;
				const s32 PTS_14_0 = read_from_ptr<be_t<u16>>(&unk0x30.pes_packet_header[12]) >> 1;

				elementary_streams[unk0x30.type_idx][unk0x30.channel]->pts = PTS_32_30 << 30 | PTS_29_15 << 15 | PTS_14_0; // Bit 32 is discarded
			}

			if (pts_dts_flag & 0x40)
			{
				const s32 DTS_32_30 = unk0x30.pes_packet_header[14] >> 1;
				const s32 DTS_29_22 = unk0x30.pes_packet_header[15];
				const s32 DTS_21_15 = unk0x30.pes_packet_header[16] >> 1;
				const s32 DTS_14_7 = unk0x30.pes_packet_header[17];
				const s32 DTS_6_0 = unk0x30.pes_packet_header[18] >> 1;

				elementary_streams[unk0x30.type_idx][unk0x30.channel]->dts = DTS_32_30 << 30 | DTS_29_22 << 22 | DTS_21_15 << 15 | DTS_14_7 << 7 | DTS_6_0; // Bit 32 is discarded
			}
		}

		unk0x30.state = demuxer::state::parsing_elementary_stream;
		[[fallthrough]];
	}
	case demuxer::state::parsing_elementary_stream:
	{
		if (!elementary_streams[unk0x30.type_idx][unk0x30.channel])
		{
			unk0x30.state = demuxer::state::initial;

			return check_demux_done<false>();
		}

		elementary_stream& es = *elementary_streams[unk0x30.type_idx][unk0x30.channel];

		for (;;)
		{
			switch (es._switch)
			{
			case 0:
			{
				if (es.unk0xc0.au_cut_status == 3) // TODO make enum
				{
					es.unk0x10.reset();
					es.unk0xc0.reset();

					if (unk0x30.pes_packet_remaining_size == 0)
					{
						es.reset();

						unk0x30.state = demuxer::state::initial;

						return check_demux_done<false>();
					}
				}

				es.parse_stream_result = es.parse_stream(unk0x30.current_addr, unk0x30.pes_packet_remaining_size, es.unk0x10);

				es._switch = 1;
				[[fallthrough]];
			}
			case 1:
			{
				if (es.unk0x10.au_size != 0 || es.unk0x10.prev_bytes_size != 0)
				{
					if (es.access_unit_queue.end_pos == 0)
					{
						if (es.unk0x10.au_size + es.unk0x10.prev_bytes_size > es.access_unit_queue.size - es.access_unit_queue.pos)
						{
							send_event(DmuxPamfEventType::fatal_error);
							return check_demux_done<true>();
						}
					}
					else
					{
						if (es.access_unit_queue.pos > es.access_unit_queue.freed_mem_size)
						{
							send_event(DmuxPamfEventType::fatal_error);
							return check_demux_done<true>();
						}

						if (es.unk0x10.au_size + sizeof(v128) + es.unk0x10.prev_bytes_size > es.access_unit_queue.freed_mem_size - es.access_unit_queue.pos)
						{
							return check_demux_done<true>();
						}
					}

					es.access_unit_queue.write(es.unk0x10);
				}

				es._switch = 2;
				[[fallthrough]];
			}
			case 2:
			{
				if (!es.start_of_au && es.unk0xc0.au_cut_status == 1)
				{
					es.unk0xc0.pts = es.pts;
					es.unk0xc0.dts = es.dts;
					es.unk0xc0.is_rap = es.is_rap;

					es.reset_timestamps();
					es.start_of_au = true;
					es.is_rap = false;
				}

				es._switch = 3;
				[[fallthrough]];
			}
			case 3:
			{
				if (es.unk0xc0.au_cut_status == 2)
				{
					es.access_unit_queue.pos -= es.unk0xc0.current_au_size;
					es.unk0xc0.current_au_size = 0;

					unk0x30.current_addr = ++unk0x30.pes_packet_data;
					unk0x30.pes_packet_remaining_size = --unk0x30.pes_packet_data_size;

					es._switch = 0;
					break;
				}

				es._switch = 4;
				[[fallthrough]];
			}
			case 4: // set au done params
			{
				if (es.unk0xc0.au_cut_status == 3)
				{
					es.au_params.au_addr = es.access_unit_queue.addr + es.access_unit_queue.pos - es.unk0xc0.current_au_size;
					es.au_params.au_size = es.unk0xc0.current_au_size;

					if (!es.start_of_au && es.unk0x10.prev_bytes_size == 0)
					{
						es.au_params.is_rap = es.is_rap;
						es.au_params.pts = es.pts;
						es.au_params.dts = es.dts;

						es.is_rap = false;
						es.reset_timestamps();
					}
					else
					{
						es.start_of_au = false;
						es.au_params.pts = es.unk0xc0.pts;
						es.au_params.dts = es.unk0xc0.dts;
						es.au_params.is_rap = es.unk0xc0.is_rap;
					}

					es.au_params.stream_header_buf = es.unk0xc0.stream_header_buf;
				}

				es._switch = 5;
				[[fallthrough]];
			}
			case 5:
			{
				if (es.unk0xc0.au_cut_status == 3)
				{
					if (!send_au_found(es.stream_id, es.private_stream_id, es.au_params.au_addr, es.au_params.pts, es.au_params.dts, es.au_params.au_size, es.au_specific_info_size, es.au_params.stream_header_buf, es.es_id, es.au_params.is_rap))
					{
						return check_demux_done<false>();
					}
				}

				es._switch = 7;
				[[fallthrough]];
			}
			case 7:
			{
				if (es.unk0xc0.au_cut_status == 3 && es.access_unit_queue.au_max_size > es.access_unit_queue.size - es.access_unit_queue.pos) // Not enough space for the next AU
				{
					if (es.access_unit_queue.end_pos != 0)
					{
						return check_demux_done<true>();
					}

					es.access_unit_queue.end_pos = es.access_unit_queue.pos;
					es.access_unit_queue.pos = 0;
				}

				es._switch = 6;
				[[fallthrough]];
			}
			case 6:
			{
				if (es.parse_stream_result == 4)
				{
					es.reset();
					es.unk0x10.reset();

					unk0x30.state = demuxer::state::initial;

					return check_demux_done<false>();
				}

				unk0x30.current_addr = es.unk0x10.addr + es.unk0x10.processed_size;
				unk0x30.pes_packet_remaining_size = static_cast<s32>(unk0x30.end_of_pes_packet - unk0x30.current_addr);

				es._switch = 0;
				break;
			}
			default:
				return check_demux_done<true>();
			}
		}
	}
	case demuxer::state::found_prog_end_code:
	{
		if (!send_event(DmuxPamfEventType::prog_end_code))
		{
			unk0x30.state = demuxer::state::found_prog_end_code;

			event_queue_too_full = true;
			return check_demux_done<false>();
		}

		unk0x30.state = demuxer::state::refreshing_buffer_and_finding_start_code;

		return check_demux_done<false>();
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

	if ((new_stream || !demux_done || !demux_done_was_notified) && !au_queue_full && !event_queue_too_full)
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

bool dmux_pamf_context::send_au_found(u8 stream_id, u8 private_stream_id, vm::ptr<u8> au_addr, u64 pts, u64 dts, u32 au_size, u32 au_specific_info_size, v128 stream_header_buf, u32 es_id, bool is_rap)
{
	if (!send_event(DmuxPamfEventType::au_found, stream_id, private_stream_id, au_addr.addr(), std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(pts)),
		std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(dts)), 0, au_size, au_specific_info_size, std::bit_cast<std::array<u8, sizeof(v128)>>(stream_header_buf), es_id, is_rap))
	{
		event_queue_too_full = true;
		return false;
	}

	return true;
}

bool dmux_pamf_context::send_event(auto&&... args)
{
	if (event_queue->size() >= max_enqueued_events)
	{
		return false;
	}

	return ensure(event_queue->emplace(std::forward<decltype(args)>(args)..., +event_queue_was_too_full));
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
			{
				max_enqueued_events += 2;
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::enable_es) + 1));
				enable_es(static_cast<u8>(cmd.enable_es.stream_id), static_cast<u8>(cmd.enable_es.private_stream_id), static_cast<b8>(cmd.enable_es.is_avc), cmd.enable_es.au_queue_buffer_size, cmd.enable_es.au_queue_buffer,
					cmd.enable_es.au_max_size, static_cast<b8>(cmd.enable_es.is_raw_es), cmd.enable_es.es_id);
				break;
			}
			case DmuxPamfCommandType::disable_es:
			{
				disable_es(static_cast<u8>(cmd.disable_flush_es.stream_id), static_cast<u8>(cmd.disable_flush_es.private_stream_id));
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::disable_es) + 1));
				max_enqueued_events -= 2;
				break;
			}
			case DmuxPamfCommandType::set_stream:
			{
				new_stream = true;
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::set_stream) + 1));
				break;
			}
			case DmuxPamfCommandType::free_memory:
			{
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::free_memory) + 1));
				free_memory(cmd.free_memory.mem_size, static_cast<u8>(cmd.free_memory.stream_id), static_cast<u8>(cmd.free_memory.private_stream_id));
				break;
			}
			case DmuxPamfCommandType::flush_es:
			{
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::flush_es) + 1));
				flush_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				break;
			}
			case DmuxPamfCommandType::close:
			{
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::close) + 1));
				while (!send_event(DmuxPamfEventType::close)){}
				return;
			}
			case DmuxPamfCommandType::reset_stream:
			{
				reset_stream();
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::reset_stream) + 1));
				break;
			}
			case DmuxPamfCommandType::reset_es:
			{
				reset_es(cmd.reset_es.stream_id, cmd.reset_es.private_stream_id, vm::ptr<u8, u64>::make(cmd.reset_es.au_addr));
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::reset_es) + 1));
				break;
			}
			case DmuxPamfCommandType::resume:
			{
				ensure(cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCommandType::resume) + 1));
				break;
			}
			default:
			{
				cmd_result_queue->emplace(1000);
			}
			}
		}
		else if (thread_ctrl::state() == thread_state::aborting)
		{
			return;
		}

		if (new_stream && demux_done && demux_done_was_notified)
		{
			new_stream = false;

			DmuxPamfStreamInfo stream_info;
			ensure(stream_info_queue->pop(stream_info));

			demux(&stream_info);
		}
		else
		{
			demux(nullptr);
		}
	}
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
	ensure(cmd_result_queue.pop(result));
	ensure(result == static_cast<u32>(type) + 1, "The HLE SPU thread sent an invalid result");
}

DmuxPamfElementaryStream* DmuxPamfContext::find_es(u16 stream_id, u16 private_stream_id)
{
	const auto type = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first;

	DmuxPamfElementaryStream* ret = nullptr;

	if (type == DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO)
	{
		for (const vm::bptr<DmuxPamfElementaryStream> es : elementary_streams)
		{
			if (es && es->stream_id == stream_id)
			{
				ret = es.get_ptr();
			}
		}
	}
	else
	{
		for (const vm::bptr<DmuxPamfElementaryStream> es : elementary_streams)
		{
			if (es && es->stream_id == stream_id && es->private_stream_id == private_stream_id)
			{
				ret = es.get_ptr();
			}
		}
	}

	return ret;
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
#define SEND_FATAL_ERR_AND_CONTINUE() \
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
						es->au_specific_info[0] = event.au_found.stream_header_buf[0] >> 4;
						es->au_specific_info[1] = static_cast<u8>(event.au_found.stream_header_buf[0] & 0xf);
						es->au_specific_info[2] = event.au_found.stream_header_buf[1] >> 6;
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
		//dmux->stop_spu_thread();
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
	//stop_spu_thread();

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
error_code DmuxPamfContext::set_stream(ppu_thread& ppu, vm::cptr<void> stream_address, u32 stream_size, b8 discontinuity, u32 user_data)
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
error_code _CellDmuxCoreOpSetStream(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle, vm::cptr<void> streamAddress, u32 streamSize, b8 discontinuity, u64 userData)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpSetStream<raw_es=%d>(handle=*0x%x, streamAddress=*0x%x, streamSize=0x%x, discontinuity=%d, userData=0x%llx)", raw_es, handle, streamAddress, streamSize, +discontinuity, userData); // TODO trace
	cellDmuxPamf.warning("_CellDmuxCoreOpSetStream(): stream address: *0x%x", streamAddress.get_ptr()); // TODO remove

	if (!streamAddress || streamSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!handle); // Not checked on LLE

	return handle->demuxer->set_stream<raw_es>(ppu, streamAddress, streamSize, discontinuity, static_cast<u32>(userData));
}

error_code DmuxPamfElementaryStream::free_memory(ppu_thread& ppu, vm::ptr<void> mem_addr, u32 mem_size) const
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
	demuxer->send_spu_command_and_wait<DmuxPamfCommandType::free_memory>(ppu, savestate, mem_addr.addr(), mem_size, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id));

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

error_code _CellDmuxCoreOpFreeMemory(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle, vm::ptr<void> memAddr, u32 memSize)
{
	cellDmuxPamf.trace("_CellDmuxCoreOpFreeMemory(esHandle=*0x%x, memAddr=*0x%x, memSize=0x%x)", esHandle, memAddr, memSize);

	if (!memAddr || memSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!esHandle); // Not checked on LLE

	return esHandle->es->free_memory(ppu, memAddr, memSize);
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

	const auto au_queue_buffer = vm::bptr<u8>::make(utils::align(_es.addr() + static_cast<u32>(sizeof(DmuxPamfElementaryStream)), 0x80));
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

	demuxer->send_spu_command_and_wait<DmuxPamfCommandType::reset_es>(ppu, waiting_for_spu_state, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id), 0);

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
	var->freeMemory.set(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(_CellDmuxCoreOpFreeMemory)));
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
	REG_HIDDEN_FUNC(_CellDmuxCoreOpFreeMemory);
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
