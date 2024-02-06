#include "stdafx.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/Cell/lv2/sys_sync.h"
#include "Emu/Cell/lv2/sys_ppu_thread.h"
#include "Emu/savestate_utils.hpp"
#include "Emu/System.h"
#include "sysPrxForUser.h"
#include "util/simd.hpp"

#include "cellPamf.h"
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

std::pair<DmuxPamfStreamTypeIndex, u32> dmuxPamfStreamIdToTypeChannel(u16 stream_id, u16 private_stream_id)
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

void au_queue::push(es_0x10& unk2)
{
	const vm::ptr<u8, u64> ea = addr + pos;

	std::memcpy(ea.get_ptr(), &unk2.prev_bytes, unk2.prev_bytes_size);
	std::memcpy(ea.get_ptr() + unk2.prev_bytes_size, unk2.addr, unk2.au_size);

	pos += unk2.prev_bytes_size + unk2.au_size;
}

template <bool is_avc>
u32 demux_es::parse_video(const u8* input_addr, u32 input_size, es_0x10& unk4)
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
u32 demux_es::parse_lpcm(const u8* input_addr, u32 input_size, es_0x10& unk4)
{
	unk4.addr = input_addr;

	if (unk0xc0.au_cut_status == 0)
	{
		unk0xc0.stream_header_buf = stream_header_buf;
		unk0xc0.stream_header_size = stream_header_size;
	}
	else
	{
		unk4.prev_bytes_size = 0;
	}

	const u32 remaining_au_bytes = _au_queue.au_max_size - unk0xc0.current_au_size;

	if (remaining_au_bytes > input_size)
	{
		unk4.au_size = input_size;
		unk4.processed_size = input_size;

		unk0xc0.current_au_size += input_size;

		if (unk0xc0.au_cut_status == 0)
		{
			unk0xc0.au_cut_status = 1;

			return 4;
		}

		unk0xc0.au_cut_status = 5;

		return 4;
	}

	unk4.au_size = remaining_au_bytes;
	unk4.processed_size = remaining_au_bytes;

	unk0xc0.current_au_size += remaining_au_bytes;
	unk0xc0.au_cut_status = 3;

	return 5;
}

// TODO: SIMPLIFY, VARIABLE NAMES
#pragma message(": warning: TODO")
template <bool is_ac3>
u32 demux_es::parse_audio(const u8* input_addr, u32 input_size, es_0x10& unk4)
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

u32 demux_es::parse_user_data(const u8* addr, u32 size, es_0x10& unk4)
{
	unk4.addr = addr;
	unk4.prev_bytes_size = 0;
	unk4.processed_size = size;

	if (sizeUNK > size)
	{
		unk4.au_size = size;
		unk0xc0.current_au_size += size;
		sizeUNK -= size;

		if (unk0xc0.au_cut_status == 0)
		{
			unk0xc0.au_cut_status = 1;
		}

		return 4;
	}

	unk4.au_size = sizeUNK;
	unk0xc0.current_au_size += sizeUNK;
	sizeUNK = 0;

	unk0xc0.au_cut_status = 3;

	return 5;
}

bool DmuxPamfSPUContext::enable_es(u8 stream_id, u8 private_stream_id, b8 is_avc, u32 au_queue_buffer_size, vm::ptr<u8, u64> au_queue_buffer, u32 au_max_size, b8 is_raw_es, u32 es_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	demux_es& es = elementary_streams[type_idx][channel];

	if (es.is_enabled)
	{
		return false;
	}

	if (au_max_size == umax || au_max_size > au_queue_buffer_size)
	{
		au_max_size = 0x800;
	}

	es.is_enabled = true;

	es._au_queue.init(au_queue_buffer, au_queue_buffer_size, au_max_size);

	es.is_avc = is_avc;
	es.sizeUNK = 0;
	es.start_of_au = false;

	es.reset();
	es.reset_timestamps();
	es.unk0x10.reset();
	es.unk0xc0.reset();

	es.au_done_params.stream_id = stream_id;
	es.au_done_params.private_stream_id = private_stream_id;
	es.au_done_params.es_id = es_id;

	this->is_raw_es = is_raw_es;

	if (is_raw_es)
	{
		es_type_idx = type_idx;
	}

	return true;
}

bool DmuxPamfSPUContext::disable_es(u8 stream_id, u8 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	if (!elementary_streams[type_idx][channel].is_enabled)
	{
		return false;
	}

	elementary_streams[type_idx][channel].is_enabled = false;
	return true;
}

bool DmuxPamfSPUContext::free_memory(u32 mem_size, u8 stream_id, u8 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	demux_es& es = elementary_streams[type_idx][channel];

	if (!es.is_enabled)
	{
		return false;
	}

	if (es._au_queue.end_pos != 0)
	{
		const u32 new_freed_mem_size = es._au_queue.freed_mem_size + mem_size;

		if (es._au_queue.end_pos <= new_freed_mem_size)
		{
			if (es._au_queue.end_pos < new_freed_mem_size) // Shouldn't be possible to happen
			{
				cellDmuxPamf.warning("Not supposed to happen"); // TODOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

				es._au_queue.freed_mem_size = new_freed_mem_size - es._au_queue.end_pos;
				es._au_queue.end_pos = 0;
			}
			else
			{
				es._au_queue.freed_mem_size = 0;
				es._au_queue.end_pos = 0;
			}

			return true;
		}
	}

	es._au_queue.freed_mem_size += mem_size;

	return true;
}

bool DmuxPamfSPUContext::flush_es(u8 stream_id, u8 private_stream_id)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	demux_es& es = elementary_streams[type_idx][channel];

	if (!es.is_enabled)
	{
		return false;
	}

	if (es.unk0xc0.current_au_size != 0)
	{
		es.unk0x10.processed_size = 0;
		es.unk0x10.au_size = 0;
		es.unk0x10.addr = nullptr;

		const u32 prev_bytes_size = sizeof(v128) - es.unk0xc0.cache_start_idx;
		es.unk0x10.prev_bytes_size = prev_bytes_size;

		es.unk0x10.prev_bytes.clear();
		std::memcpy(&es.unk0x10.prev_bytes, &es.unk0xc0.prev_packet_cache._bytes[es.unk0xc0.cache_start_idx], prev_bytes_size);

		es._au_queue.push(es.unk0x10);

		const u32 au_size = es.unk0xc0.current_au_size - es.unk0xc0.cache_start_idx + 0x10;
		const u32 au_start_offset = es._au_queue.pos - au_size;

		es.unk0xc0.current_au_size = au_size;

		es.au_done_params.au_addr = es._au_queue.addr + au_start_offset;
		es.au_done_params.is_rap = es.unk0xc0.is_rap;
		es.au_done_params.au_size = au_size;
		es.au_done_params.pts = es.unk0xc0.pts;
		es.au_done_params.dts = es.unk0xc0.dts;

		send_au_done(es.au_done_params);
	}

	es.sizeUNK = 0;
	es.start_of_au = false;

	es.reset();
	es.reset_timestamps();
	es.unk0x10.reset();
	es.unk0xc0.reset();

	unk0x30.main_switchUNK = 0;

	while (!send_event(DmuxPamfEventType::flush_done, stream_id, private_stream_id, es.au_done_params.es_id));

	return true;
}

void DmuxPamfSPUContext::reset_stream()
{
	for (demux_es (&types)[0x10] : elementary_streams)
	{
		for (demux_es& es : types)
		{
			if (!es.is_enabled)
			{
				continue;
			}

#pragma message(": warning: TODO")
			const u32 sizeUNK_1_add_0x1c = es.unk0x10.au_size + es.unk0x10.prev_bytes_size;
			const u32 unk_0xc8_sub = es.unk0xc0.current_au_size - sizeUNK_1_add_0x1c;
			const u32 unk_0xac_sub = es._au_queue.pos - unk_0xc8_sub;

			es._au_queue.pos = unk_0xac_sub;

			es.sizeUNK = 0;
			es.start_of_au = false;

			es.reset();
			es.reset_timestamps();
			es.unk0x10.reset();
			es.unk0xc0.reset();

			es.unk0xc0.cache_start_idx = 0;

			unk0x30.main_switchUNK = 0;
		}
	}

	unk0x30.reset();
	unk0x90.reset();
	input_stream.init(vm::null, 0);
}

bool DmuxPamfSPUContext::reset_es(u8 stream_id, u8 private_stream_id, vm::ptr<u8, u64> au_addr)
{
	const auto [type_idx, channel] = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id);

	if (type_idx == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return false;
	}

	demux_es& es = elementary_streams[type_idx][channel];

	if (!es.is_enabled)
	{
		return false;
	}

	if (!au_addr)
	{
		es.sizeUNK = 0;
		es.start_of_au = false;

		es.reset();
		es.reset_timestamps();
		es.unk0x10.reset();
		es.unk0xc0.reset();

		es._au_queue.init(es._au_queue.addr, es._au_queue.size, es._au_queue.au_max_size);

		unk0x30.main_switchUNK = 0;
	}
	else
	{
		const u32 au_offset = au_addr - es._au_queue.addr;

		if (es._au_queue.end_pos != 0)
		{
			if (au_offset > es._au_queue.pos)
			{
				es._au_queue.end_pos = 0;
			}
		}

		es._au_queue.pos = au_offset;
	}

	return true;
}

void DmuxPamfSPUContext::FUN_00006cc8()
{
	for (demux_es (&types)[0x10] : elementary_streams)
	{
		for (demux_es& es : types)
		{
			if (!es.is_enabled)
			{
				continue;
			}

			if (es.unk0xc0.current_au_size != 0)
			{
				es._au_queue.pos -= es.unk0xc0.current_au_size;
			}

			es.sizeUNK = 0;
			es.start_of_au = false;

			es.reset();
			es.reset_timestamps();
			es.unk0x10.reset();
			es.unk0xc0.reset();

			es.unk0xc0.cache_start_idx = 0;

			unk0x30.main_switchUNK = 0;
		}
	}
}

inline bool DmuxPamfSPUContext::check_and_notify_demux_done()
{
	if (!demux_done)
	{
		return true;
	}

	return check_demux_done_was_notified();
}

inline bool DmuxPamfSPUContext::check_demux_done_was_notified()
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

bool DmuxPamfSPUContext::demux(const DmuxPamfStreamInfo* stream_info)
{
	if (stream_info)
	{
		if (demux_done)
		{
			if (!demux_done_was_notified)
			{
				au_queue_no_memory = false;

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
				FUN_00006cc8();
			}

			unk0x30.reset();
			unk0x90.reset();

			input_stream.init(stream_info->stream_addr, stream_info->stream_size);

			demux_done = false;
			demux_done_was_notified = false;
		}
	}

	if (demux_done)
	{
		au_queue_no_memory = false;
		return check_demux_done_was_notified();
	}

	s32 end_of_pack_stuffing_bytes_offset = 0;
	v128 PES_header_buf_2 = {};

	switch (unk0x30.main_switchUNK)
	{
	case 0:
	{
		unk0x30.main_switchUNK = 1;

		[[fallthrough]];
	}
	case 1:
	{
		for (;;)
		{
			if (unk0x30.sequence_stateUNK == 4)
			{
				if (input_stream.bytes_in_buffer > 0x800)
				{
					input_stream.processed_bytes += 0x800;
					input_stream.bytes_in_buffer -= 0x800;
				}
				else
				{
					input_stream.processed_bytes = 0;

					if (input_stream.pos > input_stream.size)
					{
						unk0x30.input_addr = nullptr;

						demux_done = true;
						au_queue_no_memory = false;
						return check_demux_done_was_notified();
					}

					const u32 remaining_size = input_stream.size - input_stream.pos;

					u32 bytes_to_copy = sizeof(input_buffer);

					if (remaining_size < sizeof(input_buffer))
					{
						if (remaining_size < sizeof(v128))
						{
							unk0x30.input_addr = nullptr;

							demux_done = true;
							au_queue_no_memory = false;
							return check_demux_done_was_notified();
						}

						bytes_to_copy = remaining_size & -0x10; // should probably be -0x800, packs are always 0x800 bytes in size
					}

					std::memcpy(input_buffer, input_stream.addr.get_ptr() + input_stream.pos, bytes_to_copy);

					input_stream.pos += bytes_to_copy;
					input_stream.dma_count++;
					input_stream.bytes_in_buffer = bytes_to_copy;
				}

				input_stream.bytes_to_process = 0x800; // pack size

				unk0x30.input_addr = input_buffer + input_stream.processed_bytes;
			}

			if (is_raw_es)
			{
				unk0x30.channel = 0;
				unk0x30.main_switchUNK = 7;
				unk0x30.type_idx = es_type_idx;
				unk0x30.pes_packet_remaining_size = input_stream.bytes_to_process;
				unk0x30.end_of_pes_packet = unk0x30.input_addr + input_stream.bytes_to_process;

				au_queue_no_memory = false;
				return check_and_notify_demux_done();
			}

			unk0x90.search_start_addr = unk0x30.input_addr;
			u32 input_addr_low = static_cast<u32>(reinterpret_cast<usz>(unk0x30.input_addr) & 0xf);
			const u8* const input_addr_16_aligned = unk0x30.input_addr - input_addr_low;
			unk0x90.found_code = 0;

			const s32 bytes_to_process = input_stream.bytes_to_process;

			v128 input[2];
			const u8* next_input_addr;
			u32 input_u32 = 0;
			u32 current_offset;
			u32 max_index = 0x10;
			const u8* const end_of_input_addr = unk0x30.input_addr + bytes_to_process;

			if (unk0x90.matched_bytes == 3)
			{
				input[0] = gv_bcst32(std::bit_cast<be_t<u32>>(0x100u)); // this seems bugged
				current_offset = -0x10;
				input_addr_low = 0xd;
				next_input_addr = nullptr;
				unk0x90.matched_bytes = 0;
				input[1] = *reinterpret_cast<const v128*>(input_addr_16_aligned);
			}
			else if (unk0x90.matched_bytes == 2)
			{
				input[0] = {};
				current_offset = -0x10;
				input_addr_low = 0xe;
				next_input_addr = 0;
				unk0x90.matched_bytes = 0;
				input[1] = *reinterpret_cast<const v128*>(input_addr_16_aligned);
			}
			else if (unk0x90.matched_bytes == 1)
			{
				current_offset = -0x10;
				input[0] = {};
				input_addr_low = 0xf;
				next_input_addr = 0;
				unk0x90.matched_bytes = 0;
				input[1] = *reinterpret_cast<const v128*>(input_addr_16_aligned);
			}
			else
			{
				current_offset = 0;
				goto _unk_label_1;
			}

			u32 unk_status;

			for (;;)
			{
				for (u32 i = input_addr_low; i < max_index; i++)
				{
					// input_u32 = spu_extract(spu_shuffle(spu_slqwbyte(input_1, i), input_2, shufUNK[i]), 0);
					input_u32 = read_from_ptr<be_t<u32>>(input[0]._bytes, i);

					if (input_u32 == 0x1ba) // pack start code
					{
						unk_status = 1;
						unk0x90.processed_bytes += static_cast<u32>(next_input_addr + i - unk0x90.search_start_addr);
						unk0x90.found_code = 1;
						goto _unk_label_2;
					}

					if (input_u32 == 0x1b9) // program end code
					{
						unk_status = 6;
						unk0x90.processed_bytes += static_cast<u32>(next_input_addr + i - unk0x90.search_start_addr);
						unk0x90.found_code = 6;
						goto _unk_label_2;
					}
				}

				current_offset += 0x10;
				input_addr_low = 0;

			_unk_label_1:
				next_input_addr = input_addr_16_aligned + current_offset;
				const s32 bytes_left = static_cast<s32>(end_of_input_addr - next_input_addr);

				if (bytes_left < 4)
				{
					break;
				}

				input[0] = *reinterpret_cast<const v128*>(next_input_addr);
				input[1] = *reinterpret_cast<const v128*>(next_input_addr + sizeof(v128));

				if (bytes_left < 0x14)
				{
					if (bytes_left < 0x10)
					{
						max_index = bytes_left - 4;
					}
					else
					{
						max_index = max_index - bytes_left + 0x10;
					}
				}
			}

			if (input_u32 == 1)
			{
				unk0x90.matched_bytes = 3;
			}
			else if (!(input_u32 & 0xffff))
			{
				unk0x90.matched_bytes = 2;
			}
			else if (!(input_u32 & 0xff))
			{
				unk0x90.matched_bytes = 1;
			}

			unk_status = 4;
			unk0x90.processed_bytes += unk0x90.search_start_addr ? static_cast<u32>(end_of_input_addr - unk0x90.search_start_addr) : 0;

		_unk_label_2:
			unk0x30.sequence_stateUNK = unk_status;

			if (unk0x90.found_code == 1)
			{
				unk0x30.main_switchUNK = 2;
				unk0x30.input_addr = unk0x90.search_start_addr; // no +processed bytes ?
				unk0x30.unk0x44 = 0;
				input_stream.bytes_to_process -= unk0x90.processed_bytes;
				break;
			}

			if (unk0x90.found_code == 6)
			{
				if (!send_event(DmuxPamfEventType::prog_end_code))
				{
					unk0x30.main_switchUNK = 8;

					au_queue_no_memory = false;
					event_queue_too_full = true;
					return check_and_notify_demux_done();
				}

				unk0x30.sequence_stateUNK = 4;
				unk0x30.main_switchUNK = 1;

				au_queue_no_memory = false;
				return check_and_notify_demux_done();
			}
		}

		[[fallthrough]];
	}
	case 2: // skip pack_header
	{
		if (PACK_STUFFING_LENGTH_OFFSET - unk0x30.unk0x44 > input_stream.bytes_to_process)
		{
			unk0x30.main_switchUNK = 2;

			unk0x30.unk0x44 += input_stream.bytes_to_process;
			input_stream.bytes_to_process = 0;

			demux_done = true;
			au_queue_no_memory = false;
			return check_demux_done_was_notified();
		}

		const s32 pack_stuffing_length = read_from_ptr<u8>(unk0x30.input_addr + PACK_STUFFING_LENGTH_OFFSET - unk0x30.unk0x44) & 0x7;

		end_of_pack_stuffing_bytes_offset = pack_stuffing_length + PACK_STUFFING_LENGTH_OFFSET + sizeof(u8);

		unk0x30.main_switchUNK = 3;
		[[fallthrough]];
	}
	case 3: // after pack stuffing_byte
	{
		if (end_of_pack_stuffing_bytes_offset > input_stream.bytes_to_process)
		{
			unk0x30.main_switchUNK = 3;

			unk0x30.unk0x44 += input_stream.bytes_to_process;
			input_stream.bytes_to_process = 0;

			demux_done = true;
			au_queue_no_memory = false;
			return check_demux_done_was_notified();
		}

		unk0x30.input_addr += end_of_pack_stuffing_bytes_offset;

		std::memcpy(&unk0x30.PES_header_buf, unk0x30.input_addr, sizeof(v128));

		if (!(std::bit_cast<be_t<u32>>(unk0x30.PES_header_buf._u32[0]) & 0x100)) // check for start code
		{
			unk0x30.main_switchUNK = 1;
			unk0x30.sequence_stateUNK = 4;

			au_queue_no_memory = false;
			return check_and_notify_demux_done();
		}

		unk0x30.main_switchUNK = 4;
		[[fallthrough]];
	}
	case 4: // next start code
	{
		if (std::bit_cast<be_t<u32>>(unk0x30.PES_header_buf._u32[0]) == 0x1bb) // system_header_start_code
		{
			const u16 header_length = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[2]);

			unk0x30.input_addr += header_length + 6;

			const v128 buf = *reinterpret_cast<const v128*>(reinterpret_cast<usz>(unk0x30.input_addr) & -0x10); // LLE isn't doing an unaligned load here

			if (!(std::bit_cast<be_t<u32>>(buf._u32[0]) & 0x100)) // check for start code
			{
				unk0x30.main_switchUNK = 1;
				unk0x30.sequence_stateUNK = 4;

				au_queue_no_memory = false;
				return check_and_notify_demux_done();
			}

			if (std::bit_cast<be_t<u32>>(buf._u32[0]) == 0x1bf) // private_stream_2
			{
				const u16 PES_packet_length = std::bit_cast<be_t<u16>>(buf._u16[2]);

				const u32 channel = buf._u8[7] & 0xf;

				unk0x30.input_addr += PES_packet_length + 6;

				if (elementary_streams[0][channel].is_enabled)
				{
					elementary_streams[0][channel].is_rap = true;
				}
			}
		}

		std::memcpy(&unk0x30.PES_header_buf, unk0x30.input_addr, sizeof(v128));
		std::memcpy(&PES_header_buf_2, unk0x30.input_addr + sizeof(v128), sizeof(v128));

		if (!(std::bit_cast<be_t<u32>>(unk0x30.PES_header_buf._u32[0]) & 0x100)) // check for packet_start_code_prefix
		{
			unk0x30.main_switchUNK = 1;
			unk0x30.sequence_stateUNK = 4;

			au_queue_no_memory = false;
			return check_and_notify_demux_done();
		}

		unk0x30.main_switchUNK = 5;
		[[fallthrough]];
	}
	case 5: // packet_start_code_prefix
	{
		const u16 PES_packet_length = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[2]);
		const u8 PES_header_data_length = unk0x30.PES_header_buf._u8[8];

		const u32 total_pes_packet_length = PES_packet_length + 6;
		const u32 total_pes_header_length = PES_header_data_length + 9;

		unk0x30.pes_packet_remaining_size = total_pes_packet_length - total_pes_header_length;
		unk0x30.input_addr += total_pes_header_length;

		unk0x30.main_switchUNK = 6;
		[[fallthrough]];
	}
	case 6: // PES header
	{
		const u32 PES_packet_start_code = std::bit_cast<be_t<u32>>(unk0x30.PES_header_buf._u32[0]);

		if ((PES_packet_start_code & -0x10) == 0x1e0) // video
		{
			const u32 channel = PES_packet_start_code & 0xf;

			unk0x30.type_idx = 0;
			unk0x30.channel = channel;
			elementary_streams[0][channel].stream_header_size = 0;
		}
		else if (PES_packet_start_code == 0x1bd) // private_data_stream_1
		{
			const v128 buf = read_from_ptr<v128>(unk0x30.input_addr);

			const u8 private_stream_id = buf._u8[0];

			const u32 type = private_stream_id & 0xf0;
			const u32 channel = private_stream_id & 0xf;
			unk0x30.channel = channel;

#pragma message(": warning: TODO")
			s32 unk = 0;

			if (type == 0x40) // LPCM
			{
				unk0x30.type_idx = 1;

				elementary_streams[1][channel].stream_header_buf = gv_shuffle_right<1>(buf); // Shift towards lesser address
				elementary_streams[1][channel].stream_header_size = 3;

				if (elementary_streams[1][channel].sizeUNK == 0)
				{
					unk = std::bit_cast<be_t<u32>>(elementary_streams[1][channel].stream_header_buf._u32[0]) >> 8 & 0x7ff;

					if (unk == 0x7ff)
					{
						unk0x30.main_switchUNK = 0;

						au_queue_no_memory = false;
						return check_and_notify_demux_done();
					}

					elementary_streams[1][channel].sizeUNK = 1;
				}
			}
			else if (type == 0x30) // AC3
			{
				unk0x30.type_idx = 2;

				elementary_streams[2][channel].stream_header_buf = gv_shuffle_right<1>(buf); // Shift towards lesser address
				elementary_streams[2][channel].stream_header_size = 0;

				if (elementary_streams[2][channel].sizeUNK == 0)
				{
					unk = std::bit_cast<be_t<u32>>(elementary_streams[2][channel].stream_header_buf._u32[0]) >> 8 & 0xffff;

					if (unk == 0xffff) // padding packet ????? gets skipped
					{
						unk0x30.main_switchUNK = 0;

						au_queue_no_memory = false;
						return check_and_notify_demux_done();
					}

					elementary_streams[2][channel].sizeUNK = 1;
				}
			}
			else if (type == 0) // ATRAC3plus
			{
				unk0x30.type_idx = 3;

				elementary_streams[3][channel].stream_header_buf = gv_shuffle_right<1>(buf); // Shift towards lesser address
				elementary_streams[3][channel].stream_header_size = 0;

				if (elementary_streams[3][channel].sizeUNK == 0)
				{
					unk = std::bit_cast<be_t<u32>>(elementary_streams[3][channel].stream_header_buf._u32[0]) >> 8 & 0xffff;

					if (unk == 0xffff)
					{
						unk0x30.main_switchUNK = 0;

						au_queue_no_memory = false;
						return check_and_notify_demux_done();
					}

					elementary_streams[3][channel].sizeUNK = 1;
				}
			}
			else if (type == 0x20) // User data
			{
				unk0x30.input_addr -= 2;
				unk0x30.pes_packet_remaining_size += 2;

				unk0x30.type_idx = 4;

				elementary_streams[4][channel].stream_header_size = 0;

				if (unk0x30.PES_header_buf._s8[7] < 0) // PTS field exists
				{
					const v128 tmp = gv_shuffle_right<2>(buf); // Shift towards lesser address

					elementary_streams[4][channel].stream_header_buf = tmp;
					elementary_streams[4][channel].sizeUNK = std::bit_cast<be_t<u32>>(tmp._u32[0]) - 4; // user data au size

					unk0x30.input_addr += 8;
					unk0x30.pes_packet_remaining_size -= 8;

					const s32 PTS_32_30 = unk0x30.PES_header_buf._u8[9] >> 1;
					const s32 PTS_29_15 = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[5]) >> 1;
					const s32 PTS_14_0 = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[6]) >> 1;

					elementary_streams[4][channel].dts = umax;
					elementary_streams[4][channel].pts = PTS_32_30 << 30 /* bit 32 is discarded */ | PTS_29_15 << 15 | PTS_14_0;
				}
			}
			else
			{
				unk0x30.main_switchUNK = 0;

				au_queue_no_memory = false;
				return check_and_notify_demux_done();
			}

			unk0x30.input_addr += unk + 4;
			unk0x30.pes_packet_remaining_size -= unk + 4;
		}
		else
		{
			unk0x30.main_switchUNK = 0;

			au_queue_no_memory = false;
			return check_and_notify_demux_done();
		}

		unk0x30.unk0x38 = unk0x30.input_addr;
		unk0x30.unk0x40 = unk0x30.pes_packet_remaining_size;
		unk0x30.end_of_pes_packet = unk0x30.input_addr + unk0x30.pes_packet_remaining_size;

		if (elementary_streams[unk0x30.type_idx][unk0x30.channel].is_enabled)
		{
			const s8 pts_dts_flag = unk0x30.PES_header_buf._s8[7];

			if (pts_dts_flag < 0)
			{
				const s32 PTS_32_30 = unk0x30.PES_header_buf._u8[9] >> 1;
				const s32 PTS_29_15 = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[5]) >> 1;
				const s32 PTS_14_0 = std::bit_cast<be_t<u16>>(unk0x30.PES_header_buf._u16[6]) >> 1;

				elementary_streams[unk0x30.type_idx][unk0x30.channel].pts = PTS_32_30 << 30 /* bit 32 is discarded */ | PTS_29_15 << 15 | PTS_14_0;
			}

			if (pts_dts_flag & 0x40)
			{
				const s32 DTS_32_30 = unk0x30.PES_header_buf._u8[14] >> 1;
				const s32 DTS_29_22 = unk0x30.PES_header_buf._u8[15];
				const s32 DTS_21_15 = PES_header_buf_2._u8[0] >> 1;
				const s32 DTS_14_7 = PES_header_buf_2._u8[1];
				const s32 DTS_6_0 = PES_header_buf_2._u8[2] >> 1;

				elementary_streams[unk0x30.type_idx][unk0x30.channel].dts = DTS_32_30 << 30 /* bit 32 is discarded */ | DTS_29_22 << 22 | DTS_21_15 << 15 | DTS_14_7 << 7 | DTS_6_0;
			}
		}

		unk0x30.main_switchUNK = 7;
		[[fallthrough]];
	}
	case 7:
	{
		demux_es& es = elementary_streams[unk0x30.type_idx][unk0x30.channel];

		if (!es.is_enabled)
		{
			unk0x30.main_switchUNK = 0;
			unk0x30.sequence_stateUNK = 4;

			au_queue_no_memory = false;
			return check_and_notify_demux_done();
		}

		for (;;)
		{
			switch (es._switch)
			{
			case 0:
			{
				if (es.unk0xc0.au_cut_status == 3)
				{
					es.unk0x10.reset();
					es.unk0xc0.reset();

					if (unk0x30.pes_packet_remaining_size == 0)
					{
						es.reset();
						es.unk0x10.reset();

						unk0x30.main_switchUNK = 0;
						unk0x30.sequence_stateUNK = 4;

						au_queue_no_memory = false;
						return check_and_notify_demux_done();
					}
				}

				switch (unk0x30.type_idx)
				{
				case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
					if (es.is_avc)
					{
						es.parse_stream_result = es.parse_video<true>(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10);
					}
					else
					{
						es.parse_stream_result = es.parse_video<false>(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10);
					}
					break;

				case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:      es.parse_stream_result = es.parse_lpcm(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10); break;
				case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:       es.parse_stream_result = es.parse_audio<true>(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10); break;
				case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:    es.parse_stream_result = es.parse_audio<false>(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10); break;
				case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA: es.parse_stream_result = es.parse_user_data(unk0x30.input_addr, unk0x30.pes_packet_remaining_size, es.unk0x10); break;
				}

				es._switch = 1;
				[[fallthrough]];
			}
			case 1:
			{
				if (es.unk0x10.au_size != 0 || es.unk0x10.prev_bytes_size != 0)
				{
					if (es._au_queue.end_pos == 0)
					{
						if (es.unk0x10.au_size + es.unk0x10.prev_bytes_size > es._au_queue.size - es._au_queue.pos)
						{
							send_event(DmuxPamfEventType::fatal_error);
							au_queue_no_memory = true;
							return check_and_notify_demux_done();
						}
					}
					else
					{
						if (es._au_queue.pos > es._au_queue.freed_mem_size)
						{
							send_event(DmuxPamfEventType::fatal_error);
							au_queue_no_memory = true;
							return check_and_notify_demux_done();
						}

						if (es.unk0x10.au_size + sizeof(v128) + es.unk0x10.prev_bytes_size > es._au_queue.freed_mem_size - es._au_queue.pos)
						{
							au_queue_no_memory = true;
							return check_and_notify_demux_done();
						}
					}

					es._au_queue.push(es.unk0x10);
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

					es.start_of_au = true;
					es.pts = umax;
					es.dts = umax;
					es.is_rap = false;
				}

				es._switch = 3;
				[[fallthrough]];
			}
			case 3:
			{
				if (es.unk0xc0.au_cut_status == 2)
				{
					es._au_queue.pos -= es.unk0xc0.current_au_size;
					es.unk0xc0.current_au_size = 0;

					unk0x30.input_addr = ++unk0x30.unk0x38;
					unk0x30.pes_packet_remaining_size = --unk0x30.unk0x40;

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
					es.au_done_params.au_addr = es._au_queue.addr + es._au_queue.pos - es.unk0xc0.current_au_size;
					es.au_done_params.au_size = es.unk0xc0.current_au_size;

					if (!es.start_of_au && es.unk0x10.prev_bytes_size == 0)
					{
							const u64 pts = es.pts;
							const u64 dts = es.dts;

							es.pts = umax;
							es.dts = umax;

							const b8 is_rap = es.is_rap;

							es.is_rap = false;

							es.au_done_params.is_rap = is_rap;
							es.au_done_params.pts = pts;
							es.au_done_params.dts = dts;
					}
					else
					{
						es.start_of_au = false;
						es.au_done_params.pts = es.unk0xc0.pts;
						es.au_done_params.dts = es.unk0xc0.dts;
						es.au_done_params.is_rap = es.unk0xc0.is_rap;
					}

					es.au_done_params.stream_header_buf = es.unk0xc0.stream_header_buf;
					es.au_done_params.au_specific_info_size = es.unk0xc0.stream_header_size;
					es.au_done_params.unk = 0;
				}

				es._switch = 5;
				[[fallthrough]];
			}
			case 5:
			{
				if (es.unk0xc0.au_cut_status == 3)
				{
					if (!send_au_done(es.au_done_params))
					{
						au_queue_no_memory = false;
						return check_and_notify_demux_done();
					}

					es.au_found_num++;
				}

				es._switch = 7;
				[[fallthrough]];
			}
			case 7:
			{
				if (es.unk0xc0.au_cut_status == 3 && es._au_queue.au_max_size > es._au_queue.size - es._au_queue.pos) // Not enough space for next AU
				{
					if (es._au_queue.end_pos != 0)
					{
						au_queue_no_memory = true;
						return check_and_notify_demux_done();
					}

					es._au_queue.end_pos = es._au_queue.pos;
					es._au_queue.pos = 0;
				}

				es._switch = 6;
				[[fallthrough]];
			}
			case 6:
			{
				if (es.parse_stream_result == 4)
				{
					elementary_streams[unk0x30.type_idx][unk0x30.channel].reset();
					elementary_streams[unk0x30.type_idx][unk0x30.channel].unk0x10.reset();

					unk0x30.main_switchUNK = 0;
					unk0x30.sequence_stateUNK = 4;

					au_queue_no_memory = false;
					return check_and_notify_demux_done();
				}

				unk0x30.input_addr = es.unk0x10.addr + es.unk0x10.processed_size;
				unk0x30.pes_packet_remaining_size = static_cast<s32>(unk0x30.end_of_pes_packet - unk0x30.input_addr);

				es._switch = 0;
				break;
			}
			default:
				au_queue_no_memory = true;
				return check_and_notify_demux_done();
			}
		}
	}
	case 8:
	{
		if (unk0x90.found_code != 6)
		{
			au_queue_no_memory = false;
			return check_and_notify_demux_done();
		}

		if (!send_event(DmuxPamfEventType::prog_end_code))
		{
			unk0x30.main_switchUNK = 8;

			au_queue_no_memory = false;
			event_queue_too_full = true;
			return check_and_notify_demux_done();
		}

		unk0x30.main_switchUNK = 1;
		unk0x30.sequence_stateUNK = 4;

		au_queue_no_memory = false;
		return check_and_notify_demux_done();
	}
	default:
		fmt::throw_exception("Unreachable");
	}
}

bool DmuxPamfSPUContext::get_next_cmd(DmuxPamfCommand& lhs, bool new_stream)
{
	cellDmuxPamf.trace("Getting next command");

	if (cmd_queue->try_pop(lhs))
	{
		return true;
	}

	if (new_stream || !demux_done || !demux_done_was_notified)
	{
		if (!au_queue_no_memory && !event_queue_too_full)
		{
			cellDmuxPamf.trace("No new command, continuing work");
			return false;
		}
	}

	cellDmuxPamf.trace("No new command or work to do, waiting...");

	while (thread_ctrl::state() != thread_state::aborting && !cmd_queue->try_pop_for<static_cast<atomic_wait_timeout>(20'000'000)>(lhs));

	if (thread_ctrl::state() == thread_state::aborting)
	{
		return false;
	}

	return true;
}

bool DmuxPamfSPUContext::send_au_done(notify_au_done_params& params)
{
	if (!send_event(DmuxPamfEventType::au_found, params.stream_id, params.private_stream_id, params.au_addr.addr(), std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(params.pts)),
			std::bit_cast<CellCodecTimeStamp>(static_cast<be_t<u64>>(params.dts)), params.unk, params.au_size, params.au_specific_info_size, std::bit_cast<std::array<u8, sizeof(v128)>>(params.stream_header_buf), params.es_id, +params.is_rap))
	{
		event_queue_too_full = true;
		return false;
	}

	return true;
}

bool DmuxPamfSPUContext::send_event(auto&&... args)
{
	if (event_queue->size() >= max_enqueued_events)
	{
		return false;
	}

	return event_queue->emplace(std::forward<decltype(args)>(args)..., +event_queue_was_too_full);
}

void DmuxPamfSPUContext::operator()() // cellSpursMain()
{
	while (!Emu.IsRunning())
	{
		thread_ctrl::wait_for(5'000);
	}

	DmuxPamfCommand cmd;

	for (; thread_ctrl::state() != thread_state::aborting;)
	{
		if (get_next_cmd(cmd, new_stream))
		{
			cellDmuxPamf.trace("Received command %d", static_cast<u32>(cmd.type.get()));

			event_queue_was_too_full = event_queue_too_full;
			event_queue_too_full = false;

			switch (cmd.type)
			{
			case DmuxPamfCmdType::enable_es:
			{
				max_enqueued_events += 2;
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::enable_es) + 1);
				enable_es(static_cast<u8>(cmd.enable_es.stream_id), static_cast<u8>(cmd.enable_es.private_stream_id), static_cast<b8>(cmd.enable_es.is_avc), cmd.enable_es.au_queue_buffer_size, cmd.enable_es.au_queue_buffer,
					cmd.enable_es.au_max_size, static_cast<b8>(cmd.enable_es.is_raw_es), cmd.enable_es.es_id);
				break;
			}
			case DmuxPamfCmdType::disable_es:
			{
				disable_es(static_cast<u8>(cmd.disable_flush_es.stream_id), static_cast<u8>(cmd.disable_flush_es.private_stream_id));
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::disable_es) + 1);
				max_enqueued_events -= 2;
				break;
			}
			case DmuxPamfCmdType::set_stream:
			{
				new_stream = true;
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::set_stream) + 1);
				break;
			}
			case DmuxPamfCmdType::free_memory:
			{
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::free_memory) + 1);
				free_memory(cmd.free_memory.mem_size, static_cast<u8>(cmd.free_memory.stream_id), static_cast<u8>(cmd.free_memory.private_stream_id));
				break;
			}
			case DmuxPamfCmdType::flush_es:
			{
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::flush_es) + 1);
				flush_es(cmd.disable_flush_es.stream_id, cmd.disable_flush_es.private_stream_id);
				break;
			}
			case DmuxPamfCmdType::close:
			{
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::close) + 1);
				while (!send_event(DmuxPamfEventType::close));
				return;
			}
			case DmuxPamfCmdType::reset_stream:
			{
				reset_stream();
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::reset_stream) + 1);
				break;
			}
			case DmuxPamfCmdType::reset_es:
			{
				reset_es(cmd.reset_es.stream_id, cmd.reset_es.private_stream_id, vm::ptr<u8, u64>::make(cmd.reset_es.au_addr));
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::reset_es) + 1);
				break;
			}
			case DmuxPamfCmdType::resume:
			{
				cmd_result_queue->emplace(static_cast<u32>(DmuxPamfCmdType::resume) + 1);
				break;
			}
			default:
			{
				cmd_result_queue->emplace(1000);
			}
			}
		}

		if (thread_ctrl::state() == thread_state::aborting)
		{
			return;
		}

		if (new_stream && demux_done && demux_done_was_notified)
		{
			new_stream = false;

			DmuxPamfStreamInfo stream_info;
			stream_info_queue->pop(stream_info);

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

void DmuxPamfContext::wait_au_released_or_stream_reset(ppu_thread& ppu, AuBitset au_queue_full)
{
	std::lock_guard lock{mutex};

	if (au_queue_full.any())
	{
		cellDmuxPamf.trace("Waiting for AU to be released...");

		while (!ppu.is_stopped() && (au_queue_full & au_released.get()).none() && !this->stream_reset_requested)
		{
			lv2_obj::sleep(ppu);
			cond.wait(mutex, 20'000);
		}

		if (ppu.is_stopped())
		{
			return;
		}

		cellDmuxPamf.trace("AU released");
	}

	stream_reset_requested_save = this->stream_reset_requested;
	this->stream_reset_requested = false;

	au_released = au_released.get().reset();
}

template <bool state>
void DmuxPamfContext::set_au_skip()
{
	std::lock_guard lock{mutex};

	for (vm::bptr<DmuxPamfElementaryStream> es : elementary_streams)
	{
		if (es)
		{
			es->skip_next_au = state;
		}
	}
}
// TODO ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: BLOCK SAVESTATES DURING CALLBACKS
void DmuxPamfContext::exec(ppu_thread& ppu)
{
	if (ppu.loaded_from_savestate)
	{
		hle_spu_thread = new (spurs_context) DmuxPamfSPUThread;
	}

	DmuxPamfEvent event{};

	for (;; skip_waiting_for_au_released_or_stream_reset = false /* Savestates */)
	{
		if (!skip_waiting_for_au_released_or_stream_reset)
		{
			wait_au_released_or_stream_reset(ppu, au_queue_full);
		}

		if (ppu.state & cpu_flag::again)
		{
			return;
		}

		skip_waiting_for_au_released_or_stream_reset = true; // Savestates

		cellDmuxPamf.trace("Waiting for next event...");

		do lv2_obj::sleep(ppu);
		while (!ppu.is_stopped() && !event_queue.try_peek_for<static_cast<atomic_wait_timeout>(20'000'000)>(event));


		if (ppu.is_stopped())
		{
			ppu.state += cpu_flag::again;
			return;
		}

		cellDmuxPamf.trace("Received event %d", static_cast<u32>(event.type.get()));

		if (event.type == DmuxPamfEventType::demux_done)
		{
			{
				std::lock_guard lock{mutex};

				dmux_status = 1;
			}

			if (!stream_reset_requested_save)
			{
				wait_au_released_or_stream_reset(ppu, au_queue_full);

				if (ppu.state & cpu_flag::again)
				{
					return;
				}
			}
		}

		if (stream_reset_requested_save)
		{
			stream_reset_in_progress = true;
			set_au_skip<true>();
		}

		switch (event.type)
		{
		case DmuxPamfEventType::au_found:
		{
			DmuxPamfElementaryStream* es;
			const vm::var<DmuxPamfAuInfo> au_info;

			std::unique_lock lock{mutex};

			es = find_es(event.au_found.stream_id, event.au_found.private_stream_id);

			if (!es || es->_this.get_ptr() != es || es->es_id != event.au_found.es_id)
			{
				break;
			}

			if (!stream_reset_in_progress)
			{
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
						es->au_specific_info[1] = static_cast<u8>(static_cast<u8>(event.au_found.stream_header_buf[0]) & 0xf);
						es->au_specific_info[2] = event.au_found.stream_header_buf[1] >> 6;
					}
				}

				lock.unlock();

				// Block savestate creation during callbacks
				std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

				if (!savestate_lock.owns_lock())
				{
					ppu.state += cpu_flag::again;
					return;
				}

				if (es->notify_au_found.cbFunc(ppu, es->_this, au_info, es->notify_au_found.cbArg) != CELL_OK)
				{
					au_queue_full.set(es->this_index);
					continue;
				}
			}
			else if (es->skip_next_au)
			{
				// Block savestate creation during callbacks
				std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

				if (!savestate_lock.owns_lock())
				{
					ppu.state += cpu_flag::again;
					return;
				}

				es->skip_next_au = false;

				if (be_t<u32> result;
					!cmd_queue.try_emplace(DmuxPamfCmdType::reset_es, event.au_found.stream_id, event.au_found.private_stream_id, event.au_found.au_addr)
					|| !cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::reset_es) + 1)
				{
					lock.unlock();
					event_queue.pop();
					notify_fatal_err.cbFunc(ppu, _this, CELL_OK, notify_fatal_err.cbArg);
					continue;
				}
			}

			break;
		}
		case DmuxPamfEventType::demux_done:
		{
			if (stream_reset_in_progress)
			{
				stream_reset_in_progress = false;
				set_au_skip<false>();
			}

			notify_demux_done.cbFunc(ppu, _this, CELL_OK, notify_demux_done.cbArg);

			std::lock_guard lock{mutex};

			if (dmux_status == 1)
			{
				dmux_status = 0;
				cond.notify_all();
			}

			break;
		}
		case DmuxPamfEventType::close:
		{
			while (event_queue.try_pop());
			return;
		}
		case DmuxPamfEventType::flush_done:
		{
			std::unique_lock lock{mutex};

			const auto es = find_es(event.flush_done.stream_id, event.flush_done.private_stream_id);

			if (es && es->_this.get_ptr() == es && es->es_id == event.flush_done.es_id)
			{
				lock.unlock();
				es->notify_flush_done.cbFunc(ppu, es->_this, es->notify_flush_done.cbArg);
			}

			break;
		}
		case DmuxPamfEventType::prog_end_code:
		{
			notify_prog_end_code.cbFunc(ppu, _this, notify_prog_end_code.cbArg);
			break;
		}
		case DmuxPamfEventType::fatal_error:
		{
			event_queue.pop();
			notify_fatal_err.cbFunc(ppu, _this, CELL_OK, notify_fatal_err.cbArg); // LLE uses CELL_OK
			continue;
		}
		default:
			fmt::throw_exception("Invalid event");
		}

		event_queue.try_pop();

		if (enabled_es_num >= 0 && event_queue.size() == 2)
		{
			std::unique_lock lock{mutex};

			if (enabled_es_num >= 0)
			{
				if (!cmd_queue.try_emplace(DmuxPamfCmdType::resume))
				{
					lock.unlock();
					notify_fatal_err.cbFunc(ppu, _this, CELL_OK, notify_fatal_err.cbArg);
					continue;
				}

				cmd_result_queue.pop();
			}
		}

		au_queue_full.reset();
	}
}

void dmuxPamfEntry(ppu_thread& ppu, vm::ptr<DmuxPamfContext> dmux)
{
	dmux->exec(ppu);

	if (ppu.state & cpu_flag::again)
	{
		dmux->destruct_spu_thread();
		ppu.syscall_args[0] = dmux.addr(); // For savestates, save argument
		return;
	}

	ppu_execute<&sys_ppu_thread_exit>(ppu, CELL_OK);
}

u32 dmuxPamfVerifyEsSpecificInfo(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info)
{
	if (!es_specific_info)
	{
		return CELL_OK;
	}

	switch (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first)
	{
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:
		if (is_avc)
		{
			if (u32 level = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoAvc>(es_specific_info)->level;
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
		const u32 sampling_freq = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info)->samplingFreq;
		const u32 nch = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info)->numOfChannels;
		const u32 bps = vm::static_ptr_cast<const CellDmuxPamfEsSpecificInfoLpcm>(es_specific_info)->bitsPerSample;

		if (sampling_freq != CELL_DMUX_PAMF_FS_48K || (nch != 1 && nch != 2 && nch != 6 && nch != 8) || (bps != CELL_DMUX_PAMF_BITS_PER_SAMPLE_16 && bps != CELL_DMUX_PAMF_BITS_PER_SAMPLE_24))
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

u32 dmuxPamfGetAuSpecificInfoSize(u16 stream_id, u16 private_stream_id, bool is_avc, bool is_raw_es)
{
	if (is_raw_es)
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
	case DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO:     return 0x40;
	case DMUX_PAMF_STREAM_TYPE_INDEX_LPCM:      return 0x100;
	case DMUX_PAMF_STREAM_TYPE_INDEX_AC3:       return 0x40;
	case DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX:    return 0x40;
	case DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA: return 0x40;
	default:                                    return 0;
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
			case CELL_DMUX_PAMF_AVC_LEVEL_4P2: return 0x46a870;
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

u32 dmuxPamfGetEsMemSize(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, bool is_raw_es)
{
	return dmuxPamfGetAuSpecificInfoSize(stream_id, private_stream_id, is_avc, is_raw_es) * dmuxPamfGetAuQueueMaxSize(stream_id, private_stream_id)
		+ dmuxPamfGetAuQueueBufferSize(stream_id, private_stream_id, is_avc, es_specific_info) + 0x7f + static_cast<u32>(sizeof(DmuxPamfElementaryStream)) + 0xf;
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

u32 DmuxPamfContext::create_thread(ppu_thread& ppu)
{
	const vm::var<char[]> name = vm::make_str("HLE PAMF demuxer");
	const auto entry = g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEntry));

	if (error_code ret = ppu_execute<&sys_ppu_thread_create>(ppu, _this.ptr(&DmuxPamfContext::thread_id), entry, +_this.addr(), +resource.ppuThreadPriority, +resource.ppuThreadStackSize, SYS_PPU_THREAD_CREATE_JOINABLE, +name); ret != CELL_OK)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	return CELL_OK;
}

u32 DmuxPamfContext::close(ppu_thread& ppu)
{
	if (join_thread(ppu) != CELL_OK)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	hle_spu_thread->~named_thread();

	return CELL_OK;
}

u32 DmuxPamfContext::reset_stream()
{
	std::lock_guard lock{mutex};

	if (dmux_status != 2)
	{
		return CELL_OK;
	}

	if (!cmd_queue.try_emplace(DmuxPamfCmdType::reset_stream))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::reset_stream) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	stream_reset_requested = true;
	cond.notify_one();

	return CELL_OK;
}

u32 DmuxPamfContext::join_thread(ppu_thread& ppu)
{
	{
		std::lock_guard lock{mutex};

		for (s32 i = 0; i < enabled_es_num; i++)
		{
			elementary_streams[i] = vm::null;
		}

		enabled_es_num = -1;

		if (!cmd_queue.try_emplace(DmuxPamfCmdType::close))
		{
			return DMUX_PAMF_INTERNAL_ERROR_BUSY;
		}

		if (be_t<u32> result; !cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::close) + 1)
		{
			return DMUX_PAMF_INTERNAL_ERROR_FATAL;
		}
	}

	if (vm::var<u64> ret; sys_ppu_thread_join(ppu, static_cast<u32>(thread_id), +ret) != CELL_OK)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	return CELL_OK;
}

u32 DmuxPamfContext::set_stream(vm::cptr<void> stream_address, u32 stream_size, b8 discontinuity, u32 user_data, bool is_raw_es)
{
	std::lock_guard lock{mutex};

	this->user_data = user_data;

	if (!stream_info_queue.try_emplace(stream_address, stream_size, user_data, !discontinuity, is_raw_es))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (!cmd_queue.try_emplace(DmuxPamfCmdType::set_stream))
	{
		stream_info_queue.pop();

		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::set_stream) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	dmux_status = 2;

	return CELL_OK;
}

u32 DmuxPamfContext::enable_es(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, vm::ptr<void> mem_addr, u32 mem_size, vm::bptr<DmuxEsNotifyAuFound> notify_au_found, vm::bptr<void> notify_au_found_arg,
	vm::bptr<DmuxEsNotifyFlushDone> notify_flush_done, vm::bptr<void> notify_flush_done_arg, bool is_raw_es, vm::bptr<DmuxPamfElementaryStream>& es)
{
	if (mem_size < dmuxPamfGetEsMemSize(stream_id, private_stream_id, is_avc, es_specific_info, is_raw_es))
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
	}

	const auto stream_type = dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first;

	if (stream_type == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return DMUX_PAMF_INTERNAL_ERROR_UNKNOWN_STREAM;
	}

	if (dmuxPamfVerifyEsSpecificInfo(stream_id, private_stream_id, is_avc, es_specific_info) != CELL_OK)
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
	}

	std::lock_guard lock{mutex};

	this->is_raw_es = is_raw_es;

	if (enabled_es_num == max_enabled_es_num)
	{
		return DMUX_PAMF_INTERNAL_ERROR_NO_MEMORY;
	}

	if (find_es(stream_id, private_stream_id)) // Elementary stream already enabled
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
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
	const be_t<u32> au_specific_info_size = dmuxPamfGetAuSpecificInfoSize(stream_id, private_stream_id, is_avc, is_raw_es);

	if (!cmd_queue.try_emplace(DmuxPamfCmdType::enable_es, stream_id, private_stream_id, is_avc, au_queue_buffer, dmuxPamfGetAuQueueBufferSize(stream_id, private_stream_id, is_avc, es_specific_info), au_max_size, au_specific_info_size, is_raw_es, next_es_id))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::enable_es) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	u32 i = umax;
	while (elementary_streams[++i]); // There is guaranteed to be an empty slot, this was already checked above

	elementary_streams[i] = _es;

	new (_es.get_ptr()) DmuxPamfElementaryStream(_es, mem_size, i, _this, notify_au_found, notify_au_found_arg, notify_flush_done, notify_flush_done_arg, stream_id, private_stream_id, is_avc, au_queue_buffer, au_max_size, au_specific_info_size, next_es_id++);

	enabled_es_num++;
	es = _es;
	return CELL_OK;
}

// TODO
u32 DmuxPamfContext::reset_stream_and_wait_done()
{
	if (reset_stream() != CELL_OK)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	std::lock_guard lock{mutex};

	while (dmux_status != 0)
	{
		cond.wait(mutex); // TODO lv2_obj::sleep() ?????? check for ppu.is_stopped() ??????????
	}

	return CELL_OK;
}

u32 DmuxPamfElementaryStream::free_memory(vm::ptr<void> mem_addr, u32 mem_size)
{
	const auto dmux = demuxer.get_ptr();

	std::lock_guard lock{dmux->mutex};

	if (!dmux->cmd_queue.try_emplace(DmuxPamfCmdType::free_memory, mem_addr.addr(), mem_size, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id)))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !dmux->cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::free_memory) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	dmux->au_released = dmux->au_released.get().set(this_index);
	dmux->cond.notify_one();

	return CELL_OK;
}

u32 DmuxPamfElementaryStream::disable_es()
{
	const auto dmux = demuxer.get_ptr();

	std::lock_guard lock{dmux->mutex};

	if (!dmux->find_es(stream_id, private_stream_id))
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
	}

	if (!dmux->cmd_queue.try_emplace(DmuxPamfCmdType::disable_es, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id)))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	const u32 _this_index = this_index;

	this->~DmuxPamfElementaryStream();

	dmux->elementary_streams[_this_index] = vm::null;
	dmux->enabled_es_num--;

	if (be_t<u32> result; !dmux->cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::disable_es) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	dmux->au_released = dmux->au_released.get().set(_this_index);
	dmux->cond.notify_one();

	return CELL_OK;
}

u32 DmuxPamfElementaryStream::flush_es()
{
	const auto dmux = demuxer.get_ptr();

	std::lock_guard lock{dmux->mutex};

	if (!dmux->cmd_queue.try_emplace(DmuxPamfCmdType::flush_es, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id)))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !dmux->cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::flush_es) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	return CELL_OK;
}

u32 DmuxPamfElementaryStream::reset_es()
{
	const auto dmux = demuxer.get_ptr();

	std::lock_guard lock{dmux->mutex};

	if (!dmux->cmd_queue.try_emplace(DmuxPamfCmdType::reset_es, static_cast<be_t<u32>>(stream_id), static_cast<be_t<u32>>(private_stream_id), 0))
	{
		return DMUX_PAMF_INTERNAL_ERROR_BUSY;
	}

	if (be_t<u32> result; !dmux->cmd_result_queue.pop(result) || result != static_cast<u32>(DmuxPamfCmdType::reset_es) + 1)
	{
		return DMUX_PAMF_INTERNAL_ERROR_FATAL;
	}

	return CELL_OK;
}

error_code dmuxPamfGetError(u32 internal_error)
{
	switch (internal_error)
	{
	case CELL_OK:                                 return CELL_OK;
	case DMUX_PAMF_INTERNAL_ERROR_ARG:            return CELL_DMUX_PAMF_ERROR_ARG;
	case DMUX_PAMF_INTERNAL_ERROR_NO_MEMORY:      return CELL_DMUX_PAMF_ERROR_NO_MEMORY;
	case DMUX_PAMF_INTERNAL_ERROR_UNKNOWN_STREAM: return CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM;
	case DMUX_PAMF_INTERNAL_ERROR_BUSY:           return CELL_DMUX_PAMF_ERROR_BUSY;
	default:                                      return CELL_DMUX_PAMF_ERROR_FATAL;
	}
}

error_code dmuxPamfNotifyDemuxDone(ppu_thread& ppu, vm::ptr<void> /*core_handle*/, u32 error, vm::ptr<CellDmuxPamfHandle> handle)
{
	handle->notify_demux_done.cbFunc(ppu, handle, dmuxPamfGetError(error), handle->notify_demux_done.cbArg);
	return CELL_OK;
}

error_code dmuxPamfNotifyProgEndCode(ppu_thread& ppu, vm::ptr<void> /*core_handle*/, vm::ptr<CellDmuxPamfHandle> handle)
{
	if (handle->notify_prog_end_code.cbFunc)
	{
		handle->notify_prog_end_code.cbFunc(ppu, handle, handle->notify_prog_end_code.cbArg);
	}

	return CELL_OK;
}

error_code dmuxPamfNotifyFatalErr(ppu_thread& ppu, vm::ptr<void> /*core_handle*/, u32 error, vm::ptr<CellDmuxPamfHandle> handle)
{
	handle->notify_fatal_err.cbFunc(ppu, handle, dmuxPamfGetError(error), handle->notify_fatal_err.cbArg);
	return CELL_OK;
}

error_code dmuxPamfEsNotifyAuFound(ppu_thread& ppu, vm::ptr<void> /*core_handle*/, vm::cptr<DmuxPamfAuInfo> au_info, vm::ptr<CellDmuxPamfEsHandle> handle)
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

error_code dmuxPamfEsNotifyFlushDone(ppu_thread& ppu, vm::ptr<void> /*core_handle*/, vm::ptr<CellDmuxPamfEsHandle> handle)
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

	pamfAttr->maxEnabledEsNum = 0x40;
	pamfAttr->version = DMUX_PAMF_VERSION;
	pamfAttr->memSize = sizeof(CellDmuxPamfHandle) + sizeof(DmuxPamfContext) + 0xe7b;

	return CELL_OK;
}

u32 dmuxPamfOpen(ppu_thread& ppu, const CellDmuxPamfResource& res, vm::cptr<CellDmuxResourceSpurs> res_spurs, vm::bptr<DmuxNotifyDemuxDone> notify_demux_done, vm::bptr<void> notify_dmux_done_arg, vm::bptr<DmuxNotifyProgEndCode> notify_prog_end_code,
	vm::bptr<void> notify_prog_end_code_arg, vm::bptr<DmuxNotifyFatalErr> notify_fatal_err, vm::bptr<void> notify_fatal_err_arg, vm::bptr<DmuxPamfContext>& handle)
{
	if (res.ppuThreadPriority >= 0xc00 || res.ppuThreadStackSize < 0x1000 || res.spuThreadPriority >= 0x100 || res.numOfSpus != 1 || !res.memAddr || res.memSize < sizeof(DmuxPamfContext) + 0xe7b)
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
	}

	const auto _handle = vm::ptr<DmuxPamfContext>::make(utils::align(+res.memAddr.addr(), 0x80));

	vm::bptr<void> spurs_addr = vm::bptr<void>::make(_handle.addr());
	b8 use_existing_spurs = false;

	if (res_spurs)
	{
		spurs_addr = res_spurs->spurs;
		use_existing_spurs = true;
	}

	const auto context = new (_handle.get_ptr()) DmuxPamfContext(spurs_addr, use_existing_spurs, _handle, notify_demux_done, notify_dmux_done_arg, notify_prog_end_code, notify_prog_end_code_arg, notify_fatal_err, notify_fatal_err_arg, res);

	context->construct_spu_thread();

	handle = _handle;
	return context->create_thread(ppu);
}

error_code _CellDmuxCoreOpOpen(ppu_thread& ppu, vm::cptr<CellDmuxPamfSpecificInfo> pamfSpecificInfo, vm::cptr<CellDmuxResource> demuxerResource, vm::cptr<CellDmuxResourceSpurs> demuxerResourceSpurs, vm::cptr<DmuxCb<DmuxNotifyDemuxDone>> notifyDemuxDone,
	vm::cptr<DmuxCb<DmuxNotifyProgEndCode>> notifyProgEndCode, vm::cptr<DmuxCb<DmuxNotifyFatalErr>> notifyFatalErr, vm::pptr<CellDmuxPamfHandle> handle)
{
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

	vm::ptr<CellDmuxPamfHandle> _handle = vm::static_ptr_cast<CellDmuxPamfHandle>(demuxerResource->memAddr);

	const CellDmuxPamfResource res { demuxerResource->ppuThreadPriority, demuxerResource->ppuThreadStackSize, demuxerResource->numOfSpus, demuxerResource->spuThreadPriority,
		vm::bptr<void>::make(demuxerResource->memAddr.addr() + sizeof(CellDmuxPamfHandle)), demuxerResource->memSize - sizeof(CellDmuxPamfHandle) };

	_handle->notify_demux_done = *notifyDemuxDone;
	_handle->notify_fatal_err = *notifyFatalErr;

	if (pamfSpecificInfo && pamfSpecificInfo->programEndCodeCb)
	{
		_handle->notify_prog_end_code = *notifyProgEndCode;
	}
	else
	{
		_handle->notify_prog_end_code = { vm::null, notifyProgEndCode->cbArg };
	}

	const auto demux_done_func = vm::bptr<DmuxNotifyDemuxDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyDemuxDone)));
	const auto prog_end_code_func = vm::bptr<DmuxNotifyProgEndCode>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyProgEndCode)));
	const auto fatal_err_func = vm::bptr<DmuxNotifyFatalErr>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfNotifyFatalErr)));

	const u32 error = dmuxPamfOpen(ppu, res, demuxerResourceSpurs, demux_done_func, _handle, prog_end_code_func, _handle, fatal_err_func, _handle, _handle->demuxer);

	*handle = _handle;

	return dmuxPamfGetError(error);
}

error_code _CellDmuxCoreOpClose(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
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

	return dmuxPamfGetError(handle->demuxer->close(ppu));
}

error_code _CellDmuxCoreOpResetStream(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpResetStream(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return dmuxPamfGetError(handle->demuxer->reset_stream());
}

error_code _CellDmuxCoreOpCreateThread(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
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

	return dmuxPamfGetError(handle->demuxer->create_thread(ppu));
}

error_code _CellDmuxCoreOpJoinThread(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
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

	return dmuxPamfGetError(handle->demuxer->join_thread(ppu));
}

template <bool raw_es>
error_code _CellDmuxCoreOpSetStream(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle, vm::cptr<void> streamAddress, u32 streamSize, b8 discontinuity, u64 userData)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpSetStream<raw_es=%d>(handle=*0x%x, streamAddress=*0x%x, streamSize=0x%x, discontinuity=%d, userData=0x%llx)", raw_es, handle, streamAddress, streamSize, +discontinuity, userData);

	if (!streamAddress || streamSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!handle); // Not checked on LLE

	return dmuxPamfGetError(handle->demuxer->set_stream(streamAddress, streamSize, discontinuity, static_cast<u32>(userData), raw_es));
}

error_code _CellDmuxCoreOpFreeMemory(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle, vm::ptr<void> memAddr, u32 memSize)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.trace("_CellDmuxCoreOpFreeMemory(esHandle=*0x%x, memAddr=*0x%x, memSize=0x%x)", esHandle, memAddr, memSize);

	if (!memAddr || memSize == 0)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!esHandle);

	return dmuxPamfGetError(esHandle->es->free_memory(memAddr, memSize));
}

u32 dmuxPamfGetEsAttr(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, bool is_raw_es, CellDmuxPamfEsAttr& attr)
{
	if (dmuxPamfStreamIdToTypeChannel(stream_id, private_stream_id).first == DMUX_PAMF_STREAM_TYPE_INDEX_INVALID)
	{
		return DMUX_PAMF_INTERNAL_ERROR_UNKNOWN_STREAM;
	}

	if (dmuxPamfVerifyEsSpecificInfo(stream_id, private_stream_id, is_avc, es_specific_info) != 0)
	{
		return DMUX_PAMF_INTERNAL_ERROR_ARG;
	}

	attr.auQueueMaxSize = dmuxPamfGetAuQueueMaxSize(stream_id, private_stream_id);
	attr.memSize = dmuxPamfGetEsMemSize(stream_id, private_stream_id, is_avc, es_specific_info, is_raw_es);
	attr.specificInfoSize = dmuxPamfGetAuSpecificInfoSize(stream_id, private_stream_id, is_avc, is_raw_es);

	return CELL_OK;
}

template <bool raw_es>
error_code _CellDmuxCoreOpQueryEsAttr(vm::cptr<void> esFilterId, vm::cptr<void> esSpecificInfo, vm::ptr<CellDmuxPamfEsAttr> attr)
{
	cellDmuxPamf.notice("_CellDmuxCoreOpQueryEsAttr<raw_es=%d>(esFilterId=*0x%x, esSpecificInfo=*0x%x, attr=*0x%x)", raw_es, esFilterId, esSpecificInfo, attr);

	if (!esFilterId || !attr)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	const auto [stream_id, private_stream_id, is_avc] = [&]() -> std::tuple<u16, u16, bool>
	{
		if constexpr (raw_es)
		{
			const auto filter_id = vm::static_ptr_cast<const u8>(esFilterId);
			return { filter_id[2], filter_id[3], filter_id[8] >> 7 };
		}

		const auto filter_id = vm::static_ptr_cast<const CellCodecEsFilterId>(esFilterId);
		return { filter_id->filterIdMajor, filter_id->filterIdMinor, filter_id->supplementalInfo1 };
	}();

	CellDmuxPamfEsAttr es_attr;

	const error_code ret = dmuxPamfGetError(dmuxPamfGetEsAttr(stream_id, private_stream_id, is_avc, esSpecificInfo, raw_es, es_attr));

	*attr = es_attr;
	attr->memSize += static_cast<u32>(sizeof(CellDmuxPamfEsHandle));

	return ret;
}

template <bool raw_es>
error_code _CellDmuxCoreOpEnableEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle, vm::cptr<void> esFilterId, vm::cptr<CellDmuxEsResource> esResource, vm::cptr<DmuxCb<DmuxEsNotifyAuFound>> notifyAuFound,
	vm::cptr<DmuxCb<DmuxEsNotifyFlushDone>> notifyFlushDone, vm::cptr<void> esSpecificInfo, vm::pptr<CellDmuxPamfEsHandle> esHandle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpEnableEs<raw_es=%d>(handle=*0x%x, esFilterId=*0x%x, esResource=*0x%x, notifyAuFound=*0x%x, notifyFlushDone=*0x%x, esSpecificInfo=*0x%x, esHandle)",
		raw_es, handle, esFilterId, esResource, notifyAuFound, notifyFlushDone, esSpecificInfo, esHandle);

	if (!handle || !esFilterId || !esResource || !esResource->memAddr || esResource->memSize == 0 || !notifyAuFound || !notifyAuFound->cbFunc || !notifyAuFound->cbArg || !notifyFlushDone || !notifyFlushDone->cbFunc || !notifyFlushDone->cbArg)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	ensure(!!esHandle && esResource->memAddr.aligned(0x10)); // Not checked on LLE

	const auto es_handle = vm::static_ptr_cast<CellDmuxPamfEsHandle>(esResource->memAddr);

	es_handle->notify_au_found = *notifyAuFound;
	es_handle->notify_flush_done = *notifyFlushDone;

	const auto au_found_func = vm::bptr<DmuxEsNotifyAuFound>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEsNotifyAuFound)));
	const auto flush_done_func = vm::bptr<DmuxEsNotifyFlushDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxPamfEsNotifyFlushDone)));

	const auto [stream_id, private_stream_id, is_avc] = [&]() -> std::tuple<u16, u16, bool>
	{
		if constexpr (raw_es)
		{
			const auto filter_id = vm::static_ptr_cast<const u8>(esFilterId);
			return { filter_id[2], filter_id[3], filter_id[8] >> 7 };
		}

		const auto filter_id = vm::static_ptr_cast<const CellCodecEsFilterId>(esFilterId);
		return { filter_id->filterIdMajor, filter_id->filterIdMinor, filter_id->supplementalInfo1 };
	}();

	const u32 error = handle->demuxer->enable_es(stream_id, private_stream_id, is_avc, esSpecificInfo, vm::ptr<void>::make(esResource->memAddr.addr() + sizeof(CellDmuxPamfEsHandle)),
		esResource->memSize - sizeof(CellDmuxPamfEsHandle), au_found_func, es_handle, flush_done_func, es_handle, raw_es, es_handle->es);

	*esHandle = es_handle;

	return dmuxPamfGetError(error);
}

error_code _CellDmuxCoreOpDisableEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpDisableEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return dmuxPamfGetError(esHandle->es->disable_es());
}

error_code _CellDmuxCoreOpFlushEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpFlushEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return dmuxPamfGetError(esHandle->es->flush_es());
}

error_code _CellDmuxCoreOpResetEs(ppu_thread& ppu, vm::ptr<CellDmuxPamfEsHandle> esHandle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpResetEs(esHandle=*0x%x)", esHandle);

	if (!esHandle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return dmuxPamfGetError(esHandle->es->reset_es());
}

error_code _CellDmuxCoreOpResetStreamAndWaitDone(ppu_thread& ppu, vm::ptr<CellDmuxPamfHandle> handle)
{
	std::unique_lock savestate_lock{g_fxo->get<hle_locks_t>(), std::try_to_lock};

	if (!savestate_lock.owns_lock())
	{
		ppu.state += cpu_flag::again;
		return {};
	}

	cellDmuxPamf.notice("_CellDmuxCoreOpResetStreamAndWaitDone(handle=*0x%x)", handle);

	if (!handle)
	{
		return CELL_DMUX_PAMF_ERROR_ARG;
	}

	return dmuxPamfGetError(handle->demuxer->reset_stream_and_wait_done());
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
