#pragma once

#include "Utilities/cond.h"
#include <bitset>


// Common

template <typename T, u32 max_num_of_entries> requires(std::is_trivial_v<T> && max_num_of_entries > 0)
class alignas(0x80) DmuxPamfHLESpursQueue
{
	T* const buffer;

	atomic_t<u32> _size = 0;
	u32 front = 0;
	u32 back = 0;

	template <bool is_peek, bool is_blocking, atomic_wait_timeout timeout = atomic_wait_timeout::inf>
	bool _pop(T* lhs)
	{
		if (_size == 0)
		{
			if constexpr (is_blocking)
			{
				_size.wait(0, timeout);

				if (_size == 0)
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}

		if (lhs)
		{
			*lhs = buffer[front];
		}

		if constexpr (!is_peek)
		{
			buffer[front].~T();

			front = (front + 1) % max_num_of_entries;
			_size--;
			_size.notify_one();
		}

		return true;
	}

	template <bool is_blocking, atomic_wait_timeout timeout = atomic_wait_timeout::inf>
	bool _emplace(auto&&... args)
	{
		if (_size == max_num_of_entries)
		{
			if constexpr (is_blocking)
			{
				_size.wait(max_num_of_entries, timeout);

				if (_size == max_num_of_entries)
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}

		new (&buffer[back]) T(std::forward<decltype(args)>(args)...);

		back = (back + 1) % max_num_of_entries;
		_size++;
		_size.notify_one();

		return true;
	}

public:
	DmuxPamfHLESpursQueue(T (&buffer)[max_num_of_entries]) : buffer(buffer) {}

	bool pop(T& lhs)
	{
		return _pop<false, true>(&lhs);
	}

	bool pop()
	{
		return _pop<false, true>(nullptr);
	}

	bool try_pop(T& lhs)
	{
		return _pop<false, false>(&lhs);
	}

	bool try_pop()
	{
		return _pop<false, false>(nullptr);
	}

	template <atomic_wait_timeout timeout>
	bool try_pop_for(T& lhs)
	{
		return _pop<false, true, timeout>(&lhs);
	}

	template <atomic_wait_timeout timeout>
	bool try_pop_for()
	{
		return _pop<false, true, timeout>(nullptr);
	}

	bool peek(T& lhs) const
	{
		return const_cast<DmuxPamfHLESpursQueue<T, max_num_of_entries>*>(this)->template _pop<true, true>(&lhs);
	}

	template <atomic_wait_timeout timeout>
	bool try_peek_for(T& lhs) const
	{
		return const_cast<DmuxPamfHLESpursQueue<T, max_num_of_entries>*>(this)->template _pop<true, true, timeout>(&lhs);
	}

	bool emplace(auto&&... args)
	{
		return _emplace<true>(std::forward<decltype(args)>(args)...);
	}

	bool try_emplace(auto&&... args)
	{
		return _emplace<false>(std::forward<decltype(args)>(args)...);
	}

	template <atomic_wait_timeout timeout>
	bool try_emplace_for(auto&&... args)
	{
		return _emplace<true, timeout>(std::forward<decltype(args)>(args)...);
	}

	u32 size() const
	{
		return this->_size.observe();
	}
};

enum class DmuxPamfCmdType : u32
{
	enable_es = 0,
	disable_es = 2,
	set_stream = 4,
	free_memory = 6,
	flush_es = 8,
	close = 10,
	reset_stream = 12,
	reset_es = 14,
	resume = 16,
};

struct alignas(0x80) DmuxPamfCommand
{
	be_t<DmuxPamfCmdType> type;

	union
	{
		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			be_t<u32> is_avc;
			vm::bptr<u8, u64> au_queue_buffer;
			be_t<u32> au_queue_buffer_size;
			be_t<u32> au_max_size;
			be_t<u32> au_specific_info_size;
			be_t<u32> is_raw_es;
			be_t<u32> es_id;
		}
		enable_es;

		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
		}
		disable_flush_es;

		struct
		{
			be_t<u64, 4> mem_addr;
			be_t<u32> mem_size;
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
		}
		free_memory;

		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			be_t<u64, 4> au_addr;
		}
		reset_es;

		struct
		{
		}
		other;
	};

	DmuxPamfCommand() = default; // Leave queue buffer uninitialized when opening the demuxer

	DmuxPamfCommand(be_t<DmuxPamfCmdType>&& type)
		: type(type), other{}
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCmdType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id)
		: type(type), disable_flush_es{ stream_id, private_stream_id }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCmdType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u64, 4>& unk)
		: type(type), reset_es{ stream_id, private_stream_id, unk }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCmdType>&& type, const be_t<u64, 4>& mem_addr, const be_t<u32>& mem_size, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id)
		: type(type), free_memory{ mem_addr, mem_size, stream_id, private_stream_id }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCmdType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u32>& is_avc, const vm::bptr<u8, u64>& au_queue_buffer,
		const be_t<u32>& au_queue_buffer_size, const be_t<u32>& au_max_size, const be_t<u32>& au_specific_info_size, const be_t<u32>& is_raw_es, const be_t<u32>& es_id)
		: type(type), enable_es{ stream_id, private_stream_id, is_avc, au_queue_buffer, au_queue_buffer_size, au_max_size, au_specific_info_size, is_raw_es, es_id }
	{
	}
};

CHECK_SIZE_ALIGN(DmuxPamfCommand, 0x80, 0x80);

enum class DmuxPamfEventType : u32
{
	au_found,
	demux_done,
	fatal_error,
	close,
	flush_done,
	prog_end_code,
};

struct alignas(0x80) DmuxPamfEvent
{
	be_t<DmuxPamfEventType> type;

	union
	{
		u8 pad[0x78];

		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			be_t<u64, 4> au_addr;
			CellCodecTimeStamp pts;
			CellCodecTimeStamp dts;
			be_t<u64, 4> unk;
			u8 reserved[4];
			be_t<u32> au_size;
			be_t<u32> stream_header_size;
			std::array<u8, sizeof(v128)> stream_header_buf;
			be_t<u32> es_id;
			be_t<u32> is_rap;
		}
		au_found;

		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			be_t<u32> es_id;
		}
		flush_done;

		struct
		{
		}
		other;
	};

	be_t<u32> event_queue_was_too_full;

	DmuxPamfEvent() = default; // Leave queue buffer uninitialized when opening the demuxer

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& event_queue_was_too_full)
		: type(type), other{}, event_queue_was_too_full(event_queue_was_too_full)
	{
	}

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u32>& es_id, const be_t<u32>& event_queue_was_too_full)
		: type(type), flush_done{ stream_id, private_stream_id, es_id }, event_queue_was_too_full(event_queue_was_too_full)
	{
	}

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u64>& au_addr, const CellCodecTimeStamp& pts, const CellCodecTimeStamp& dts, const be_t<u64>& unk1,
		const be_t<u32>& au_size, const be_t<u32>& au_specific_info_size, const std::array<u8, sizeof(v128)>& au_specific_info, const be_t<u32>& es_id, const be_t<u32>& is_rap, const be_t<u32>& event_queue_was_too_full)
		: type(type), au_found{ stream_id, private_stream_id, static_cast<be_t<u64, 4>>(au_addr), pts, dts, static_cast<be_t<u64, 4>>(unk1), {}, au_size, au_specific_info_size, au_specific_info, es_id, is_rap }, event_queue_was_too_full(event_queue_was_too_full)
	{
	}
};

CHECK_SIZE_ALIGN(DmuxPamfEvent, 0x80, 0x80);

struct alignas(0x80) DmuxPamfStreamInfo
{
	vm::bcptr<void, u64> stream_addr;
	be_t<u32> stream_size;
	be_t<u32> user_data;
	be_t<u32> continuity;
	be_t<u32> is_raw_es;
};

CHECK_SIZE_ALIGN(DmuxPamfStreamInfo, 0x80, 0x80);

enum DmuxPamfStreamTypeIndex
{
	DMUX_PAMF_STREAM_TYPE_INDEX_INVALID = -1,
	DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO = 0,
	DMUX_PAMF_STREAM_TYPE_INDEX_LPCM,
	DMUX_PAMF_STREAM_TYPE_INDEX_AC3,
	DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX,
	DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA,
};


//----------------------------------------------------------------------------
// SPU
//----------------------------------------------------------------------------

constexpr u16 AC3_SYNCWORD = 0x0b77;
constexpr u16 ATRACX_SYNCWORD = 0x0fd0;

constexpr u16 AC3_FRMSIZE_TABLE[3][38] =
{
	{ 0x40, 0x40, 0x50, 0x50, 0x60, 0x60, 0x70, 0x70, 0x80, 0x80, 0xa0, 0xa0, 0xc0, 0xc0, 0xe0, 0xe0, 0x100, 0x100, 0x140, 0x140, 0x180, 0x180, 0x1c0, 0x1c0, 0x200, 0x200, 0x280, 0x280, 0x300, 0x300, 0x380, 0x380, 0x400, 0x400, 0x480, 0x480, 0x500, 0x500 },
	{ 0x45, 0x46, 0x57, 0x58, 0x68, 0x69, 0x79, 0x7a, 0x8b, 0x8c, 0xae, 0xaf, 0xd0, 0xd1, 0xf3, 0xf4, 0x116, 0x117, 0x15c, 0x15d, 0x1a1, 0x1a2, 0x1e7, 0x1e8, 0x22d, 0x22e, 0x2b8, 0x2b9, 0x343, 0x344, 0x3cf, 0x3d0, 0x45a, 0x45b, 0x4e5, 0x4e6, 0x571, 0x572 },
	{ 0x60, 0x60, 0x78, 0x78, 0x90, 0x90, 0xa8, 0xa8, 0xc0, 0xc0, 0xf0, 0xf0, 0x120, 0x120, 0x150, 0x150, 0x180, 0x180, 0x1e0, 0x1e0, 0x240, 0x240, 0x2a0, 0x2a0, 0x300, 0x300, 0x3c0, 0x3c0, 0x480, 0x480, 0x540, 0x540, 0x600, 0x600, 0x6c0, 0x6c0, 0x780, 0x780 }
};

constexpr s8 PACK_STUFFING_LENGTH_OFFSET = 0xd;

enum start_codes : u32
{
	M2V_PIC_START       = 0x100,
	AVC_AU_DELIMITER    = 0x109,
	M2V_SEQUENCE_HEADER = 0x1b3,
	M2V_SEQUENCE_END    = 0x1b7,
	PACK_START          = 0x1ba,
	SYSTEM_HEADER       = 0x1bb,
	PRIVATE_STREAM_1    = 0x1bd,
	PRIVATE_STREAM_2    = 0x1bf,
	PROG_END            = 0x1b9,
	VIDEO_STREAM        = 0x1e0,
};

struct notify_au_done_params
{
	u8 stream_id;
	u8 private_stream_id;
	u32 au_size;
	vm::ptr<u8, u64> au_addr;
	u64 pts;
	u64 dts;
	u32 au_specific_info_size;
	u64 unk;
	v128 stream_header_buf;
	u32 es_id;
	b8 is_rap;
};

struct es_0xc0
{
	u32 au_cut_status; // 0 = not cutting out an au, 1 = started cutting, 3 = finished cutting, 5 = in the middle of cutting

	//u8 unk_0x04; // Unused

	b8 first_au_delimiter_found;

	u32 current_au_size;

	b8 is_rap;

	u64 pts;
	u64 dts;

	u32 stream_header_size;
	v128 stream_header_buf;

	//u32 unk10; // Unused

	v128 prev_packet_cache;
	u32 cache_start_idx;

	u32 parsed_au_size;
	u32 au_info_offset;
	//u32 ac3_sync_info; // Unused

	void reset()
	{
		au_cut_status = 0;
		first_au_delimiter_found = false;
		current_au_size = 0;
		is_rap = false;
		pts = umax;
		dts = umax;
		stream_header_size = 0;
		stream_header_buf = {};
		parsed_au_size = 0;
		au_info_offset = 0;
	}
};

struct es_0x10
{
	const u8* addr;
	u32 au_size;
	u32 processed_size;

	u32 prev_bytes_size;
	v128 prev_bytes;

	s32 remaining_bytes_to_parse;
	// v128 unk_0x30; // Unused

	void reset()
	{
		addr = nullptr;
		au_size = 0;
		processed_size = 0;
		prev_bytes_size = 0;
		prev_bytes = {};
		remaining_bytes_to_parse = 0;
	}
};

struct au_queue
{
	// v128 buf; // LLE stores the last (pos & alignof(v128) - 1) bytes due to SPU shenanigans, we don't need that

	vm::ptr<u8, u64> addr;

	u32 freed_mem_size;
	u32 pos;
	u32 end_pos;

	u32 size;
	u32 au_max_size;

	void push(es_0x10& unk2);

	void init(vm::ptr<u8, u64> addr, u32 size, u32 au_max_size)
	{
		this->addr = addr;
		this->freed_mem_size = 0;
		this->pos = 0;
		this->end_pos = 0;
		this->size = size;
		this->au_max_size = au_max_size;
	}
};

struct demux_es
{
	b8 is_enabled;

	u32 _switch;
	// u32 unk0x08; // Unused

	es_0x10 unk0x10;

	u32 parse_stream_result;

	u64 pts;
	u64 dts;

	b8 is_rap;

	u32 stream_header_size;

	v128 stream_header_buf;

	u32 (demux_es::*parse_stream)(const u8* input_addr, u32 input_size, es_0x10& unk4);

	au_queue _au_queue;

	es_0xc0 unk0xc0;

	notify_au_done_params au_done_params;

	u32 au_found_num;

	//u32 m2v_sequence_header_code_found_num; // Unused

	b8 start_of_au;

	u32 sizeUNK; // contains start of new au ???

	b8 is_avc;

	void reset()
	{
		_switch = 0;
		parse_stream_result = 0;
		is_rap = false;
		stream_header_size = 0;
		stream_header_buf = {};
	}

	void reset_timestamps()
	{
		pts = umax;
		dts = umax;
	}

	template <bool is_avc>
	u32 parse_video(const u8* input_addr, u32 input_size, es_0x10& unk4);
	u32 parse_lpcm(const u8* input_addr, u32 input_size, es_0x10& unk4);
	template <bool is_ac3>
	u32 parse_audio(const u8* input_addr, u32 input_size, es_0x10& unk4);
	u32 parse_user_data(const u8* input_addr, u32 input_size, es_0x10& unk4);
};

struct alignas(0x10) demux_pack_finder
{
	u32 found_code; // 0 = none, 1 = pack start code, 6 = program end code
	const u8* search_start_addr;
	u32 processed_bytes;
	u32 matched_bytes;

	void reset()
	{
		found_code = 0;
		search_start_addr = nullptr;
		processed_bytes = 0;
		matched_bytes = 0;
	}
};

struct demux_0x30
{
	u32 main_switchUNK;
	// u32 unk0x04; // Error code? Unused

	// v128 invalid_bytes; // Unused
	v128 PES_header_buf;

	// u32 input_addr_low; // Unused

	const u8* input_addr;

	const u8* unk0x38; // input addr ???

	const u8* end_of_pes_packet;

	s32 unk0x40; // pes_packet_remaining_size ???

	s32 unk0x44; // Unused ???

	s32 pes_packet_remaining_size;
	// u32 pes_header_size; // Unused

	u32 type_idx;
	u32 channel;

	u32 sequence_stateUNK;

	void reset()
	{
		main_switchUNK = 0;
		PES_header_buf = {};
		input_addr = nullptr;
		unk0x44 = 0;
		pes_packet_remaining_size = 0;
		type_idx = 0;
		channel = 0;
		sequence_stateUNK = 4;
	}
};

struct demux_input_stream
{
	s32 bytes_to_process;
	s32 bytes_in_buffer;
	u32 processed_bytes;
	u32 pos;
	u32 size;
	vm::cptr<u8, u64> addr;
	u32 dma_count;

	void init(vm::cptr<void, u64> addr, u32 size)
	{
		bytes_to_process = 0;
		bytes_in_buffer = 0;
		processed_bytes = 0;
		pos = 0;
		this->size = size;
		this->addr = vm::static_ptr_cast<const u8>(addr);
		dma_count = 0;
	}
};

class DmuxPamfSPUContext
{
	// These are globals on LLE
	alignas(0x80) u8 input_buffer[0x4000];
	// alignas(0x80) u8 output_buffer[0x4080]; // We don't need this

	DmuxPamfHLESpursQueue<DmuxPamfCommand, 1>* cmd_queue;
	DmuxPamfHLESpursQueue<be_t<u32>, 1>* cmd_result_queue;
	DmuxPamfHLESpursQueue<DmuxPamfStreamInfo, 1>* stream_info_queue;
	DmuxPamfHLESpursQueue<DmuxPamfEvent, 132>* event_queue;

	b8 au_queue_no_memory;
	b8 event_queue_too_full;
	b8 event_queue_was_too_full;
	u32 max_enqueued_events;

	bool get_next_cmd(DmuxPamfCommand& lhs, bool new_stream);
	bool send_event(auto&&... args);

	// This is a local in cellSpursMain(), needs to be saved for savestates
	bool new_stream;

	// These are part of a class on LLE
	u32 demux_done;
	u32 demux_done_was_notified;

	demux_input_stream input_stream;

	demux_0x30 unk0x30;
	demux_pack_finder unk0x90;

	// u8 unk[0x20]; // Unused

	demux_es elementary_streams[5][0x10];

	b8 is_raw_es;
	DmuxPamfStreamTypeIndex es_type_idx;

	// u32 packs_found_num;                     // Unused
	// u32 pes_packets_found_num;               // Unused
	// u32 invalid_packet_start_code_found_num; // Unused

	bool enable_es(u8 stream_id, u8 private_stream_id, b8 is_avc, u32 au_queue_buffer_size, vm::ptr<u8, u64> au_queue_buffer, u32 au_max_size, b8 is_raw_es, u32 es_id);
	bool disable_es(u8 stream_id, u8 private_stream_id);
	bool free_memory(u32 mem_size, u8 stream_id, u8 private_stream_id);
	bool flush_es(u8 stream_id, u8 private_stream_id);
	void reset_stream();
	bool reset_es(u8 stream_id, u8 private_stream_id, vm::ptr<u8, u64> au_addr);
#pragma message(": warning: TODO: function name")
	void FUN_00006cc8();

	bool send_au_done(notify_au_done_params& params);

	inline bool check_and_notify_demux_done();
	inline bool check_demux_done_was_notified();
	bool demux(const DmuxPamfStreamInfo* stream_inf);

public:
	DmuxPamfSPUContext(){} // For savestates, prevent zero-initialization when default constructing the thread

	DmuxPamfSPUContext(DmuxPamfHLESpursQueue<DmuxPamfCommand, 1>* cmd_queue, DmuxPamfHLESpursQueue<be_t<u32>, 1>* cmd_result_queue, DmuxPamfHLESpursQueue<DmuxPamfStreamInfo, 1>* stream_info_queue, DmuxPamfHLESpursQueue<DmuxPamfEvent, 132>* event_queue)
		: input_buffer(), cmd_queue(cmd_queue), cmd_result_queue(cmd_result_queue), stream_info_queue(stream_info_queue), event_queue(event_queue), au_queue_no_memory(), event_queue_too_full(), event_queue_was_too_full(),
		  max_enqueued_events(4), new_stream(), demux_done(true), demux_done_was_notified(true), input_stream(), unk0x30{ 0, v128(), nullptr, nullptr, nullptr, 0, 0, 0, 0, 0, 4 }, unk0x90(), elementary_streams(), is_raw_es(), es_type_idx()
	{
	}

	void operator()(); // cellSpursMain()
	static constexpr auto thread_name = "PAMF Demuxer Thread"sv;
};

using DmuxPamfSPUThread = named_thread<DmuxPamfSPUContext>;

//----------------------------------------------------------------------------
// PPU
//----------------------------------------------------------------------------

// Error Codes
enum CellDmuxPamfError : u32
{
	CELL_DMUX_PAMF_ERROR_BUSY = 1,
	CELL_DMUX_PAMF_ERROR_ARG = 2,
	CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM = 3,
	CELL_DMUX_PAMF_ERROR_NO_MEMORY = 5,
	CELL_DMUX_PAMF_ERROR_FATAL = 6,
};

enum DmuxPamfInternalError : u32
{
	DMUX_PAMF_INTERNAL_ERROR_ARG = 1,
	DMUX_PAMF_INTERNAL_ERROR_NO_MEMORY = 2,
	DMUX_PAMF_INTERNAL_ERROR_UNKNOWN_STREAM = 3,
	DMUX_PAMF_INTERNAL_ERROR_BUSY = 5,
	DMUX_PAMF_INTERNAL_ERROR_FATAL = 6,
};

enum CellDmuxPamfM2vLevel : s32
{
	CELL_DMUX_PAMF_M2V_MP_LL = 0,
	CELL_DMUX_PAMF_M2V_MP_ML,
	CELL_DMUX_PAMF_M2V_MP_H14,
	CELL_DMUX_PAMF_M2V_MP_HL,
};

enum CellDmuxPamfAvcLevel : s32
{
	CELL_DMUX_PAMF_AVC_LEVEL_2P1 = 21,
	CELL_DMUX_PAMF_AVC_LEVEL_3P0 = 30,
	CELL_DMUX_PAMF_AVC_LEVEL_3P1 = 31,
	CELL_DMUX_PAMF_AVC_LEVEL_3P2 = 32,
	CELL_DMUX_PAMF_AVC_LEVEL_4P1 = 41,
	CELL_DMUX_PAMF_AVC_LEVEL_4P2 = 42,
};

struct CellDmuxPamfAuSpecificInfoM2v
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfAuSpecificInfoAvc
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfAuSpecificInfoLpcm
{
	u8 channelAssignmentInfo;
	u8 samplingFreqInfo;
	u8 bitsPerSample;
};

struct CellDmuxPamfAuSpecificInfoAc3
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfAuSpecificInfoAtrac3plus
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfAuSpecificInfoUserData
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfEsSpecificInfoM2v
{
	be_t<u32> profileLevel;
};

struct CellDmuxPamfEsSpecificInfoAvc
{
	be_t<u32> level;
};

struct CellDmuxPamfEsSpecificInfoLpcm
{
	be_t<u32> samplingFreq;
	be_t<u32> numOfChannels;
	be_t<u32> bitsPerSample;
};

struct CellDmuxPamfEsSpecificInfoAc3
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfEsSpecificInfoAtrac3plus
{
	be_t<u32> reserved1;
};

struct CellDmuxPamfEsSpecificInfoUserData
{
	be_t<u32> reserved1;
};

enum CellDmuxPamfSamplingFrequency : s32
{
	CELL_DMUX_PAMF_FS_48K = 48000,
};

enum CellDmuxPamfBitsPerSample : s32
{
	CELL_DMUX_PAMF_BITS_PER_SAMPLE_16 = 16,
	CELL_DMUX_PAMF_BITS_PER_SAMPLE_24 = 24,
};

enum CellDmuxPamfLpcmChannelAssignmentInfo : s32
{
	CELL_DMUX_PAMF_LPCM_CH_M1 = 1,
	CELL_DMUX_PAMF_LPCM_CH_LR = 3,
	CELL_DMUX_PAMF_LPCM_CH_LRCLSRSLFE = 9,
	CELL_DMUX_PAMF_LPCM_CH_LRCLSCS1CS2RSLFE = 11,
};

enum CellDmuxPamfLpcmFs : s32
{
	CELL_DMUX_PAMF_LPCM_FS_48K = 1,
};

enum CellDmuxPamfLpcmBitsPerSamples : s32
{
	CELL_DMUX_PAMF_LPCM_BITS_PER_SAMPLE_16 = 1,
	CELL_DMUX_PAMF_LPCM_BITS_PER_SAMPLE_24 = 3,
};

struct CellDmuxPamfSpecificInfo
{
	be_t<u32> thisSize;
	b8 programEndCodeCb;
};

struct CellDmuxPamfAttr
{
	be_t<u32> maxEnabledEsNum;
	be_t<u32> version;
	be_t<u32> memSize;
};

struct CellDmuxPamfEsAttr
{
	be_t<u32> auQueueMaxSize;
	be_t<u32> memSize;
	be_t<u32> specificInfoSize;
};

struct CellDmuxPamfResource
{
	be_t<u32> ppuThreadPriority;
	be_t<u32> ppuThreadStackSize;
	be_t<u32> numOfSpus;
	be_t<u32> spuThreadPriority;
	vm::bptr<void> memAddr;
	be_t<u32> memSize;
};

struct DmuxPamfAuInfo
{
	vm::bptr<void> addr;
	be_t<u32> size;
	CellCodecTimeStamp pts;
	CellCodecTimeStamp dts;
	be_t<u64> user_data;
	vm::bptr<void> specific_info;
	be_t<u32> specific_info_size;
	b8 is_rap;
};

CHECK_SIZE(DmuxPamfAuInfo, 0x30);

constexpr u32 DMUX_PAMF_VERSION = 0x280000;

struct DmuxPamfElementaryStream;

class DmuxPamfContext
{
	static constexpr u32 MAX_ENABLED_ES_NUM = 0x40;

	using AuBitset = std::bitset<MAX_ENABLED_ES_NUM>;
	static_assert(sizeof(AuBitset) == sizeof(u64), "std::bitset implementation not suitable");

	friend struct DmuxPamfElementaryStream;


	alignas(0x80) u8 spurs[0x1000]; // CellSpurs
	const vm::bptr<void> spurs_addr;       // CellSpurs*
	const b8 use_existing_spurs;

	alignas(0x80) const u8 spurs_taskset[0x1900]{}; // CellSpursTaskset
	const be_t<u32> spurs_task_id{};                       // CellSpursTaskId
	const vm::bptr<void> spurs_context_addr;

	u8 reserved_1[16]; // Unused

	const vm::bptr<DmuxPamfContext> _this;
	const be_t<u32> this_size;
	const be_t<u32> version = DMUX_PAMF_VERSION;

	const DmuxCb<DmuxNotifyDemuxDone> notify_demux_done;
	const DmuxCb<DmuxNotifyProgEndCode> notify_prog_end_code;
	const DmuxCb<DmuxNotifyFatalErr> notify_fatal_err;

	const CellDmuxPamfResource resource;

	be_t<u64> thread_id; // sys_ppu_thread_t

	const be_t<u32> unk = 0; // Unused

	const be_t<u32> ppu_thread_stack_size;

	be_t<AuBitset> au_released{}; // Each bit corresponds to one elementary stream index. 1 means cellDmuxReleaseAu() was called

	b8 stream_reset_requested = false;

	be_t<u32> dmux_status = 0; // TODOOOOOOOOOO make enum

	const be_t<u32> max_enabled_es_num = MAX_ENABLED_ES_NUM;
	be_t<s32> enabled_es_num = 0;
	vm::bptr<DmuxPamfElementaryStream> elementary_streams[MAX_ENABLED_ES_NUM]{};

	shared_mutex mutex; // sys_mutex_t
	cond_variable cond; // sys_cond_t

	const vm::bptr<DmuxPamfHLESpursQueue<DmuxPamfCommand, 1>> cmd_queue_addr_; // Same as cmd_queue_addr, unused
	const vm::bptr<DmuxPamfCommand[1]> cmd_queue_buffer_addr_;                 // Same as cmd_queue_buffer_addr, unused

	const vm::bptr<DmuxPamfHLESpursQueue<DmuxPamfCommand, 1>> cmd_queue_addr;            // CellSpursQueue*
	const vm::bptr<DmuxPamfHLESpursQueue<be_t<u32>, 1>> cmd_result_queue_addr;           // CellSpursQueue*
	const vm::bptr<DmuxPamfHLESpursQueue<DmuxPamfStreamInfo, 1>> stream_info_queue_addr; // CellSpursQueue*
	const vm::bptr<DmuxPamfHLESpursQueue<DmuxPamfEvent, 132>> event_queue_addr;          // CellSpursQueue*

	const vm::bptr<DmuxPamfCommand[1]> cmd_queue_buffer_addr;
	const vm::bptr<be_t<u32>[1]> cmd_result_queue_buffer_addr;
	const vm::bptr<DmuxPamfEvent[132]> event_queue_buffer_addr;
	const vm::bptr<DmuxPamfStreamInfo[1]> stream_info_queue_buffer_addr;

	const vm::bptr<DmuxPamfHLESpursQueue<DmuxPamfCommand, 1>> cmd_queue_addr__; // Same as cmd_queue_addr, unused

	be_t<u64> user_data;

	b8 is_raw_es;

	be_t<u32> next_es_id = 0;

	const std::array<char, 24> spurs_taskset_name;

	// HLE exclusive
	DmuxPamfSPUThread* hle_spu_thread;
	AuBitset au_queue_full{};
	b8 stream_reset_requested_save = false;
	b8 stream_reset_in_progress = false;
	b8 skip_waiting_for_au_released_or_stream_reset = false;

	u8 reserved_2[909]; // Unused, 928 bytes on LLE

	DmuxPamfHLESpursQueue<DmuxPamfCommand, 1> cmd_queue;            // CellSpursQueue
	DmuxPamfHLESpursQueue<be_t<u32>, 1> cmd_result_queue;           // CellSpursQueue
	DmuxPamfHLESpursQueue<DmuxPamfStreamInfo, 1> stream_info_queue; // CellSpursQueue
	DmuxPamfHLESpursQueue<DmuxPamfEvent, 132> event_queue;          // CellSpursQueue

	DmuxPamfCommand cmd_queue_buffer[1];
	alignas(0x80) be_t<u32> cmd_result_queue_buffer[1];
	DmuxPamfStreamInfo stream_info_queue_buffer[1];
	DmuxPamfEvent event_queue_buffer[132];

	alignas(0x80) u8 spurs_context[0x36400];
	static_assert(sizeof(spurs_context) >= sizeof(DmuxPamfSPUThread)); // We can put the thread here

public:
	DmuxPamfContext(vm::bptr<void> spurs_addr, b8 use_existing_spurs, vm::ptr<DmuxPamfContext> _this, vm::bptr<DmuxNotifyDemuxDone> notify_demux_done, vm::bptr<void> notify_dmux_done_arg, vm::bptr<DmuxNotifyProgEndCode> notify_prog_end_code,
		vm::bptr<void> notify_prog_end_code_arg, vm::bptr<DmuxNotifyFatalErr> notify_fatal_err, vm::bptr<void> notify_fatal_err_arg, const CellDmuxPamfResource& res)
		: spurs_addr(spurs_addr)
		, use_existing_spurs(use_existing_spurs)
		, spurs_context_addr(_this.ptr(&DmuxPamfContext::spurs_context))
		, _this(_this)
		, this_size(res.memSize)
		, notify_demux_done{ notify_demux_done, notify_dmux_done_arg }
		, notify_prog_end_code{ notify_prog_end_code, notify_prog_end_code_arg }
		, notify_fatal_err{ notify_fatal_err, notify_fatal_err_arg }
		, resource(res)
		, ppu_thread_stack_size(res.ppuThreadStackSize)
		, cmd_queue_addr_(_this.ptr(&DmuxPamfContext::cmd_queue))
		, cmd_queue_buffer_addr_(_this.ptr(&DmuxPamfContext::cmd_queue_buffer))
		, cmd_queue_addr(_this.ptr(&DmuxPamfContext::cmd_queue))
		, cmd_result_queue_addr(_this.ptr(&DmuxPamfContext::cmd_result_queue))
		, stream_info_queue_addr(_this.ptr(&DmuxPamfContext::stream_info_queue))
		, event_queue_addr(_this.ptr(&DmuxPamfContext::event_queue))
		, cmd_queue_buffer_addr(_this.ptr(&DmuxPamfContext::cmd_queue_buffer))
		, cmd_result_queue_buffer_addr(_this.ptr(&DmuxPamfContext::cmd_result_queue_buffer))
		, event_queue_buffer_addr(_this.ptr(&DmuxPamfContext::event_queue_buffer))
		, stream_info_queue_buffer_addr(_this.ptr(&DmuxPamfContext::stream_info_queue_buffer))
		, cmd_queue_addr__(_this.ptr(&DmuxPamfContext::cmd_queue))
		, spurs_taskset_name([&]
			{
				std::array<char, 24> ret;
				std::snprintf(ret.data(), sizeof(ret), "_libdmux_pamf_%08x", _this.addr());
				ret[23] = '\0';
				return ret;
			}())
		, cmd_queue(cmd_queue_buffer)
		, cmd_result_queue(cmd_result_queue_buffer)
		, stream_info_queue(stream_info_queue_buffer)
		, event_queue(event_queue_buffer)
	{
		ensure(this == _this.get_ptr());
	}

	void construct_spu_thread() { hle_spu_thread = new (&spurs_context) DmuxPamfSPUThread(&cmd_queue, &cmd_result_queue, &stream_info_queue, &event_queue); }
	void destruct_spu_thread() { hle_spu_thread->~named_thread(); }

	DmuxPamfElementaryStream* find_es(u16 stream_id, u16 private_stream_id);

	u32 create_thread(ppu_thread& ppu);
	u32 close(ppu_thread& ppu);
	u32 reset_stream();
	u32 join_thread(ppu_thread& ppu);
	u32 set_stream(vm::cptr<void> stream_address, u32 stream_size, b8 discontinuity, u32 user_data, bool is_raw_es);
	u32 enable_es(u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, vm::ptr<void> mem_addr, u32 mem_size, vm::bptr<DmuxEsNotifyAuFound> notify_au_found, vm::bptr<void> notify_au_found_arg,
		vm::bptr<DmuxEsNotifyFlushDone> notify_flush_done, vm::bptr<void> notify_flush_done_arg, bool is_raw_es, vm::bptr<DmuxPamfElementaryStream>& es);
	u32 reset_stream_and_wait_done();

	void exec(ppu_thread& ppu);

	void wait_au_released_or_stream_reset(ppu_thread& ppu, AuBitset au_queue_full);

	template <bool state>
	void set_au_skip();
};

static_assert(std::is_standard_layout_v<DmuxPamfContext>);
CHECK_SIZE_ALIGN(DmuxPamfContext, 0x3d880, 0x80);

struct CellDmuxPamfHandle
{
	vm::bptr<DmuxPamfContext> demuxer;

	DmuxCb<DmuxNotifyDemuxDone> notify_demux_done;
	DmuxCb<DmuxNotifyProgEndCode> notify_prog_end_code;
	DmuxCb<DmuxNotifyFatalErr> notify_fatal_err;
};

CHECK_SIZE(CellDmuxPamfHandle, 0x1c);

struct DmuxPamfElementaryStream
{
	vm::bptr<DmuxPamfElementaryStream> _this;
	be_t<u32> this_size;
	u8 this_index;

	vm::bptr<DmuxPamfContext> demuxer;

	DmuxCb<DmuxEsNotifyAuFound> notify_au_found;
	const DmuxCb<DmuxEsNotifyFlushDone> notify_flush_done;

	be_t<u16> stream_id;
	be_t<u16> private_stream_id;
	b8 is_avc;

	vm::bptr<u8> au_queue_buffer;
	be_t<u32> unk; // Likely au_queue_buffer_size, unused
	be_t<u32> au_max_size;
	std::array<u8, 0x10> au_specific_info;
	const be_t<u32> au_specific_info_size;

	b8 skip_next_au = false;

	const be_t<u32> es_id;

	u8 reserved[72];

	DmuxPamfElementaryStream(vm::bptr<DmuxPamfElementaryStream> _this, be_t<u32> this_size, u8 this_index, vm::bptr<DmuxPamfContext> demuxer, vm::bptr<DmuxEsNotifyAuFound> notify_au_found, vm::bptr<void> notify_au_found_arg,
		vm::bptr<DmuxEsNotifyFlushDone> notify_flush_done, vm::bptr<void> notify_flush_done_arg, be_t<u16> stream_id, be_t<u16> private_stream_id, b8 is_avc, vm::bptr<u8> au_queue_buffer, be_t<u32> au_max_size, be_t<u32> au_specific_info_size, be_t<u32> es_id)
		: _this(_this)
		, this_size(this_size)
		, this_index(this_index)
		, demuxer(demuxer)
		, notify_au_found{ notify_au_found, notify_au_found_arg }
		, notify_flush_done{ notify_flush_done, notify_flush_done_arg }
		, stream_id(stream_id)
		, private_stream_id(private_stream_id)
		, is_avc(is_avc)
		, au_queue_buffer(au_queue_buffer)
		, au_max_size(au_max_size)
		, au_specific_info_size(au_specific_info_size)
		, es_id(es_id)
	{
	}

	~DmuxPamfElementaryStream()
	{
		_this = vm::null;
		this_size = 0;
		this_index = 0;
		demuxer = vm::null;
		notify_au_found = {};
		au_queue_buffer = vm::null;
		unk = 0;
	}

	u32 free_memory(vm::ptr<void> mem_addr, u32 mem_size);
	u32 disable_es();
	u32 flush_es();
	u32 reset_es();
};

static_assert(std::is_standard_layout_v<DmuxPamfElementaryStream>);
CHECK_SIZE_ALIGN(DmuxPamfElementaryStream, 0x98, 4);

struct CellDmuxPamfEsHandle
{
	vm::bptr<DmuxPamfElementaryStream> es;

	DmuxCb<DmuxEsNotifyAuFound> notify_au_found;
	DmuxCb<DmuxEsNotifyFlushDone> notify_flush_done;
};

CHECK_SIZE(CellDmuxPamfEsHandle, 0x14);
