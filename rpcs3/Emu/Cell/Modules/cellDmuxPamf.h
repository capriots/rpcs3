#pragma once

#include <ranges>

// Replacement for CellSpursQueue
template <typename T, u32 max_num_of_entries> requires(std::is_trivial_v<T> && max_num_of_entries > 0)
class alignas(0x80) dmux_pamf_hle_spurs_queue
{
	T* buffer;

	atomic_t<u32> _size;
	u32 front;
	u32 back;

	template <bool is_peek>
	bool _pop(T* lhs)
	{
		if (_size == 0)
		{
			return false;
		}

		if (lhs)
		{
			*lhs = buffer[front];
		}

		if constexpr (!is_peek)
		{
			front = (front + 1) % max_num_of_entries;
			_size--;
			_size.notify_one();
		}

		return true;
	}

public:
	void init(T (&buffer)[max_num_of_entries])
	{
		this->buffer = buffer;
		new (&_size) atomic_t<u32>(0);
		front = 0;
		back = 0;
	}

	bool pop(T& lhs) { return _pop<false>(&lhs); }
	bool pop() { return _pop<false>(nullptr); }
	bool peek(T& lhs) const { return const_cast<dmux_pamf_hle_spurs_queue*>(this)->_pop<true>(&lhs); }
	bool emplace(auto&&... args)
	{
		if (_size >= max_num_of_entries)
		{
			return false;
		}

		new (&buffer[back]) T(std::forward<decltype(args)>(args)...);

		back = (back + 1) % max_num_of_entries;
		_size++;
		_size.notify_one();

		return true;
	}

	[[nodiscard]] u32 size() const { return this->_size.observe(); }

	void wait() const
	{
		while (_size == 0 && thread_ctrl::state() != thread_state::aborting)
		{
			_size.wait(0, static_cast<atomic_wait_timeout>(10'000'000));
		}
	}
};

enum class DmuxPamfCommandType : u32
{
	enable_es = 0,
	disable_es = 2,
	set_stream = 4,
	release_au = 6,
	flush_es = 8,
	close = 10,
	reset_stream = 12,
	reset_es = 14,
	resume = 16,
};

struct alignas(0x80) DmuxPamfCommand
{
	be_t<DmuxPamfCommandType> type;

	union
	{
		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			be_t<u32> is_avc;
			vm::bptr<std::byte, u64> au_queue_buffer;
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
			vm::ptr<std::byte, be_t<u64, 4>> au_addr;
			be_t<u32> au_size;
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
		}
		release_au;

		struct
		{
			be_t<u32> stream_id;
			be_t<u32> private_stream_id;
			vm::ptr<std::byte, be_t<u64, 4>> au_addr;
		}
		reset_es;
	};

	DmuxPamfCommand() = default;

	DmuxPamfCommand(be_t<DmuxPamfCommandType>&& type)
		: type(type)
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCommandType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id)
		: type(type), disable_flush_es{ stream_id, private_stream_id }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCommandType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const vm::ptr<std::byte,be_t<u64, 4>>& au_addr)
		: type(type), reset_es{ stream_id, private_stream_id, au_addr }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCommandType>&& type, const vm::ptr<std::byte,be_t<u64, 4>>& au_addr, const be_t<u32>& au_size, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id)
		: type(type), release_au{ au_addr, au_size, stream_id, private_stream_id }
	{
	}

	DmuxPamfCommand(be_t<DmuxPamfCommandType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u32>& is_avc, const vm::bptr<std::byte, u64>& au_queue_buffer,
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
			vm::ptr<std::byte, be_t<u64, 4>> au_addr;
			CellCodecTimeStamp pts;
			CellCodecTimeStamp dts;
			be_t<u64, 4> unk;
			u8 reserved[4];
			be_t<u32> au_size;
			be_t<u32> stream_header_size;
			std::array<std::byte, 0x10> stream_header_buf;
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
	};

	be_t<u32> event_queue_was_too_full;

	DmuxPamfEvent() = default;

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& event_queue_was_too_full)
		: type(type), event_queue_was_too_full(event_queue_was_too_full)
	{
	}

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const be_t<u32>& es_id, const be_t<u32>& event_queue_was_too_full)
		: type(type), flush_done{ stream_id, private_stream_id, es_id }, event_queue_was_too_full(event_queue_was_too_full)
	{
	}

	DmuxPamfEvent(be_t<DmuxPamfEventType>&& type, const be_t<u32>& stream_id, const be_t<u32>& private_stream_id, const vm::ptr<std::byte,be_t<u64, 4>>& au_addr, const CellCodecTimeStamp& pts, const CellCodecTimeStamp& dts, const be_t<u64>& unk,
		const be_t<u32>& au_size, const be_t<u32>& au_specific_info_size, const std::array<std::byte, 0x10>& au_specific_info, const be_t<u32>& es_id, const be_t<u32>& is_rap, const be_t<u32>& event_queue_was_too_full)
		: type(type)
		, au_found{ stream_id, private_stream_id, au_addr, pts, dts, static_cast<be_t<u64, 4>>(unk), {}, au_size, au_specific_info_size, au_specific_info, es_id, is_rap }
		, event_queue_was_too_full(event_queue_was_too_full)
	{
	}
};

CHECK_SIZE_ALIGN(DmuxPamfEvent, 0x80, 0x80);

struct alignas(0x80) DmuxPamfStreamInfo
{
	vm::bcptr<std::byte, u64> stream_addr;
	be_t<u32> stream_size;
	be_t<u32> user_data;
	be_t<u32> continuity;
	be_t<u32> is_raw_es;
};

CHECK_SIZE_ALIGN(DmuxPamfStreamInfo, 0x80, 0x80);

enum DmuxPamfStreamTypeIndex
{
	DMUX_PAMF_STREAM_TYPE_INDEX_INVALID = -1,
	DMUX_PAMF_STREAM_TYPE_INDEX_VIDEO,
	DMUX_PAMF_STREAM_TYPE_INDEX_LPCM,
	DMUX_PAMF_STREAM_TYPE_INDEX_AC3,
	DMUX_PAMF_STREAM_TYPE_INDEX_ATRACX,
	DMUX_PAMF_STREAM_TYPE_INDEX_USER_DATA,
};


//----------------------------------------------------------------------------
// SPU thread
//----------------------------------------------------------------------------

class dmux_pamf_context
{
	static constexpr u16 PACK_SIZE = 0x800;
	static constexpr s8 PACK_STUFFING_LENGTH_OFFSET = 0xd;
	static constexpr s8 PES_PACKET_LENGTH_OFFSET = 0x4;
	static constexpr s8 PES_HEADER_DATA_LENGTH_OFFSET = 0x8;
	static constexpr s8 PTS_DTS_FLAG_OFFSET = 0x7;
	static constexpr u8 PACKET_START_CODE_PREFIX = 1;

	static constexpr be_t<u32> M2V_PIC_START       = 0x100;
	static constexpr be_t<u32> AVC_AU_DELIMITER    = 0x109;
	static constexpr be_t<u32> M2V_SEQUENCE_HEADER = 0x1b3;
	static constexpr be_t<u32> M2V_SEQUENCE_END    = 0x1b7;
	static constexpr be_t<u32> PACK_START          = 0x1ba;
	static constexpr be_t<u32> SYSTEM_HEADER       = 0x1bb;
	static constexpr be_t<u32> PRIVATE_STREAM_1    = 0x1bd;
	static constexpr be_t<u32> PRIVATE_STREAM_2    = 0x1bf;
	static constexpr be_t<u32> PROG_END            = 0x1b9;
	static constexpr be_t<u32> VIDEO_STREAM_BASE   = 0x1e0; // The lower 4 bits indicate the channel

	class elementary_stream
	{
	protected:
		// Partial access unit to be written to the output queue
		struct access_unit_chunk
		{
			std::vector<std::byte> cached_data; // Up to three bytes of access unit data from the previous stream section (copy, since it might not be in memory anymore)
			std::span<const std::byte> data;    // Access unit data of the current stream section
		};

		// Access unit output queue
		// Since access units have a variable size, we can't use indices and modulo to automatically wrap around the buffer
		class output_queue
		{
			const std::span<std::byte> buffer;
			std::byte* back = buffer.data();
			const std::byte* front = buffer.data();
			const std::byte* wrap_pos = buffer.data(); // The address where the back pointer wrapped around to the beginning of the queue

		public:
			explicit output_queue(const std::span<std::byte>& buffer) : buffer(buffer) {}

			explicit output_queue(utils::serial& ar)
				: buffer{vm::_ptr<std::byte>(ar.pop<vm::addr_t>()), ar.pop<u32>()}
				, back(vm::_ptr<std::byte>(ar.pop<vm::addr_t>()))
				, front(vm::_ptr<std::byte>(ar.pop<vm::addr_t>()))
				, wrap_pos(vm::_ptr<std::byte>(ar.pop<vm::addr_t>()))
			{
			}

			void save(utils::serial& ar) const { ar(vm::get_addr(buffer.data()), static_cast<u32>(buffer.size()), vm::get_addr(back), vm::get_addr(front), vm::get_addr(wrap_pos)); }

			// Removes the specified number of bytes from the back of the queue
			void pop_back(u32 size);

			// Removes everything up to the specified address from the back of the queue
			void pop_back(std::byte* addr);

			// Removes the specified number of bytes from the front of the queue
			void pop_front(u32 size);

			// Removes all data in the queue
			void clear() { wrap_pos = front = back = buffer.data(); }

			// Writes the access unit chunk to the back of the queue
			void push(const access_unit_chunk& au_chunk);

			// Returns the address of the access unit from the back of the queue with the specified size
			[[nodiscard]] const std::byte* peek_back(u32 size) const { return back - size; }

			// Checks, if the back pointer was wrapped around to the beginning (there are access units behind the back pointer that have not yet been consumed by the user)
			[[nodiscard]] bool is_wrapped_around() const { return wrap_pos != buffer.data(); }

			// Wraps the back pointer around to the beginning of the buffer and sets wrap_pos to where the back pointer was
			void wrap_around() { wrap_pos = back; back = buffer.data(); }

			// Returns the remaining capacity available
			[[nodiscard]] u32 get_remaining_capacity() const { return static_cast<u32>(wrap_pos != buffer.data() ? front - back : std::to_address(buffer.end()) - back); }
		};

		// Access unit info
		struct access_unit
		{
			ENABLE_BITWISE_SERIALIZATION

			enum class state : u8
			{
				none,          // An access unit is not currently being cut out
				incomplete,    // An access unit is currently being cut out
				commenced,     // The current PES-packet contains the beginning of an access unit
				complete,      // The current PES-packet contains the end of an access unit
				size_mismatch, // The distance between sync words and size indicated in the access unit's info header does not match
				m2v_sequence,  // Special case for M2V, access unit commenced, but the next start code does not complete the access unit
			}
			state = state::none;

			bool rap = false;
			bool timestamps_rap_set = false;

			// Since the delimiters of compressed audio streams are allowed to appear anywhere in the stream (instead of just the beginning of an access unit), we need to parse the size of the access unit from the stream
			u8 size_info_offset = 0;
			u16 parsed_size = 0;

			u32 accumulated_size = 0; // Incremented after every access unit chunk cut out from the stream

			u64 pts = umax;
			u64 dts = umax;

			alignas(0x10) std::array<std::byte, 0x10> au_specific_info_buf{};
		};

	private:
		dmux_pamf_context& ctx;

		enum class state : u8
		{
			initial,
			pushing_au_queue,
			notifying_au_found,
			waiting_for_au_released
		}
		state = state::initial;

		const u8 stream_id;             // 0xe0 to 0xef for video streams, 0xbd for private streams
		const u8 private_stream_id;     // Type and channel for private streams. For video streams, this will be zero
		const u8 au_specific_info_size; // Size of the access unit info in au_specific_info_buf
		const u32 es_id;                // Unique id received by the PPU thread when enabling the elementary stream

		std::span<const std::byte> stream; // Elementary stream section to be processed
		u64 pts = umax;                    // Presentation time stamp from the current PES packet. Will be set to the next new access unit
		u64 dts = umax;                    // Decoder time stamp from the current PES packet. Will be set to the next new access unit
		bool rap = false;                  // Random access point indicator. Will be set to the next new access unit

		output_queue au_queue;

	protected:
		const u32 au_max_size;                                            // Maximum size an access unit can be
		u32 au_size_unk = 0;                                              // For user data streams, used to store the size of the current access unit indicated in the stream header. For other private streams, used as a flag
		alignas(0x10) std::array<std::byte, 0x10> au_specific_info_buf{}; // Stores the first 0x10 bytes of the current stream section, contains info like number of channels for LPCM
		access_unit current_au;                                           // The access unit that is currently being cut out
		access_unit_chunk au_chunk;                                       // A partial access unit that will be written to the access unit queue
		std::vector<std::byte> cache;                                     // The last three bytes of the current stream chunk need to be saved, since they could contain part of the access unit delimiter

	private:
		// Extracts access units from the stream by searching for the access unit delimiter and setting au_chunk accordingly. Returns the number of bytes that were parsed
		virtual u32 parse_stream(const std::span<const std::byte>& stream) = 0;

		void reset()
		{
			state = state::initial;
			au_size_unk = 0;
			pts = umax;
			dts = umax;
			rap = false;
			au_chunk = {};
			current_au = {};
		}

		// Moves the timestamps and rap indicator to the access unit
		void set_au_timestamps_rap()
		{
			current_au.pts = pts;
			current_au.dts = dts;
			current_au.rap = rap;
			pts = umax;
			dts = umax;
			rap = false;
			current_au.timestamps_rap_set = true;
		}

	protected:
		// Returns the stream header size of audio streams. The only difference between LPCM and compressed streams is the unk_size_mask
		template <u32 unk_size_mask>
		u32 parse_audio_stream_header(const std::span<const std::byte>& elementary_stream);

	public:
		elementary_stream(dmux_pamf_context& ctx, u8 stream_id, u8 private_stream_id, u32 es_id, u8 au_specific_info_size, const std::span<std::byte>& au_queue_buffer, u32 au_max_size)
			: ctx(ctx)
			, stream_id(stream_id)
			, private_stream_id(private_stream_id)
			, au_specific_info_size(au_specific_info_size)
			, es_id(es_id)
			, au_queue(au_queue_buffer)
			, au_max_size(au_max_size == umax || au_max_size > au_queue_buffer.size() ? 0x800 : au_max_size)
		{
		}

		elementary_stream(dmux_pamf_context& ctx, utils::serial& ar, u8 stream_id, u8 private_stream_id, u8 au_specific_info_size)
			: ctx(ctx)
			, state(ar.pop<enum state>())
			, stream_id(stream_id)
			, private_stream_id(private_stream_id)
			, au_specific_info_size(au_specific_info_size)
			, es_id(ar.pop<u32>())
			, stream{vm::_ptr<const std::byte>(ar.pop<vm::addr_t>()), ar.pop<u32>()}
			, pts(ar.pop<u64>())
			, dts(ar.pop<u64>())
			, rap(ar.pop<bool>())
			, au_queue(ar)
			, au_max_size(ar.pop<u32>())
			, au_size_unk(ar.pop<u32>())
			, au_specific_info_buf(ar.pop<std::array<std::byte, 0x10>>())
			, current_au(ar.pop<access_unit>())
			, au_chunk(state != state::pushing_au_queue ? access_unit_chunk() : access_unit_chunk{ar.pop<std::vector<std::byte>>(), std::span{vm::_ptr<const std::byte>(ar.pop<vm::addr_t>()), ar.pop<u32>()}})
			, cache(current_au.state == access_unit::state::complete ? std::vector<std::byte>() : ar.pop<std::vector<std::byte>>())
		{
		}

		virtual ~elementary_stream() = default;

		void save(utils::serial& ar);

		static bool is_enabled(const std::unique_ptr<elementary_stream>& es) { return !!es; }
		void set_stream(const std::span<const std::byte>& stream) { this->stream = stream; }
		void set_pts(u64 pts) { this->pts = pts; }
		void set_dts(u64 dts) { this->dts = dts; }
		void set_rap() { rap = true; }

		// Parses the proprietary header of private streams. Returns the size of the header or umax if the stream is invalid
		virtual u32 parse_stream_header(const std::span<const std::byte>& elementary_stream, s8 pts_dts_flag) = 0;

		// Processes the current stream chunk. Returns, whether the stream chunk has been entirely consumed
		bool process_stream_chunk();

		// Removes an access unit from the front of the queue
		void release_au(u32 au_size) { au_queue.pop_front(au_size); }

		// Writes the current cache to the access unit queue and manually completes the access unit. Used for the last access unit since there is no further delimiter to detect the completion of the access unit
		void flush_es();

		// If au_addr is null, clears the entire access unit queue and resets the elementary stream, otherwise removes the access unit at au_addr (and any access unit behind it)
		void reset_es(std::byte* au_addr);

		// Discards the access unit that is currently being cut out
		void discard_access_unit();
	};

	template <bool avc>
	class video_stream final : public elementary_stream
	{
		u32 parse_stream(const std::span<const std::byte>& stream) override;
		u32 parse_stream_header([[maybe_unused]] const std::span<const std::byte>& elementary_stream, [[maybe_unused]] s8 pts_dts_flag) override { return 0; }

	public:
		video_stream(dmux_pamf_context& ctx, u32 stream_id, u32 private_stream_id, u32 es_id, const std::span<std::byte>& au_queue_buffer, u32 au_max_size)
			: elementary_stream(ctx, stream_id, private_stream_id, es_id, 0, au_queue_buffer, au_max_size) {}
		video_stream(dmux_pamf_context& ctx, utils::serial& ar, u32 stream_id) : elementary_stream(ctx, ar, stream_id, 0, 0) {}
	};

	class lpcm_stream final : public elementary_stream
	{
		u32 parse_stream(const std::span<const std::byte>& stream) override;
		u32 parse_stream_header(const std::span<const std::byte>& elementary_stream, [[maybe_unused]] s8 pts_dts_flag) override;

	public:
		lpcm_stream(dmux_pamf_context& ctx, u32 stream_id, u32 private_stream_id, u32 es_id, const std::span<std::byte>& au_queue_buffer, u32 au_max_size)
			: elementary_stream(ctx, stream_id, private_stream_id, es_id, 3, au_queue_buffer, au_max_size) {}
		lpcm_stream(dmux_pamf_context& ctx, utils::serial& ar, u32 private_stream_id) : elementary_stream(ctx, ar, 0xbd, private_stream_id, 3) {}
	};

	template <bool ac3>
	class audio_stream final : public elementary_stream
	{
		static constexpr be_t<u16> SYNC_WORD = ac3 ? 0x0b77 : 0x0fd0;
		static constexpr u8 ATRACX_ATS_HEADER_SIZE = 8;
		static constexpr u16 AC3_FRMSIZE_TABLE[3][38] =
		{
			{ 0x40, 0x40, 0x50, 0x50, 0x60, 0x60, 0x70, 0x70, 0x80, 0x80, 0xa0, 0xa0, 0xc0, 0xc0, 0xe0, 0xe0, 0x100, 0x100, 0x140, 0x140, 0x180, 0x180, 0x1c0, 0x1c0, 0x200, 0x200, 0x280, 0x280, 0x300, 0x300, 0x380, 0x380, 0x400, 0x400, 0x480, 0x480, 0x500, 0x500 },
			{ 0x45, 0x46, 0x57, 0x58, 0x68, 0x69, 0x79, 0x7a, 0x8b, 0x8c, 0xae, 0xaf, 0xd0, 0xd1, 0xf3, 0xf4, 0x116, 0x117, 0x15c, 0x15d, 0x1a1, 0x1a2, 0x1e7, 0x1e8, 0x22d, 0x22e, 0x2b8, 0x2b9, 0x343, 0x344, 0x3cf, 0x3d0, 0x45a, 0x45b, 0x4e5, 0x4e6, 0x571, 0x572 },
			{ 0x60, 0x60, 0x78, 0x78, 0x90, 0x90, 0xa8, 0xa8, 0xc0, 0xc0, 0xf0, 0xf0, 0x120, 0x120, 0x150, 0x150, 0x180, 0x180, 0x1e0, 0x1e0, 0x240, 0x240, 0x2a0, 0x2a0, 0x300, 0x300, 0x3c0, 0x3c0, 0x480, 0x480, 0x540, 0x540, 0x600, 0x600, 0x6c0, 0x6c0, 0x780, 0x780 }
		};

		u32 parse_stream_header(const std::span<const std::byte>& elementary_stream, [[maybe_unused]] s8 pts_dts_flag) override { return parse_audio_stream_header<0xffff>(elementary_stream); }
		u32 parse_stream(const std::span<const std::byte>& stream) override;

	public:
		audio_stream(dmux_pamf_context& ctx, u32 stream_id, u32 private_stream_id, u32 es_id, const std::span<std::byte>& au_queue_buffer, u32 au_max_size)
			: elementary_stream(ctx, stream_id, private_stream_id, es_id, 0, au_queue_buffer, au_max_size) {}
		audio_stream(dmux_pamf_context& ctx, utils::serial& ar, u32 private_stream_id) : elementary_stream(ctx, ar, 0xbd, private_stream_id, 0) {}
	};

	class user_data_stream final : public elementary_stream
	{
		u32 parse_stream(const std::span<const std::byte>& stream) override;
		u32 parse_stream_header(const std::span<const std::byte>& elementary_stream, s8 pts_dts_flag) override;

	public:
		user_data_stream(dmux_pamf_context& ctx, u32 stream_id, u32 private_stream_id, u32 es_id, const std::span<std::byte>& au_queue_buffer, u32 au_max_size)
			: elementary_stream(ctx, stream_id, private_stream_id, es_id, 0, au_queue_buffer, au_max_size) {}
		user_data_stream(dmux_pamf_context& ctx, utils::serial& ar, u32 private_stream_id) : elementary_stream(ctx, ar, 0xbd, private_stream_id, 0) {}
	};

	class demuxer
	{
		// The stream to be demultiplexed, set by the user
		class input_stream
		{
			std::span<const std::byte> stream;
			std::span<const std::byte>::iterator pos;

		public:
			void set_stream(const std::span<const std::byte>& input_stream)
			{
				stream = input_stream;
				pos = stream.begin();
			}

			[[nodiscard]] bool eof() const { return pos >= stream.end(); }

			std::span<const std::byte, PACK_SIZE> get_next_pack()
			{
				const auto old_pos = pos;
				pos += PACK_SIZE;
				return std::span<const std::byte, PACK_SIZE>(old_pos, PACK_SIZE);
			}

			void save(utils::serial& ar)
			{
				if (ar.is_writing())
				{
					ar(vm::get_addr(stream.data()), static_cast<u32>(stream.size()), vm::get_addr(std::to_address(pos)));
				}
				else
				{
					stream = { vm::_ptr<const std::byte>(ar.pop<vm::addr_t>()), ar.pop<u32>() };
					pos = std::span<const std::byte>::iterator(vm::_ptr<const std::byte>(ar.pop<vm::addr_t>()));
				}
			}
		};


		dmux_pamf_context& ctx;

		enum class state : u8
		{
			initial,
			elementary_stream,
			prog_end
		}
		state = state::initial;

		bool demux_done = true;          // User input stream has been entirely consumed
		bool demux_done_notified = true; // User was successfully notified that the stream has been consumed

		u8 pack_es_type_idx = umax; // Elementary stream type in the current pack
		u8 pack_es_channel = 0;     // Elementary stream channel in the current pack

		bool raw_es = false; // Indicates that the input stream is a raw elementary stream instead of a multiplexed MPEG Program Stream. If set to true, MPEG-PS related parsing will be skipped

		input_stream input_stream; // The stream section to be demultiplexed provided by the user

		std::unique_ptr<elementary_stream> elementary_streams[5][0x10]; // One for each possible type and channel


		inline void notify_demux_done();

	public:
		explicit demuxer(dmux_pamf_context& ctx) : ctx(ctx) {}
		demuxer(dmux_pamf_context& ctx, utils::serial& ar) : ctx(ctx) { save(ar); }

		void save(utils::serial& ar);

		[[nodiscard]] bool has_work() const { return !demux_done || !demux_done_notified; }
		[[nodiscard]] u32 get_enabled_es_count() const { return static_cast<u32>(std::ranges::count_if(elementary_streams | std::views::join, elementary_stream::is_enabled)); }

		// Enables an elementary stream to be parsed and split into access units
		void enable_es(u32 stream_id, u32 private_stream_id, bool is_avc, const std::span<std::byte>& au_queue_buffer, u32 au_max_size, bool raw_es, u32 es_id);

		// Disables an elementary stream
		void disable_es(u32 stream_id, u32 private_stream_id);

		// Removes an access unit from the front of the access unit queue
		void release_au(u32 stream_id, u32 private_stream_id, u32 au_size) const;

		// Manually completes the access unit that is currently being cut out
		void flush_es(u32 stream_id, u32 private_stream_id);

		// Sets the stream to be demultiplexed
		void set_stream(const std::span<const std::byte>& stream, bool continuity);

		// Removes the stream that is currently being multiplexed
		void reset_stream();

		// If au_addr is null, clears the entire access unit queue and resets the elementary stream, otherwise removes the access unit at au_addr (and any access unit behind it)
		void reset_es(u32 stream_id, u32 private_stream_id, std::byte* au_addr);

		// Demultiplexes the next pack
		void demux_next_pack();
	};


	// These are globals in the SPU task
	const vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>> cmd_queue;
	const vm::ptr<dmux_pamf_hle_spurs_queue<be_t<u32>, 1>> cmd_result_queue;
	const vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1>> stream_info_queue;
	const vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfEvent, 132>> event_queue;
	demuxer demuxer{ *this };
	bool au_queue_full = false;
	bool event_queue_too_full = false;
	bool event_queue_was_too_full = false; // Sent to the PPU thread
	u8 max_enqueued_events = 4;            // 4 + 2 * number of enabled elementary streams

	bool get_next_cmd(DmuxPamfCommand& lhs, bool new_stream) const;
	bool send_event(auto&&... args) const;

	// This is a local variable in cellSpursMain(), needs to be saved for savestates
	bool new_stream = false;

public:
	static constexpr u32 id_base = 0;
	static constexpr u32 id_step = 1;
	static constexpr u32 id_count = 0x400;
	SAVESTATE_INIT_POS(std::numeric_limits<f64>::max()); // Doesn't depend on or is a dependency of anything

	dmux_pamf_context(vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>> cmd_queue, vm::ptr<dmux_pamf_hle_spurs_queue<be_t<u32>, 1>> cmd_result_queue,
		vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1>> stream_info_queue, vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfEvent, 132>> event_queue)
		: cmd_queue(cmd_queue), cmd_result_queue(cmd_result_queue), stream_info_queue(stream_info_queue), event_queue(event_queue)
	{
	}

	explicit dmux_pamf_context(utils::serial& ar)
		: cmd_queue(ar.pop<vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>>>())
		, cmd_result_queue(vm::ptr<dmux_pamf_hle_spurs_queue<be_t<u32>, 1>>::make(cmd_queue.addr() + sizeof(dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>)))
		, stream_info_queue(vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1>>::make(cmd_result_queue.addr() + sizeof(dmux_pamf_hle_spurs_queue<be_t<u32>, 1>)))
		, event_queue(vm::ptr<dmux_pamf_hle_spurs_queue<DmuxPamfEvent, 132>>::make(stream_info_queue.addr() + sizeof(dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1>)))
		, demuxer(*this, ar)
		, max_enqueued_events(4 + 2 * demuxer.get_enabled_es_count())
		, new_stream(ar.pop<bool>())
	{
	}

	void save(utils::serial& ar);

	void operator()(); // cellSpursMain()
	static constexpr auto thread_name = "PAMF Demuxer Thread"sv;
};

using dmux_pamf_thread = named_thread<dmux_pamf_context>;


//----------------------------------------------------------------------------
// PPU thread
//----------------------------------------------------------------------------

// For some reason, cellDmuxPamf doesn't use regular error code values and also has a second set of error codes that's only used internally
enum CellDmuxPamfError
{
	CELL_DMUX_PAMF_ERROR_BUSY = 1,
	CELL_DMUX_PAMF_ERROR_ARG = 2,
	CELL_DMUX_PAMF_ERROR_UNKNOWN_STREAM = 3,
	CELL_DMUX_PAMF_ERROR_NO_MEMORY = 5,
	CELL_DMUX_PAMF_ERROR_FATAL = 6,
};

enum CellDmuxPamfM2vLevel
{
	CELL_DMUX_PAMF_M2V_MP_LL = 0,
	CELL_DMUX_PAMF_M2V_MP_ML,
	CELL_DMUX_PAMF_M2V_MP_H14,
	CELL_DMUX_PAMF_M2V_MP_HL,
};

enum CellDmuxPamfAvcLevel
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

enum CellDmuxPamfSamplingFrequency
{
	CELL_DMUX_PAMF_FS_48K = 48000,
};

enum CellDmuxPamfBitsPerSample
{
	CELL_DMUX_PAMF_BITS_PER_SAMPLE_16 = 16,
	CELL_DMUX_PAMF_BITS_PER_SAMPLE_24 = 24,
};

enum CellDmuxPamfLpcmChannelAssignmentInfo
{
	CELL_DMUX_PAMF_LPCM_CH_M1 = 1,
	CELL_DMUX_PAMF_LPCM_CH_LR = 3,
	CELL_DMUX_PAMF_LPCM_CH_LRCLSRSLFE = 9,
	CELL_DMUX_PAMF_LPCM_CH_LRCLSCS1CS2RSLFE = 11,
};

enum CellDmuxPamfLpcmFs
{
	CELL_DMUX_PAMF_LPCM_FS_48K = 1,
};

enum CellDmuxPamfLpcmBitsPerSamples
{
	CELL_DMUX_PAMF_LPCM_BITS_PER_SAMPLE_16 = 1,
	CELL_DMUX_PAMF_LPCM_BITS_PER_SAMPLE_24 = 3,
};

struct CellDmuxPamfSpecificInfo
{
	be_t<u32> thisSize;
	b8 programEndCodeCb;
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
constexpr s32 DMUX_PAMF_MAX_ENABLED_ES_NUM = 0x40;

// HLE exclusive, for savestates
enum class dmux_pamf_state : u8
{
	initial,
	waiting_for_au_released,
	waiting_for_au_released_error,
	checking_event_queue,
	waiting_for_event,
	starting_demux_done,
	starting_demux_done_mutex_lock_error,
	starting_demux_done_mutex_unlock_error,
	starting_demux_done_checking_stream_reset,
	starting_demux_done_checking_stream_reset_error,
	setting_au_reset,
	setting_au_reset_error,
	processing_event,
	au_found_waiting_for_spu,
	unsetting_au_cancel,
	demux_done_notifying,
	demux_done_mutex_lock,
	demux_done_cond_signal,
	resuming_demux_mutex_lock,
	resuming_demux_waiting_for_spu,
	sending_fatal_err
};

enum class DmuxPamfSequenceState : u32
{
	dormant,
	resetting,
	running
};

struct DmuxPamfElementaryStream;

class DmuxPamfContext
{
	// HLE exclusive
	// These are local variables in the PPU thread function, they're here for savestates
	DmuxPamfEvent event;
	u64 au_queue_full_bitset;
	b8 stream_reset_started;
	b8 stream_reset_in_progress;

	u32 hle_spu_thread_id;
	dmux_pamf_state savestate;

	[[maybe_unused]] u8 spurs[0xf6b]; // CellSpurs, 0x1000 bytes on LLE
	[[maybe_unused]] vm::bptr<void> spurs_addr; // CellSpurs*
	[[maybe_unused]] b8 use_existing_spurs;

	[[maybe_unused]] alignas(0x80) u8 spurs_taskset[0x1900]; // CellSpursTaskset
	[[maybe_unused]] be_t<u32> spurs_task_id;                // CellSpursTaskId
	vm::bptr<void> spurs_context_addr;

	[[maybe_unused]] u8 reserved1[0x10];

	vm::bptr<DmuxPamfContext> _this;
	be_t<u32> this_size;
	be_t<u32> version;

	DmuxCb<DmuxNotifyDemuxDone> notify_demux_done;
	DmuxCb<DmuxNotifyProgEndCode> notify_prog_end_code;
	DmuxCb<DmuxNotifyFatalErr> notify_fatal_err;

	CellDmuxPamfResource resource;

	be_t<u64> thread_id; // sys_ppu_thread_t

	be_t<u32> unk; // Unused

	be_t<u32> ppu_thread_stack_size;

	be_t<u64> au_released_bitset; // Each bit corresponds to an elementary stream, if a bit is set then cellDmuxReleaseAu() was called for that elementary stream

	b8 stream_reset_requested;

	be_t<DmuxPamfSequenceState> sequence_state;

	be_t<s32> max_enabled_es_num;
	be_t<s32> enabled_es_num;
	vm::bptr<DmuxPamfElementaryStream> elementary_streams[DMUX_PAMF_MAX_ENABLED_ES_NUM];

	be_t<u32> mutex; // sys_mutex_t
	be_t<u32> cond;  // sys_cond_t

	vm::bptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>> cmd_queue_addr_; // Same as cmd_queue_addr, unused
	vm::bptr<DmuxPamfCommand[1]> cmd_queue_buffer_addr_;                     // Same as cmd_queue_buffer_addr, unused

	vm::bptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>> cmd_queue_addr;                                    // CellSpursQueue*
	vm::bptr<dmux_pamf_hle_spurs_queue<be_t<u32>, 1>> cmd_result_queue_addr;                                   // CellSpursQueue*
	vm::bptr<dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1>> stream_info_queue_addr;                         // CellSpursQueue*
	vm::bptr<dmux_pamf_hle_spurs_queue<DmuxPamfEvent, 4 + 2 * DMUX_PAMF_MAX_ENABLED_ES_NUM>> event_queue_addr; // CellSpursQueue*

	vm::bptr<DmuxPamfCommand[1]> cmd_queue_buffer_addr;
	vm::bptr<be_t<u32>[1]> cmd_result_queue_buffer_addr;
	vm::bptr<DmuxPamfEvent[4 + 2 * DMUX_PAMF_MAX_ENABLED_ES_NUM]> event_queue_buffer_addr;
	vm::bptr<DmuxPamfStreamInfo[1]> stream_info_queue_buffer_addr;

	vm::bptr<dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1>> cmd_queue_addr__; // Same as cmd_queue_addr, unused

	be_t<u64> user_data;

	b8 is_raw_es;

	be_t<u32> next_es_id;

	char spurs_taskset_name[24];

	[[maybe_unused]] u8 reserved2[928]; // Unused

	dmux_pamf_hle_spurs_queue<DmuxPamfCommand, 1> cmd_queue;                                    // CellSpursQueue
	dmux_pamf_hle_spurs_queue<be_t<u32>, 1> cmd_result_queue;                                   // CellSpursQueue
	dmux_pamf_hle_spurs_queue<DmuxPamfStreamInfo, 1> stream_info_queue;                         // CellSpursQueue
	dmux_pamf_hle_spurs_queue<DmuxPamfEvent, 4 + 2 * DMUX_PAMF_MAX_ENABLED_ES_NUM> event_queue; // CellSpursQueue

	DmuxPamfCommand cmd_queue_buffer[1];
	alignas(0x80) be_t<u32> cmd_result_queue_buffer[1];
	DmuxPamfStreamInfo stream_info_queue_buffer[1];
	DmuxPamfEvent event_queue_buffer[4 + 2 * DMUX_PAMF_MAX_ENABLED_ES_NUM];

	alignas(0x80) u8 spurs_context[0x36400];


	template <DmuxPamfCommandType type>
	void send_spu_command_and_wait(ppu_thread& ppu, bool waiting_for_spu_state, auto&&... cmd_params);

	error_code wait_au_released_or_stream_reset(ppu_thread& ppu, u64 au_queue_full_bitset, b8& stream_reset_started, dmux_pamf_state& savestate);

	template <bool reset>
	error_code set_au_reset(ppu_thread& ppu);

	template <typename F>
	static error_code callback(ppu_thread& ppu, DmuxCb<F> cb, auto&&... args);

	friend struct DmuxPamfElementaryStream;

public:
	void run_spu_thread(){ hle_spu_thread_id = idm::make<dmux_pamf_thread>(cmd_queue_addr, cmd_result_queue_addr, stream_info_queue_addr, event_queue_addr); }

	DmuxPamfElementaryStream* find_es(u16 stream_id, u16 private_stream_id);

	void exec(ppu_thread& ppu);

	static error_code open(ppu_thread& ppu, const CellDmuxPamfResource& res, const DmuxCb<DmuxNotifyDemuxDone>& notify_dmux_done, const DmuxCb<DmuxNotifyProgEndCode>& notify_prog_end_code, const DmuxCb<DmuxNotifyFatalErr>& notify_fatal_err, vm::bptr<DmuxPamfContext>& handle);
	error_code create_thread(ppu_thread& ppu);
	error_code close(ppu_thread& ppu);
	error_code reset_stream(ppu_thread& ppu);
	error_code join_thread(ppu_thread& ppu);

	template <bool raw_es>
	error_code set_stream(ppu_thread& ppu, vm::cptr<std::byte> stream_address, u32 stream_size, b8 discontinuity, u32 user_data);

	template <bool raw_es>
	error_code enable_es(ppu_thread& ppu, u16 stream_id, u16 private_stream_id, bool is_avc, vm::cptr<void> es_specific_info, vm::ptr<void> mem_addr, u32 mem_size, const DmuxCb<DmuxEsNotifyAuFound>& notify_au_found,
		const DmuxCb<DmuxEsNotifyFlushDone>& notify_flush_done, vm::bptr<DmuxPamfElementaryStream>& es);

	error_code reset_stream_and_wait_done(ppu_thread& ppu);
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
	DmuxCb<DmuxEsNotifyFlushDone> notify_flush_done;

	be_t<u16> stream_id;
	be_t<u16> private_stream_id;
	b8 is_avc;

	vm::bptr<std::byte> au_queue_buffer;
	be_t<u32> unk; // Likely au_queue_buffer_size, unused
	be_t<u32> au_max_size;
	u8 au_specific_info[0x10];
	be_t<u32> au_specific_info_size;

	b8 reset_next_au;

	be_t<u32> es_id;

	u8 reserved[72];

	error_code release_au(ppu_thread& ppu, vm::ptr<std::byte> au_addr, u32 au_size) const;
	error_code disable_es(ppu_thread& ppu);
	error_code flush_es(ppu_thread& ppu) const;
	error_code reset_es(ppu_thread& ppu) const;
};

static_assert(std::is_standard_layout_v<DmuxPamfElementaryStream> && std::is_trivial_v<DmuxPamfElementaryStream>);
CHECK_SIZE_ALIGN(DmuxPamfElementaryStream, 0x98, 4);

struct CellDmuxPamfEsHandle
{
	vm::bptr<DmuxPamfElementaryStream> es;

	DmuxCb<DmuxEsNotifyAuFound> notify_au_found;
	DmuxCb<DmuxEsNotifyFlushDone> notify_flush_done;
};

CHECK_SIZE(CellDmuxPamfEsHandle, 0x14);
