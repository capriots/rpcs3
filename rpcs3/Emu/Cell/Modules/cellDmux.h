#pragma once

// Error Codes
enum CellDmuxError :u32
{
	CELL_DMUX_ERROR_ARG     = 0x80610201,
	CELL_DMUX_ERROR_SEQ     = 0x80610202,
	CELL_DMUX_ERROR_BUSY    = 0x80610203,
	CELL_DMUX_ERROR_EMPTY   = 0x80610204,
	CELL_DMUX_ERROR_FATAL   = 0x80610205,
};

enum CellDmuxStreamType : s32
{
	CELL_DMUX_STREAM_TYPE_UNDEF = 0,
	CELL_DMUX_STREAM_TYPE_PAMF = 1,
	CELL_DMUX_STREAM_TYPE_TERMINATOR = 2,
};

enum CellDmuxMsgType : s32
{
	CELL_DMUX_MSG_TYPE_DEMUX_DONE = 0,
	CELL_DMUX_MSG_TYPE_FATAL_ERR = 1,
	CELL_DMUX_MSG_TYPE_PROG_END_CODE = 2,
};

enum CellDmuxEsMsgType : s32
{
	CELL_DMUX_ES_MSG_TYPE_AU_FOUND = 0,
	CELL_DMUX_ES_MSG_TYPE_FLUSH_DONE = 1,
};

struct CellDmuxMsg
{
	be_t<s32> msgType; // CellDmuxMsgType
	be_t<u64> supplementalInfo;
};

struct CellDmuxEsMsg
{
	be_t<s32> msgType; // CellDmuxEsMsgType
	be_t<u64> supplementalInfo;
};

struct CellDmuxType
{
	be_t<s32> streamType; // CellDmuxStreamType
	be_t<s32> reserved1;
	be_t<s32> reserved2;
};

struct CellDmuxType2
{
	be_t<s32> streamType;
	vm::bcptr<void> streamSpecificInfo;
};

struct CellDmuxResource
{
	vm::bptr<void> memAddr;
	be_t<u32> memSize;
	be_t<u32> ppuThreadPriority;
	be_t<u32> ppuThreadStackSize;
	be_t<u32> spuThreadPriority;
	be_t<u32> numOfSpus;
};

struct CellDmuxResourceEx
{
	vm::bptr<void> memAddr;
	be_t<u32> memSize;
	be_t<u32> ppuThreadPriority;
	be_t<u32> ppuThreadStackSize;
	vm::bptr<void> spurs; // CellSpurs*
	be_t<u64, 1> priority;
	be_t<u32> maxContention;
};

struct CellDmuxResourceSpurs
{
	vm::bptr<void> spurs; // CellSpurs*
	be_t<u64, 1> priority;
	be_t<u32> maxContention;
};

struct CellDmuxResource2
{
	b8 isResourceEx;
	union
	{
		CellDmuxResource resource;
		CellDmuxResourceEx resourceEx;
	};
};

struct CellDmuxHandle;
struct CellDmuxEsHandle;

using CellDmuxCbMsg = u32(vm::ptr<CellDmuxHandle> demuxerHandle, vm::cptr<CellDmuxMsg> demuxerMsg, vm::ptr<void> cbArg);

using CellDmuxCbEsMsg = u32(vm::ptr<CellDmuxHandle> demuxerHandle, vm::ptr<CellDmuxEsHandle> esHandle, vm::cptr<CellDmuxEsMsg> esMsg, vm::ptr<void> cbArg);

// Used for internal callbacks as well
template <typename F>
struct DmuxCb
{
	vm::bptr<F> cbFunc;
	vm::bptr<void> cbArg;
};

using CellDmuxCb = DmuxCb<CellDmuxCbMsg>;

using CellDmuxEsCb = DmuxCb<CellDmuxCbEsMsg>;

struct CellDmuxAttr
{
	be_t<u32> memSize;
	be_t<u32> demuxerVerUpper;
	be_t<u32> demuxerVerLower;
};

struct CellDmuxEsAttr
{
	be_t<u32> memSize;
};

struct CellDmuxEsResource
{
	vm::bptr<void> memAddr;
	be_t<u32> memSize;
};

struct CellDmuxAuInfo
{
	vm::bptr<void> auAddr;
	be_t<u32> auSize;
	be_t<u32> auMaxSize;
	b8 isRap;
	be_t<u64> userData;
	CellCodecTimeStamp pts;
	CellCodecTimeStamp dts;
};

using CellDmuxAuInfoEx = CellDmuxAuInfo;

struct DmuxAuInfo
{
	CellDmuxAuInfo info;
	vm::bptr<void> specific_info;
	be_t<u32> specific_info_size;
};

struct DmuxAuQueueData
{
	be_t<u32> index;
	u8 unk; // unused
	DmuxAuInfo au_info;
};

CHECK_SIZE(DmuxAuQueueData, 0x38);

struct CellDmuxEsHandle;

enum
{
	DMUX_STOPPED = 1 << 0,
	DMUX_IN_PROGRESS = 1 << 1,
};

struct alignas(0x10) CellDmuxHandle
{
	vm::bptr<CellDmuxHandle> _this;
	be_t<u32> _this_size;
	be_t<u32> version1;
	be_t<u32> dmux_status;
	CellDmuxType dmux_type;
	CellDmuxCb dmux_cb;
	b8 stream_is_set;
	vm::bptr<void> core_handle;
	be_t<u32> version2;
	be_t<u64> user_data;
	be_t<s32> max_enabled_es_num;
	be_t<s32> enabled_es_num;
	shared_mutex _dx_mhd; // sys_mutex_t
	u8 reserved[0x7c];
	vm::bptr<CellDmuxEsHandle> es_handles[0x40];
};

CHECK_SIZE_ALIGN(CellDmuxHandle, 0x1c0, 0x10);

struct alignas(0x10) CellDmuxEsHandle
{
	shared_mutex _dx_mes; // sys_mutex_t
	be_t<u32> is_enabled;
	be_t<u32> error_mem_size;
	be_t<u32> error_count;
	vm::bptr<void> error_mem_addr;
	vm::bptr<CellDmuxEsHandle> _this;
	be_t<u32> _this_size;
	be_t<s32> _this_index;
	vm::bptr<CellDmuxHandle> dmux_handle;
	CellDmuxEsCb es_cb;
	vm::bptr<void> core_es_handle;
	be_t<s32> flush_started;

	struct
	{
		be_t<s32> max_size;
		be_t<s32> allocated_size;
		be_t<s32> size;
		be_t<s32> front;
		be_t<s32> back;
		be_t<s32> allocated_back;
	}
	au_queue;
};

CHECK_SIZE_ALIGN(CellDmuxEsHandle, 0x50, 0x10);

struct CellDmuxPamfAttr;
struct CellDmuxPamfEsAttr;

using DmuxNotifyDemuxDone = error_code(vm::ptr<void>, u32, vm::ptr<void>); // TODO broken ?!?!?!?
using DmuxNotifyFatalErr = error_code(vm::ptr<void>, u32, vm::ptr<void>);
using DmuxNotifyProgEndCode = error_code(vm::ptr<void>, vm::ptr<void>);

using DmuxEsNotifyAuFound = error_code(vm::ptr<void>, vm::cptr<void> au_info, vm::ptr<void>);
using DmuxEsNotifyFlushDone = error_code(vm::ptr<void>, vm::ptr<void>);

using CellDmuxCoreOpQueryAttr = error_code(vm::cptr<void>, vm::ptr<CellDmuxPamfAttr>);
using CellDmuxCoreOpOpen = error_code(vm::cptr<void>, vm::cptr<CellDmuxResource>, vm::cptr<CellDmuxResourceSpurs>, vm::cptr<DmuxCb<DmuxNotifyDemuxDone>>, vm::cptr<DmuxCb<DmuxNotifyProgEndCode>>, vm::cptr<DmuxCb<DmuxNotifyFatalErr>>, vm::pptr<void>);
using CellDmuxCoreOpClose = error_code(vm::ptr<void>);
using CellDmuxCoreOpResetStream = error_code(vm::ptr<void>);
using CellDmuxCoreOpCreateThread = error_code(vm::ptr<void>);
using CellDmuxCoreOpJoinThread = error_code(vm::ptr<void>);
using CellDmuxCoreOpSetStream = error_code(vm::ptr<void>, vm::cptr<void>, u32, b8, u64);
using CellDmuxCoreOpFreeMemory = error_code(vm::ptr<void>, vm::ptr<void>, u32);
using CellDmuxCoreOpQueryEsAttr = error_code(vm::cptr<void>, vm::cptr<void>, vm::ptr<CellDmuxPamfEsAttr>);
using CellDmuxCoreOpEnableEs = error_code(vm::ptr<void>, vm::cptr<void>, vm::cptr<CellDmuxEsResource>, vm::cptr<DmuxCb<DmuxEsNotifyAuFound>>, vm::cptr<DmuxCb<DmuxEsNotifyFlushDone>>, vm::cptr<void>, vm::pptr<void>);
using CellDmuxCoreOpDisableEs = u32(vm::ptr<void>);
using CellDmuxCoreOpFlushEs = u32(vm::ptr<void>);
using CellDmuxCoreOpResetEs = u32(vm::ptr<void>);
using CellDmuxCoreOpResetStreamAndWaitDone = u32(vm::ptr<void>);

struct CellDmuxCoreOps
{
	vm::bptr<CellDmuxCoreOpQueryAttr> queryAttr;
	vm::bptr<CellDmuxCoreOpOpen> open;
	vm::bptr<CellDmuxCoreOpClose> close;
	vm::bptr<CellDmuxCoreOpResetStream> resetStream;
	vm::bptr<CellDmuxCoreOpCreateThread> createThread;
	vm::bptr<CellDmuxCoreOpJoinThread> joinThread;
	vm::bptr<CellDmuxCoreOpSetStream> setStream;
	vm::bptr<CellDmuxCoreOpFreeMemory> freeMemory;
	vm::bptr<CellDmuxCoreOpQueryEsAttr> queryEsAttr;
	vm::bptr<CellDmuxCoreOpEnableEs> enableEs;
	vm::bptr<CellDmuxCoreOpDisableEs> disableEs;
	vm::bptr<CellDmuxCoreOpFlushEs> flushEs;
	vm::bptr<CellDmuxCoreOpResetEs> resetEs;
	vm::bptr<CellDmuxCoreOpResetStreamAndWaitDone> resetStreamAndWaitDone;
};
