#include "stdafx.h"
#include "Emu/Cell/lv2/sys_sync.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/IdManager.h"

#include "cellPamf.h"
#include "cellDmux.h"
#include "cellDmuxPamf.h"

#include "util/asm.hpp"

LOG_CHANNEL(cellDmux);

extern vm::gvar<CellDmuxCoreOps> g_cell_dmux_core_ops_pamf;

template <>
void fmt_class_string<CellDmuxError>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](CellDmuxError value)
	{
		switch (value)
		{
			STR_CASE(CELL_DMUX_ERROR_ARG);
			STR_CASE(CELL_DMUX_ERROR_SEQ);
			STR_CASE(CELL_DMUX_ERROR_BUSY);
			STR_CASE(CELL_DMUX_ERROR_EMPTY);
			STR_CASE(CELL_DMUX_ERROR_FATAL);
		}

		return unknown;
	});
}

error_code dmuxGetError(u32 internal_error)
{
	switch (internal_error)
	{
	case 0:  return CELL_OK;
	case 1:  return CELL_DMUX_ERROR_FATAL;
	case 2:  return CELL_DMUX_ERROR_ARG;
	case 3:  return CELL_DMUX_ERROR_ARG;
	case 4:  return CELL_DMUX_ERROR_ARG;
	case 5:  return CELL_DMUX_ERROR_ARG;
	default: return CELL_DMUX_ERROR_FATAL;
	}
}


u32 dmuxNotifyDemuxDone(ppu_thread& ppu, vm::ptr<void> core_handle, u32 error, vm::ptr<CellDmuxHandle> handle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("dmuxNotifyDemuxDone(core_handle=*0x%x, error=%d, handle=*0x%x)", core_handle, error, handle);

	ensure(!!handle); // Not checked on LLE

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{handle->_dx_mhd};

		handle->dmux_status = DMUX_STOPPED;
	}

	if (handle->_this)
	{
		const vm::var<CellDmuxMsg> demuxerMsg{{ CELL_DMUX_MSG_TYPE_DEMUX_DONE, handle->user_data }};

		handle->dmux_cb(ppu, handle, demuxerMsg);
	}

	return 0;
}

u32 dmuxNotifyFatalErr(ppu_thread& ppu, vm::ptr<void> core_handle, u32 error, vm::ptr<CellDmuxHandle> handle)
{
	cellDmux.error("dmuxNotifyError(core_handle=*0x%x, error=%d, handle=*0x%x)", core_handle, error, handle);

	ensure(!!handle); // Not checked on LLE

	const vm::var<CellDmuxMsg> demuxerMsg{{ CELL_DMUX_MSG_TYPE_FATAL_ERR, static_cast<u32>(dmuxGetError(error)) }};

	return handle->dmux_cb(ppu, handle, demuxerMsg);
}

u32 dmuxNotifyProgEndCode(ppu_thread& ppu, vm::ptr<void> core_handle, vm::ptr<CellDmuxHandle> handle)
{
	cellDmux.notice("dmuxNotifyProgEndCode(core_handle=*0x%x, handle=*0x%x)", core_handle, handle);

	ensure(!!handle); // Not checked on LLE

	if (handle->_this)
	{
		const vm::var<CellDmuxMsg> demuxerMsg{{ CELL_DMUX_MSG_TYPE_PROG_END_CODE, handle->user_data }};

		handle->dmux_cb(ppu, handle, demuxerMsg);
	}

	return 0;
}


u32 dmuxEsNotifyAuFound(ppu_thread& ppu, vm::ptr<void> core_es_handle, vm::cptr<DmuxAuInfo> au_info, vm::ptr<CellDmuxEsHandle> es_handle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.trace("dmuxEsNotifyAuFound(core_es_handle=*0x%x, au_info=*0x%x, es_handle=*0x%x)", core_es_handle, au_info, es_handle);

	ensure(!!au_info && !!es_handle); // Not checked on LLE

	if (!es_handle->is_enabled)
	{
		return 0;
	}

	vm::ptr<DmuxAuInfo> _au_info;

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{ es_handle->_dx_mes };

		if (!es_handle->is_enabled)
		{
			return 0;
		}

		if (es_handle->au_queue.allocated_size >= es_handle->au_queue.max_size - 1 + (es_handle->flush_started & 1))
		{
			return 1;
		}

		const auto au_queue_data = vm::ptr<DmuxAuQueueData>::make(es_handle.addr() + sizeof(CellDmuxEsHandle));
		_au_info = (au_queue_data + es_handle->au_queue.back).ptr(&DmuxAuQueueData::au_info);
	}

	_au_info->info = au_info->info;
	std::memcpy(_au_info->specific_info.get_ptr(), au_info->specific_info.get_ptr(), au_info->specific_info_size);

	if (!es_handle->is_enabled)
	{
		return 0;
	}

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{es_handle->_dx_mes};

		if (!es_handle->is_enabled)
		{
			return 0;
		}

		es_handle->au_queue.back = (es_handle->au_queue.back + 1) % es_handle->au_queue.max_size;
		es_handle->au_queue.allocated_size++;
		es_handle->au_queue.size++;
	}

	if (!es_handle->is_enabled)
	{
		return 0;
	}

	const vm::var<CellDmuxEsMsg> es_msg{{ CELL_DMUX_ES_MSG_TYPE_AU_FOUND, es_handle->dmux_handle->user_data }};

	es_handle->es_cb(ppu, es_handle->dmux_handle, es_handle, es_msg);

	return 0;
}

u32 dmuxEsNotifyFlushDone(ppu_thread& ppu, vm::ptr<void> core_es_handle, vm::ptr<CellDmuxEsHandle> es_handle)
{
	cellDmux.notice("dmuxEsNotifyFlushDone(unk=*0x%x, es_handle=*0x%x)", core_es_handle, es_handle);

	ensure(!!es_handle); // Not checked on LLE

	if (!es_handle->dmux_handle->_this || !es_handle->is_enabled)
	{
		return 0;
	}

	es_handle->flush_started &= ~1;

	const vm::var<CellDmuxEsMsg> es_msg{{ CELL_DMUX_ES_MSG_TYPE_FLUSH_DONE, es_handle->dmux_handle->user_data }};

	es_handle->es_cb(ppu, es_handle->dmux_handle, es_handle, es_msg);

	return 0;
}


static vm::cptr<CellDmuxCoreOps> core_ops()
{
	return vm::cptr<CellDmuxCoreOps>::make(*ppu_module_manager::find_static_variable<&g_cell_dmux_core_ops_pamf>().export_addr);
}

error_code dmuxQueryAttr(ppu_thread& ppu, vm::ptr<CellDmuxAttr> demuxerAttr, vm::cptr<void> streamSpecificInfo)
{
	vm::var<CellDmuxPamfAttr> pamf_attr;

	if (u32 ret = dmuxGetError(core_ops()->queryAttr(ppu, streamSpecificInfo, pamf_attr)); ret != CELL_OK)
	{
		return ret;
	}

	demuxerAttr->memSize = ((pamf_attr->max_enabled_es_num * sizeof(vm::ptr<CellDmuxEsHandle>) + sizeof(CellDmuxEsHandle) + 0x7f) & -0x10) + pamf_attr->mem_size + 0xf;
	demuxerAttr->demuxerVerUpper = 0x260000;
	demuxerAttr->demuxerVerLower = pamf_attr->version;

	return CELL_OK;
}

error_code cellDmuxQueryAttr(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::ptr<CellDmuxAttr> demuxerAttr)
{
	cellDmux.notice("cellDmuxQueryAttr(demuxerType=*0x%x, demuxerAttr=*0x%x)", demuxerType, demuxerAttr);

	if (!demuxerType || !demuxerAttr || demuxerType->streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	return dmuxQueryAttr(ppu, demuxerAttr, vm::null);
}

error_code cellDmuxQueryAttr2(ppu_thread& ppu, vm::cptr<CellDmuxType2> demuxerType2, vm::ptr<CellDmuxAttr> demuxerAttr)
{
	cellDmux.notice("cellDmuxQueryAttr2(demuxerType2=*0x%x, demuxerAttr=*0x%x)", demuxerType2, demuxerAttr);

	if (!demuxerType2 || !demuxerAttr || demuxerType2->streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	return dmuxQueryAttr(ppu, demuxerAttr, demuxerType2->streamSpecificInfo);
}

error_code dmuxOpen(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::cptr<CellDmuxResource> demuxerResource, vm::cptr<CellDmuxResourceEx> demuxerResourceEx, vm::cptr<CellDmuxCb> demuxerCb, vm::cptr<void> streamSpecificInfo, vm::pptr<CellDmuxHandle> demuxerHandle)
{
	const vm::var<CellDmuxType2> type{{ demuxerType->streamType, streamSpecificInfo }};
	const vm::var<CellDmuxAttr> attr;

	if (error_code ret = cellDmuxQueryAttr2(ppu, type, attr); ret != CELL_OK)
	{
		return ret;
	}

	if (attr->memSize > demuxerResource->memSize)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellDmuxPamfAttr> core_attr;

	if (error_code ret = dmuxGetError(core_ops()->queryAttr(ppu, streamSpecificInfo, core_attr)); ret != CELL_OK)
	{
		return ret;
	}

	const u32 es_handles_size = core_attr->max_enabled_es_num * sizeof(vm::ptr<CellDmuxEsHandle>);

	const auto handle = vm::ptr<CellDmuxHandle>::make(utils::align(+demuxerResource->memAddr.addr(), 0x10));

	const auto core_mem_addr = vm::ptr<void>::make(utils::align(handle.ptr(&CellDmuxHandle::es_handles).addr() + es_handles_size, 0x10));

	const vm::var<CellDmuxResource> core_resource{{
		.memAddr = core_mem_addr,
		.memSize = demuxerResource->memAddr.addr() + demuxerResource->memSize - core_mem_addr.addr(),
		.ppuThreadPriority = demuxerResource->ppuThreadPriority,
		.ppuThreadStackSize = demuxerResource->ppuThreadStackSize,
		.spuThreadPriority = demuxerResource->spuThreadPriority,
		.numOfSpus = demuxerResource->numOfSpus
	}};

	vm::cptr<CellDmuxResourceSpurs> res_spurs_addr = vm::null;
	const vm::var<CellDmuxResourceSpurs> res_spurs;

	if (demuxerResourceEx)
	{
		res_spurs->spurs = demuxerResourceEx->spurs;
		res_spurs->priority = demuxerResourceEx->priority;
		res_spurs->maxContention = demuxerResourceEx->maxContention;

		res_spurs_addr = res_spurs;
	}

	const auto demux_done_func = vm::bptr<DmuxNotifyDemuxDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxNotifyDemuxDone)));
	const auto prog_end_code_func = vm::bptr<DmuxNotifyProgEndCode>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxNotifyProgEndCode)));
	const auto fatal_err_func = vm::bptr<DmuxNotifyFatalErr>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxNotifyFatalErr)));

	const vm::var<DmuxCb<DmuxNotifyDemuxDone>> cb_demux_done{{ demux_done_func, handle }};
	const vm::var<DmuxCb<DmuxNotifyProgEndCode>> cb_prog_end_code{{ prog_end_code_func, handle }};
	const vm::var<DmuxCb<DmuxNotifyFatalErr>> cb_fatal_err{{ fatal_err_func, handle }};

	const vm::var<vm::bptr<void>> core_handle;

	if (error_code ret = dmuxGetError(core_ops()->open(ppu, streamSpecificInfo, core_resource, res_spurs_addr, cb_demux_done, cb_prog_end_code, cb_fatal_err, core_handle)); ret != CELL_OK)
	{
		return ret;
	}

	handle->_this = handle;
	handle->_this_size = es_handles_size + 0xc0;
	handle->version1 = core_attr->version;
	handle->dmux_status = DMUX_STOPPED;
	handle->dmux_type = *demuxerType;
	handle->dmux_cb = *demuxerCb;
	handle->stream_is_set = false;
	handle->core_handle = *core_handle;
	handle->version2 = core_attr->version;
	handle->user_data = 0;
	handle->max_enabled_es_num = core_attr->max_enabled_es_num;
	handle->enabled_es_num = 0;
	new (handle.ptr(&CellDmuxHandle::_dx_mhd).get_ptr()) shared_mutex();

	*demuxerHandle = handle;

	return CELL_OK;
}

error_code cellDmuxOpen(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::cptr<CellDmuxResource> demuxerResource, vm::cptr<CellDmuxCb> demuxerCb, vm::pptr<CellDmuxHandle> demuxerHandle)
{
	cellDmux.notice("cellDmuxOpen(demuxerType=*0x%x, demuxerResource=*0x%x, demuxerCb=*0x%x, handle=*0x%x)", demuxerType, demuxerResource, demuxerCb, demuxerHandle);

	if (!demuxerType || demuxerType->streamType != CELL_DMUX_STREAM_TYPE_PAMF
		|| !demuxerResource || !demuxerResource->memAddr || demuxerResource->memSize == umax || demuxerResource->ppuThreadStackSize == umax
		|| !demuxerCb || !demuxerCb->cbFunc
		|| !demuxerHandle)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	return dmuxOpen(ppu, demuxerType, demuxerResource, vm::null, demuxerCb, vm::null, demuxerHandle);
}

error_code cellDmuxOpenEx(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::cptr<CellDmuxResourceEx> demuxerResourceEx, vm::cptr<CellDmuxCb> demuxerCb, vm::pptr<CellDmuxHandle> demuxerHandle)
{
	cellDmux.notice("cellDmuxOpenEx(demuxerType=*0x%x, demuxerResourceEx=*0x%x, demuxerCb=*0x%x, demuxerHandle=*0x%x)", demuxerType, demuxerResourceEx, demuxerCb, demuxerHandle);

	if (!demuxerType || demuxerType->streamType != CELL_DMUX_STREAM_TYPE_PAMF
		|| !demuxerResourceEx || !demuxerResourceEx->memAddr || demuxerResourceEx->memSize == umax || demuxerResourceEx->ppuThreadStackSize == umax || !demuxerResourceEx->spurs || demuxerResourceEx->maxContention == 0
		|| (demuxerResourceEx->priority & 0xf0f0f0f0f0f0f0f0) != 0 // for each byte in priority, its value must be less than 0x10
		|| !demuxerCb
		|| !demuxerHandle)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellDmuxResource> demuxerResource{{ demuxerResourceEx->memAddr, demuxerResourceEx->memSize, demuxerResourceEx->ppuThreadPriority, demuxerResourceEx->ppuThreadStackSize, 0xfa, 1 }};

	return dmuxOpen(ppu, demuxerType, demuxerResource, demuxerResourceEx, demuxerCb, vm::null, demuxerHandle);
}

error_code cellDmuxOpenExt(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::cptr<CellDmuxResourceEx> demuxerResourceEx, vm::cptr<CellDmuxCb> demuxerCb, vm::pptr<CellDmuxHandle> demuxerHandle)
{
	cellDmux.notice("cellDmuxOpenExt(demuxerType=*0x%x, demuxerResourceEx=*0x%x, demuxerCb=*0x%x, demuxerHandle=*0x%x)", demuxerType, demuxerResourceEx, demuxerCb, demuxerHandle);

	return cellDmuxOpenEx(ppu, demuxerType, demuxerResourceEx, demuxerCb, demuxerHandle);
}

error_code cellDmuxOpen2(ppu_thread& ppu, vm::cptr<CellDmuxType2> demuxerType2, vm::cptr<CellDmuxResource2> demuxerResource2, vm::cptr<CellDmuxCb> demuxerCb, vm::pptr<CellDmuxHandle> demuxerHandle)
{
	cellDmux.notice("cellDmuxOpen2(demuxerType2=*0x%x, demuxerResource2=*0x%x, demuxerCb=*0x%x, demuxerHandle=*0x%x)", demuxerType2, demuxerResource2, demuxerCb, demuxerHandle);

	if (!demuxerType2 || demuxerType2->streamType != CELL_DMUX_STREAM_TYPE_PAMF
		|| !demuxerResource2
		|| !demuxerCb || !demuxerCb->cbFunc
		|| !demuxerHandle)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellDmuxType> demuxerType{{CELL_DMUX_STREAM_TYPE_PAMF, 0, 0}};

	if (demuxerResource2->isResourceEx)
	{
		if (!demuxerResource2->resourceEx.memAddr || demuxerResource2->resourceEx.memSize == umax || demuxerResource2->resourceEx.ppuThreadStackSize == umax || !demuxerResource2->resourceEx.spurs || demuxerResource2->resourceEx.maxContention == 0
			|| (demuxerResource2->resourceEx.priority & 0xf0f0f0f0f0f0f0f0) != 0) // for each byte in priority, its value must be less than 0x10
		{
			return CELL_DMUX_ERROR_ARG;
		}

		const vm::var<CellDmuxResource> demuxerResource{{ demuxerResource2->resourceEx.memAddr, demuxerResource2->resourceEx.memSize, demuxerResource2->resourceEx.ppuThreadPriority, demuxerResource2->resourceEx.ppuThreadStackSize, 0xfa, 1 }};

		return dmuxOpen(ppu, demuxerType, demuxerResource, demuxerResource2.ptr(&CellDmuxResource2::resourceEx), demuxerCb, demuxerType2->streamSpecificInfo, demuxerHandle);
	}

	if (!demuxerResource2->resource.memAddr || demuxerResource2->resource.memSize == umax || demuxerResource2->resource.ppuThreadStackSize == umax)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	return dmuxOpen(ppu, demuxerType, demuxerResource2.ptr(&CellDmuxResource2::resource), vm::null, demuxerCb, demuxerType2->streamSpecificInfo, demuxerHandle);
}

error_code dmuxDisableEs(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle)
{
	lv2_obj::sleep(ppu);
	std::lock_guard lock{esHandle->_dx_mes};

	const u32 ret = core_ops()->disableEs(ppu, esHandle->core_es_handle);

	esHandle->is_enabled = false;
	esHandle->_this = vm::null;

	return dmuxGetError(ret);
}

error_code cellDmuxClose(ppu_thread& ppu, vm::ptr<CellDmuxHandle> demuxerHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxClose(demuxerHandle=*0x%x)", demuxerHandle);

	if (!demuxerHandle || !demuxerHandle->_this || demuxerHandle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	demuxerHandle->_this = vm::null;

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{demuxerHandle->_dx_mhd};

		for (s32 i = 0; i < demuxerHandle->enabled_es_num; i++)
		{
			if (error_code ret = dmuxDisableEs(ppu, demuxerHandle->es_handles[i]); ret != CELL_OK)
			{
				demuxerHandle->enabled_es_num -= i;
				demuxerHandle->_this = demuxerHandle;
				return ret;
			}

			demuxerHandle->es_handles[i]->dmux_handle = vm::null;
		}

		demuxerHandle->enabled_es_num = 0;
	}

	if (error_code ret = dmuxGetError(core_ops()->close(ppu, demuxerHandle->core_handle)); ret != CELL_OK)
	{
		demuxerHandle->_this = demuxerHandle;
		return ret;
	}

	return CELL_OK;
}

error_code cellDmuxSetStream(ppu_thread& ppu, vm::ptr<CellDmuxHandle> demuxerHandle, vm::cptr<void> streamAddress, u32 streamSize, b8 discontinuity, u64 userData)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxSetStream(demuxerHandle=*0x%x, streamAddress=*0x%x, streamSize=0x%x, discontinuity=%d, userData=0x%llx)", demuxerHandle, streamAddress, streamSize, +discontinuity, userData);

	if (!demuxerHandle || !demuxerHandle->_this || streamSize == 0 || streamSize == umax || demuxerHandle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	if (!(demuxerHandle->dmux_status & DMUX_STOPPED))
	{
		return CELL_DMUX_ERROR_BUSY;
	}

	lv2_obj::sleep(ppu);
	std::lock_guard lock{demuxerHandle->_dx_mhd};

	if (error_code ret = dmuxGetError(core_ops()->setStream(ppu, demuxerHandle->core_handle, streamAddress, streamSize, discontinuity, userData)); ret != CELL_OK)
	{
		return ret;
	}

	demuxerHandle->stream_is_set = true;
	demuxerHandle->dmux_status = DMUX_IN_PROGRESS;
	demuxerHandle->user_data = userData;

	return CELL_OK;
}

error_code cellDmuxResetStream(ppu_thread& ppu, vm::ptr<CellDmuxHandle> demuxerHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxResetStream(demuxerHandle=*0x%x)", demuxerHandle);

	if (!demuxerHandle || !demuxerHandle->_this || demuxerHandle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	u32 dmux_status;

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{demuxerHandle->_dx_mhd};

		dmux_status = demuxerHandle->dmux_status;
	}

	if (!(dmux_status & DMUX_IN_PROGRESS) || !demuxerHandle->stream_is_set)
	{
		return CELL_DMUX_ERROR_SEQ;
	}

	if (error_code ret = dmuxGetError(core_ops()->resetStream(ppu, demuxerHandle->core_handle)); ret != CELL_OK)
	{
		return ret;
	}

	demuxerHandle->stream_is_set = false;

	return CELL_OK;
}

error_code cellDmuxResetStreamAndWaitDone(ppu_thread& ppu, vm::ptr<CellDmuxHandle> demuxerHandle)
{
	cellDmux.notice("cellDmuxResetStreamAndWaitDone(demuxerHandle=*0x%x)", demuxerHandle);

	if (!demuxerHandle || !demuxerHandle->_this || demuxerHandle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	if (error_code ret = dmuxGetError(core_ops()->resetStreamAndWaitDone(ppu, demuxerHandle->core_handle)); ret != CELL_OK)
	{
		return ret;
	}

	// LLE doesn't set demuxerHandle->stream_is_set to false

	return CELL_OK;
}

error_code cellDmuxQueryEsAttr(ppu_thread& ppu, vm::cptr<CellDmuxType> demuxerType, vm::cptr<CellCodecEsFilterId> esFilterId, vm::cptr<void> esSpecificInfo, vm::ptr<CellDmuxEsAttr> esAttr)
{
	cellDmux.notice("cellDmuxQueryEsAttr(demuxerType=*0x%x, esFilterId=*0x%x, esSpecificInfo=*0x%x, esAttr=*0x%x)", demuxerType, esFilterId, esSpecificInfo, esAttr);

	if (!demuxerType || demuxerType->streamType != CELL_DMUX_STREAM_TYPE_PAMF || !esFilterId || !esAttr)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellDmuxPamfEsAttr> core_es_attr;

	if (error_code ret = dmuxGetError(core_ops()->queryEsAttr(ppu, vm::make_var<CellCodecEsFilterId>(*esFilterId), esSpecificInfo, core_es_attr)); ret != CELL_OK)
	{
		return ret;
	}

	esAttr->memSize = utils::align((core_es_attr->au_queue_max_size + 1) * (core_es_attr->specific_info_size + static_cast<u32>(sizeof(DmuxAuQueueData))) + static_cast<u32>(sizeof(CellDmuxEsHandle)), 0x10) + core_es_attr->mem_size + 0xf;

	return CELL_OK;
}

error_code cellDmuxQueryEsAttr2(ppu_thread& ppu, vm::cptr<CellDmuxType2> demuxerType2, vm::cptr<CellCodecEsFilterId> esFilterId, vm::cptr<void> esSpecificInfo, vm::ptr<CellDmuxEsAttr> esAttr)
{
	cellDmux.notice("cellDmuxQueryEsAttr2(demuxerType2=*0x%x, esFilterId=*0x%x, esSpecificInfo=*0x%x, esAttr=*0x%x)", demuxerType2, esFilterId, esSpecificInfo, esAttr);

	const vm::var<CellDmuxType> demuxerType{{ demuxerType2->streamType, 0, 0 }};

	return cellDmuxQueryEsAttr(ppu, demuxerType, esFilterId, esSpecificInfo, esAttr);
}

error_code cellDmuxEnableEs(ppu_thread& ppu, vm::ptr<CellDmuxHandle> demuxerHandle, vm::cptr<CellCodecEsFilterId> esFilterId, vm::cptr<CellDmuxEsResource> esResourceInfo, vm::cptr<CellDmuxEsCb> esCb, vm::cptr<void> esSpecificInfo, vm::pptr<CellDmuxEsHandle> esHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxEnableEs(demuxerHandle=*0x%x, esFilterId=*0x%x, esResourceInfo=*0x%x, esCb=*0x%x, esSpecificInfo=*0x%x, esHandle=**0x%x)", demuxerHandle, esFilterId, esResourceInfo, esCb, esSpecificInfo, esHandle);

	if (!demuxerHandle || !demuxerHandle->_this || demuxerHandle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF
		|| !esFilterId
		|| !esResourceInfo || !esResourceInfo->memAddr || esResourceInfo->memSize == umax
		|| !esCb || !esCb->cbFunc
		|| !esHandle)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	std::lock_guard lock_handle{demuxerHandle->_dx_mhd};

	if (demuxerHandle->enabled_es_num >= demuxerHandle->max_enabled_es_num)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellDmuxEsAttr> esAttr;

	if (error_code ret = cellDmuxQueryEsAttr(ppu, demuxerHandle.ptr(&CellDmuxHandle::dmux_type), esFilterId, esSpecificInfo, esAttr); ret != CELL_OK)
	{
		return ret;
	}

	if (esAttr->memSize > esResourceInfo->memSize)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	const vm::var<CellCodecEsFilterId> es_filter_id{ *esFilterId };

	const vm::var<CellDmuxPamfEsAttr> core_es_attr;

	if (error_code ret = dmuxGetError(core_ops()->queryEsAttr(ppu, es_filter_id, esSpecificInfo, core_es_attr)); ret != CELL_OK)
	{
		return ret;
	}

	core_es_attr->au_queue_max_size++;

	const u32 au_queue_data_size = core_es_attr->au_queue_max_size * (core_es_attr->specific_info_size + sizeof(DmuxAuQueueData));

	const auto es_handle = vm::ptr<CellDmuxEsHandle>::make(utils::align(+esResourceInfo->memAddr.addr(), 0x10));
	const auto core_mem_addr = vm::bptr<void>::make(utils::align(es_handle.addr() + static_cast<u32>(sizeof(CellDmuxEsHandle)) + au_queue_data_size, 0x10));

	const vm::var<CellDmuxEsResource> core_es_resource{{ core_mem_addr, esResourceInfo->memAddr.addr() + esResourceInfo->memSize - core_mem_addr.addr() }};

	new (es_handle.ptr(&CellDmuxEsHandle::_dx_mes).get_ptr()) shared_mutex();

	std::lock_guard lock_es{es_handle->_dx_mes};

	const auto au_found_func = vm::bptr<DmuxEsNotifyAuFound>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxEsNotifyAuFound)));
	const auto flush_done_func = vm::bptr<DmuxEsNotifyFlushDone>::make(g_fxo->get<ppu_function_manager>().func_addr(FIND_FUNC(dmuxEsNotifyFlushDone)));

	const vm::var<DmuxCb<DmuxEsNotifyAuFound>> cb_au_found{{ au_found_func, es_handle }};
	const vm::var<DmuxCb<DmuxEsNotifyFlushDone>> cb_flush_done{{ flush_done_func, es_handle }};

	const vm::var<vm::bptr<void>> core_es_handle;

	if (error_code ret = dmuxGetError(core_ops()->enableEs(ppu, demuxerHandle->core_handle, es_filter_id, core_es_resource, cb_au_found, cb_flush_done, esSpecificInfo, core_es_handle)); ret != CELL_OK)
	{
		return ret;
	}

	es_handle->is_enabled = true;
	es_handle->error_mem_size = 0;
	es_handle->error_count = 0;
	// es_handle->error_mem_addr is not initialized on LLE
	es_handle->_this = es_handle;
	es_handle->_this_size = sizeof(CellDmuxEsHandle) + au_queue_data_size;
	es_handle->_this_index = demuxerHandle->enabled_es_num;
	es_handle->dmux_handle = demuxerHandle;
	es_handle->es_cb = *esCb;
	es_handle->core_es_handle = *core_es_handle;
	es_handle->flush_started = 0;
	es_handle->au_queue.max_size = core_es_attr->au_queue_max_size;
	es_handle->au_queue.allocated_size = 0;
	es_handle->au_queue.size = 0;
	es_handle->au_queue.front = 0;
	es_handle->au_queue.back = 0;
	es_handle->au_queue.allocated_back = 0;

	const auto au_queue_data = vm::ptr<DmuxAuQueueData>::make(es_handle.addr() + sizeof(CellDmuxEsHandle));

	for (u32 i = 0; i < core_es_attr->au_queue_max_size; i++)
	{
		au_queue_data[i].index = i;
		au_queue_data[i].unk = 0;
		au_queue_data[i].au_info.info.auAddr = vm::null;
		au_queue_data[i].au_info.info.auMaxSize = 0;
		au_queue_data[i].au_info.specific_info.set(au_queue_data.addr() + core_es_attr->au_queue_max_size * sizeof(DmuxAuQueueData) + i * core_es_attr->specific_info_size);
		au_queue_data[i].au_info.specific_info_size = core_es_attr->specific_info_size;
	}

	demuxerHandle->es_handles[demuxerHandle->enabled_es_num] = es_handle;
	demuxerHandle->enabled_es_num++;
	*esHandle = es_handle;

	return CELL_OK;
}

error_code cellDmuxDisableEs(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxDisableEs(esHandle=*0x%x)", esHandle);

	if (!esHandle || !esHandle->_this || !esHandle->dmux_handle || esHandle->dmux_handle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	lv2_obj::sleep(ppu);
	std::lock_guard lock{esHandle->dmux_handle->_dx_mhd};

	if (error_code ret = dmuxDisableEs(ppu, esHandle); ret != CELL_OK)
	{
		return ret;
	}

	for (s32 i = 0; i < esHandle->dmux_handle->enabled_es_num - 1; i++)
	{
		if (esHandle->dmux_handle->es_handles[i] == esHandle)
		{
			std::memmove(esHandle->dmux_handle->es_handles + i, esHandle->dmux_handle->es_handles + i + 1, esHandle->dmux_handle->enabled_es_num - 1 - i);
			break;
		}
	}

	esHandle->dmux_handle->es_handles[esHandle->dmux_handle->enabled_es_num - 1] = vm::null;
	esHandle->dmux_handle->enabled_es_num--;

	return CELL_OK;
}

error_code cellDmuxResetEs(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.notice("cellDmuxResetEs(esHandle=*0x%x)", esHandle);

	if (!esHandle || !esHandle->_this || !esHandle->dmux_handle || esHandle->dmux_handle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	u32 dmux_status;

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{esHandle->dmux_handle->_dx_mhd};

		dmux_status = esHandle->dmux_handle->dmux_status;
	}

	if (dmux_status & DMUX_STOPPED)
	{
		return CELL_DMUX_ERROR_SEQ;
	}

	lv2_obj::sleep(ppu);
	std::lock_guard lock{esHandle->dmux_handle->_dx_mhd};

	if (error_code ret = dmuxGetError(core_ops()->resetEs(ppu, esHandle->core_es_handle)); ret != CELL_OK)
	{
		return ret;
	}

	const auto au_queue_data = vm::ptr<DmuxAuQueueData>::make(esHandle.addr() + sizeof(CellDmuxEsHandle));

	for (s32 i = 0; i < esHandle->au_queue.max_size; i++)
	{
		au_queue_data[i].index = i;
		au_queue_data[i].unk = 0;
		au_queue_data[i].au_info.info.auAddr = vm::null;
		au_queue_data[i].au_info.info.auMaxSize = 0;
	}

	esHandle->error_mem_size = 0;
	esHandle->error_count = 0;
	esHandle->au_queue.allocated_size = 0;
	esHandle->au_queue.size = 0;
	esHandle->au_queue.front = 0;
	esHandle->au_queue.back = 0;
	esHandle->au_queue.allocated_back = 0;

	return CELL_OK;
}

template <bool is_peek>
error_code dmuxPopAu(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle, vm::cpptr<CellDmuxAuInfo> auInfo, vm::cpptr<void> auSpecificInfo)
{
	ppu.state += cpu_flag::wait;

	if (!esHandle || !esHandle->_this || !esHandle->dmux_handle || esHandle->dmux_handle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	lv2_obj::sleep(ppu);
	std::lock_guard lock{esHandle->_dx_mes};

	if (esHandle->au_queue.size <= 0)
	{
		return CELL_DMUX_ERROR_EMPTY;
	}

	const auto au_queue_data = vm::ptr<DmuxAuQueueData>::make(esHandle.addr() + sizeof(CellDmuxEsHandle));
	const vm::ptr<DmuxAuInfo> au_info = (au_queue_data + esHandle->au_queue.front).ptr(&DmuxAuQueueData::au_info);

	if (auInfo)
	{
		*auInfo = au_info.ptr(&DmuxAuInfo::info);
	}

	if (auSpecificInfo)
	{
		*auSpecificInfo = au_info->specific_info;
	}

	if constexpr (!is_peek)
	{
		esHandle->au_queue.front = (esHandle->au_queue.front + 1) % esHandle->au_queue.max_size;
		esHandle->au_queue.size--;
	}

	return CELL_OK;
}

error_code cellDmuxGetAu(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle, vm::cpptr<CellDmuxAuInfo> auInfo, vm::cpptr<void> auSpecificInfo)
{
	cellDmux.trace("cellDmuxGetAu(esHandle=*0x%x, auInfo=**0x%x, auSpecificInfo=**0x%x)", esHandle, auInfo, auSpecificInfo);

	return dmuxPopAu<false>(ppu, esHandle, auInfo, auSpecificInfo);
}

error_code cellDmuxPeekAu(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle, vm::cpptr<CellDmuxAuInfo> auInfo, vm::cpptr<void> auSpecificInfo)
{
	cellDmux.trace("cellDmuxPeekAu(esHandle=*0x%x, auInfo=**0x%x, auSpecificInfo=**0x%x)", esHandle, auInfo, auSpecificInfo);

	return dmuxPopAu<true>(ppu, esHandle, auInfo, auSpecificInfo);
}

error_code cellDmuxGetAuEx(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle, vm::cpptr<CellDmuxAuInfoEx> auInfoEx, vm::cpptr<void> auSpecificInfo)
{
	cellDmux.trace("cellDmuxGetAuEx(esHandle=*0x%x, auInfoEx=**0x%x, auSpecificInfo=**0x%x)", esHandle, auInfoEx, auSpecificInfo);

	return dmuxPopAu<false>(ppu, esHandle, static_cast<vm::cpptr<CellDmuxAuInfo>>(auInfoEx), auSpecificInfo);
}

error_code cellDmuxPeekAuEx(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle, vm::cpptr<CellDmuxAuInfoEx> auInfoEx, vm::cpptr<void> auSpecificInfo)
{
	cellDmux.trace("cellDmuxPeekAuEx(esHandle=*0x%x, auInfoEx=**0x%x, auSpecificInfo=**0x%x)", esHandle, auInfoEx, auSpecificInfo);

	return dmuxPopAu<true>(ppu, esHandle, static_cast<vm::cpptr<CellDmuxAuInfo>>(auInfoEx), auSpecificInfo);
}

error_code cellDmuxReleaseAu(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle)
{
	ppu.state += cpu_flag::wait;

	cellDmux.trace("cellDmuxReleaseAu(esHandle=*0x%x)", esHandle);

	if (!esHandle || !esHandle->_this || !esHandle->dmux_handle || esHandle->dmux_handle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	lv2_obj::sleep(ppu);
	std::lock_guard lock{esHandle->_dx_mes};

	vm::bptr<void> mem_addr;
	u32 mem_size;

	if (esHandle->au_queue.allocated_size < 1)
	{
		if (esHandle->error_count == 0)
		{
			return CELL_DMUX_ERROR_SEQ;
		}

		mem_addr = esHandle->error_mem_addr;
		mem_size = esHandle->error_mem_size;
	}
	else
	{
		const auto au_queue_data = vm::ptr<DmuxAuQueueData>::make(esHandle.addr() + sizeof(CellDmuxEsHandle));
		const vm::ptr<DmuxAuInfo> au_info = (au_queue_data + esHandle->au_queue.allocated_back).ptr(&DmuxAuQueueData::au_info);

		mem_size = (esHandle->error_mem_size += au_info->info.auSize);

		if (esHandle->error_count == 0)
		{
			mem_addr = au_info->info.auAddr;
		}
		else
		{
			mem_addr = esHandle->error_mem_addr;
		}

		esHandle->au_queue.allocated_back = (esHandle->au_queue.allocated_back + 1) % esHandle->au_queue.max_size;
		esHandle->au_queue.allocated_size--;

		if (esHandle->au_queue.allocated_size < esHandle->au_queue.size)
		{
			esHandle->au_queue.front = (esHandle->au_queue.front + 1) % esHandle->au_queue.max_size;
			esHandle->au_queue.size--;
		}
	}

	if (error_code ret = dmuxGetError(core_ops()->freeMemory(ppu, esHandle->core_es_handle, mem_addr, mem_size)); ret != CELL_OK)
	{
		if (esHandle->error_count == 0)
		{
			esHandle->error_mem_addr = mem_addr;
		}

		esHandle->error_count++;

		return ret;
	}

	esHandle->error_count = 0;
	esHandle->error_mem_size = 0;

	return CELL_OK;
}

error_code cellDmuxFlushEs(ppu_thread& ppu, vm::ptr<CellDmuxEsHandle> esHandle)
{
	cellDmux.notice("cellDmuxFlushEs(esHandle=*0x%x)", esHandle);

	if (!esHandle || !esHandle->_this || !esHandle->dmux_handle || esHandle->dmux_handle->dmux_type.streamType != CELL_DMUX_STREAM_TYPE_PAMF)
	{
		return CELL_DMUX_ERROR_ARG;
	}

	u32 dmux_status;

	{
		lv2_obj::sleep(ppu);
		std::lock_guard lock{esHandle->dmux_handle->_dx_mhd};

		dmux_status = esHandle->dmux_handle->dmux_status;
	}

	if (!(dmux_status & DMUX_STOPPED))
	{
		return CELL_DMUX_ERROR_SEQ;
	}

	esHandle->flush_started |= 1;

	if (error_code ret = dmuxGetError(core_ops()->flushEs(ppu, esHandle->core_es_handle)); ret != CELL_OK)
	{
		esHandle->flush_started &= ~1;
		return ret;
	}

	return CELL_OK;
}

DECLARE(ppu_module_manager::cellDmux)("cellDmux", []()
{
	REG_FUNC(cellDmux, cellDmuxQueryAttr);
	REG_FUNC(cellDmux, cellDmuxQueryAttr2);
	REG_FUNC(cellDmux, cellDmuxOpen);
	REG_FUNC(cellDmux, cellDmuxOpenEx);
	REG_FUNC(cellDmux, cellDmuxOpenExt); // 0xe075fabc
	REG_FUNC(cellDmux, cellDmuxOpen2);
	REG_FUNC(cellDmux, cellDmuxClose);
	REG_FUNC(cellDmux, cellDmuxSetStream);
	REG_FUNC(cellDmux, cellDmuxResetStream);
	REG_FUNC(cellDmux, cellDmuxResetStreamAndWaitDone);
	REG_FUNC(cellDmux, cellDmuxQueryEsAttr);
	REG_FUNC(cellDmux, cellDmuxQueryEsAttr2);
	REG_FUNC(cellDmux, cellDmuxEnableEs);
	REG_FUNC(cellDmux, cellDmuxDisableEs);
	REG_FUNC(cellDmux, cellDmuxResetEs);
	REG_FUNC(cellDmux, cellDmuxGetAu);
	REG_FUNC(cellDmux, cellDmuxPeekAu);
	REG_FUNC(cellDmux, cellDmuxGetAuEx);
	REG_FUNC(cellDmux, cellDmuxPeekAuEx);
	REG_FUNC(cellDmux, cellDmuxReleaseAu);
	REG_FUNC(cellDmux, cellDmuxFlushEs);

	REG_HIDDEN_FUNC(dmuxNotifyDemuxDone);
	REG_HIDDEN_FUNC(dmuxNotifyFatalErr);
	REG_HIDDEN_FUNC(dmuxNotifyProgEndCode);

	REG_HIDDEN_FUNC(dmuxEsNotifyAuFound);
	REG_HIDDEN_FUNC(dmuxEsNotifyFlushDone);
});
