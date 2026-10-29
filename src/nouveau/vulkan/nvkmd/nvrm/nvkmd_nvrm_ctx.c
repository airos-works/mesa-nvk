/*
 * Copyright © 2024 Collabora Ltd. and Red Hat Inc.
 * SPDX-License-Identifier: MIT
 */

#include "nvkmd_nvrm.h"

#include "util/u_debug.h"
#include "util/os_time.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <inttypes.h>

#include "vk_log.h"

#include "util/u_memory.h"
#include "nv_push_clc36f.h"
#include "nv_push_cl906f.h"

#include "class/cl0002.h" // NV01_CONTEXT_DMA
#include "class/clc361.h" // NVC361_NOTIFY_CHANNEL_PENDING
#include "class/clc46f.h" // TURING_CHANNEL_GPFIFO_A
#include "class/cl2080_notification.h" // NV2080_ENGINE_TYPE_GRAPHICS
#include "class/clc36f.h" // VOLTA_CHANNEL_GPFIFO_A
#include "class/cla16f.h" // KeplerBControlGPFifo
#include "class/cl0005.h" // NV01_EVENT
#include "ctrl/ctrla06f/ctrla06fgpfifo.h" // NVA06F_CTRL_CMD_BIND
#include "ctrl/ctrlc36f.h"
#include "ctrl/ctrl906f.h"
#include "ctrl/ctrl2080/ctrl2080rc.h" // NV906F_CTRL_CMD_GET_MMU_FAULT_INFO // NVC36F_CTRL_CMD_INTERNAL_GPFIFO_GET_WORK_SUBMIT_TOKEN

#define SUBC_NVC36F 0
#define SUBC_NV906F 0


#define NV_CHECK(nvRes) {NV_STATUS _nvRes = nvRes; if (_nvRes != NV_OK) { \
	if (getenv("NVK_NVRM_DEBUG") != NULL) \
		fprintf(stderr, "nvrm ctx: %s:%d failed: %#x\n", __func__, __LINE__, _nvRes); \
	vkRes = vk_error(log_obj, VK_ERROR_UNKNOWN); goto error;}}
#define VK_CHECK(vkResIn) {VkResult _vkRes = vkResIn; if (_vkRes != VK_SUCCESS) {vkRes = vk_error(log_obj, _vkRes); goto error;}}


static void
write_gp_fifo_entry(struct nvkmd_nvrm_exec_ctx *ctx, const struct nvkmd_ctx_exec *exec)
{
	//fprintf(stderr, "write_gp_fifo_entry(%#" PRIx64 ", %#" PRIx32 ")\n", exec->addr, exec->size_B);
	uint32_t *ptr = (uint32_t*)ctx->gpFifo->map + 2*ctx->gpPut;

	ptr[0] = DRF_NUM(A16F, _GP_ENTRY0, _GET, NvU64_LO32(exec->addr) >> 2);
	ptr[1] =
		DRF_NUM(A16F, _GP_ENTRY1, _GET_HI, NvU64_HI32(exec->addr)) |
		DRF_NUM(A16F, _GP_ENTRY1, _LENGTH, (exec->size_B >> 2)) |
		DRF_NUM(A16F, _GP_ENTRY1, _SYNC, (exec->no_prefetch ? NVA16F_GP_ENTRY1_SYNC_WAIT : NVA16F_GP_ENTRY1_SYNC_PROCEED));

	ctx->gpPut = (ctx->gpPut + 1) % 0x8000;
}

static void
write_semaphore_release(struct nv_push *push, uint64_t adrGpu, uint64_t value, bool waitForIdle)
{
   P_MTHD(push, NVC36F, SEM_ADDR_LO);
   P_NVC36F_SEM_ADDR_LO(push, (uint32_t)adrGpu >> 2);
   P_NVC36F_SEM_ADDR_HI(push, (uint32_t)(adrGpu >> 32));
   P_NVC36F_SEM_PAYLOAD_LO(push, (uint32_t)value);
   P_NVC36F_SEM_PAYLOAD_HI(push, (uint32_t)(value >> 32));
   P_NVC36F_SEM_EXECUTE(push, {
      .operation = OPERATION_RELEASE,
      .release_wfi = waitForIdle ? RELEASE_WFI_EN : RELEASE_WFI_DIS,
      .payload_size = PAYLOAD_SIZE_64BIT,
      .release_timestamp = RELEASE_TIMESTAMP_DIS,
   });
   P_MTHD(push, NVC36F, NON_STALL_INTERRUPT);
   P_NVC36F_NON_STALL_INTERRUPT(push, 0);
}

/* Channels before Volta only have the GF100 semaphore methods. */
static void
write_semaphore_release_gf100(struct nv_push *push, uint64_t adrGpu, uint32_t value)
{
   P_MTHD(push, NV906F, SEMAPHOREA);
   P_NV906F_SEMAPHOREA(push, (uint32_t)(adrGpu >> 32));
   P_NV906F_SEMAPHOREB(push, (uint32_t)adrGpu >> 2);
   P_NV906F_SEMAPHOREC(push, value);
   const bool waitForIdle = !debug_get_bool_option("NVK_NVRM_NO_SEM_WFI", false);
   if (waitForIdle) {
      P_NV906F_SEMAPHORED(push, {
         .operation = OPERATION_RELEASE,
         .release_wfi = RELEASE_WFI_EN,
         .release_size = RELEASE_SIZE_4BYTE,
      });
   } else {
      P_NV906F_SEMAPHORED(push, {
         .operation = OPERATION_RELEASE,
         .release_wfi = RELEASE_WFI_DIS,
         .release_size = RELEASE_SIZE_4BYTE,
      });
   }
   P_MTHD(push, NV906F, NON_STALL_INTERRUPT);
   P_NV906F_NON_STALL_INTERRUPT(push, 0);
}

static void
write_semaphore_acquire(struct nv_push *push, uint64_t adrGpu, uint64_t value)
{
   P_MTHD(push, NVC36F, SEM_ADDR_LO);
   P_NVC36F_SEM_ADDR_LO(push, (uint32_t)adrGpu >> 2);
   P_NVC36F_SEM_ADDR_HI(push, (uint32_t)(adrGpu >> 32));
   P_NVC36F_SEM_PAYLOAD_LO(push, (uint32_t)value);
   P_NVC36F_SEM_PAYLOAD_HI(push, (uint32_t)(value >> 32));
   P_NVC36F_SEM_EXECUTE(push, {
      .operation = OPERATION_ACQ_STRICT_GEQ,
      .acquire_switch_tsg = ACQUIRE_SWITCH_TSG_EN,
      .payload_size = PAYLOAD_SIZE_64BIT,
   });
}


static VkResult
nvkmd_nvrm_create_exec_ctx(struct nvkmd_dev *_dev,
                              struct vk_object_base *log_obj,
                              enum nvkmd_engines engines,
                              struct nvkmd_ctx **ctx_out)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   VkResult vkRes;

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   struct nvkmd_nvrm_exec_ctx *ctx = CALLOC_STRUCT(nvkmd_nvrm_exec_ctx);
   if (ctx == NULL)
      return vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);

   ctx->base.ops = &nvkmd_nvrm_exec_ctx_ops;
   ctx->base.dev = &dev->base;

   ctx->osEvent = -1;

   VK_CHECK(nvkmd_dev_alloc_mapped_mem(_dev, log_obj,  0x1000,  0x1000, NVKMD_MEM_GART, NVKMD_MEM_MAP_RDWR,  &ctx->notifier));
   VK_CHECK(nvkmd_dev_alloc_mapped_mem(_dev, log_obj, 0x80000, 0x10000, NVKMD_MEM_LOCAL, NVKMD_MEM_MAP_RDWR, &ctx->userD));
   enum nvkmd_mem_flags ctxMemFlags = getenv("NVK_NVRM_CTX_VRAM") != NULL ? NVKMD_MEM_LOCAL : NVKMD_MEM_GART;
   VK_CHECK(nvkmd_dev_alloc_mapped_mem(_dev, log_obj, 0x40000,  0x1000, ctxMemFlags, NVKMD_MEM_MAP_RDWR,  &ctx->gpFifo));
   VK_CHECK(nvkmd_dev_alloc_mapped_mem(_dev, log_obj, 0x10000,  0x1000, ctxMemFlags, NVKMD_MEM_MAP_RDWR,  &ctx->cmdBuf));
   VK_CHECK(nvkmd_dev_alloc_mapped_mem(_dev, log_obj,  0x1000,  0x1000, ctxMemFlags, NVKMD_MEM_MAP_RDWR,  &ctx->sem));

	memset(ctx->notifier->map, 0, ctx->notifier->size_B);
	NV_CONTEXT_DMA_ALLOCATION_PARAMS ctxDmaParams = {
		.flags =
			DRF_DEF(OS03, _FLAGS, _MAPPING, _KERNEL) |
			DRF_DEF(OS03, _FLAGS, _HASH_TABLE, _DISABLE),

		.hMemory = nvkmd_nvrm_mem(ctx->notifier)->hMemoryPhys,
		.offset = 0,
		.limit = ctx->notifier->size_B - 1,
	};
   NV_CHECK(nvRmApiAlloc(&rm, pdev->hDevice, &ctx->hCtxDma, NV01_CONTEXT_DMA, &ctxDmaParams));

   NvU32 engineType = NV2080_ENGINE_TYPE_GRAPHICS;
	NV_CHANNEL_ALLOC_PARAMS createChannelParams = {
		.hObjectError  = ctx->hCtxDma,
		.gpFifoOffset  = ctx->gpFifo->va->addr,
		.gpFifoEntries = 0x8000,
		.flags         = 0,
		.hVASpace      = pdev->hVaSpace,
		.hUserdMemory  = {pdev->hUsermode != 0 ? nvkmd_nvrm_mem(ctx->userD)->hMemoryPhys : 0},
		.userdOffset   = {0},
		.engineType    = engineType,
	};
   NV_CHECK(nvRmApiAlloc(&rm, pdev->hDevice, &ctx->hChannel, pdev->channelClass, &createChannelParams));

	if (pdev->hUsermode == 0) {
		NV_CHECK(nvRmApiMapMemory(&rm, pdev->hSubdevice, ctx->hChannel, 0, 0x1000, false,
			DRF_DEF(OS33, _FLAGS, _FIFO_MAPPING, _ENABLE), &ctx->userdMap));
		ctx->hasUserdMap = true;
	}

   /* Only allocate the engine objects the GPU provides. */
   if (pdev->base.dev_info.cls_copy != 0)
      NV_CHECK(nvRmApiAlloc(&rm, ctx->hChannel, &ctx->subchannels.hCopy, pdev->base.dev_info.cls_copy, NULL));
   if (pdev->base.dev_info.cls_eng2d != 0)
      NV_CHECK(nvRmApiAlloc(&rm, ctx->hChannel, &ctx->subchannels.hEng2d, pdev->base.dev_info.cls_eng2d, NULL));
   NV_CHECK(nvRmApiAlloc(&rm, ctx->hChannel, &ctx->subchannels.hEng3d, pdev->base.dev_info.cls_eng3d, NULL));
   if (pdev->base.dev_info.cls_m2mf != 0)
      NV_CHECK(nvRmApiAlloc(&rm, ctx->hChannel, &ctx->subchannels.hM2mf, pdev->base.dev_info.cls_m2mf, NULL));
   if (pdev->base.dev_info.cls_compute != 0)
      NV_CHECK(nvRmApiAlloc(&rm, ctx->hChannel, &ctx->subchannels.hCompute, pdev->base.dev_info.cls_compute, NULL));


	NVA06F_CTRL_BIND_PARAMS bindParams = {.engineType = engineType};
	NV_CHECK(nvRmApiControl(&rm, ctx->hChannel, NVA06F_CTRL_CMD_BIND, &bindParams, sizeof(bindParams)));

	NVA06F_CTRL_GPFIFO_SCHEDULE_PARAMS scheduleParams = {.bEnable = NV_TRUE};
	NV_CHECK(nvRmApiControl(&rm, ctx->hChannel, NVA06F_CTRL_CMD_GPFIFO_SCHEDULE, &scheduleParams, sizeof(scheduleParams)));

	if (pdev->hUsermode != 0) {
		NVC36F_CTRL_GPFIFO_SET_WORK_SUBMIT_TOKEN_NOTIF_INDEX_PARAMS notifParams = {
			.index = NV_CHANNELGPFIFO_NOTIFICATION_TYPE_WORK_SUBMIT_TOKEN
		};
		NV_CHECK(nvRmApiControl(&rm,
			ctx->hChannel,
			NVC36F_CTRL_CMD_GPFIFO_SET_WORK_SUBMIT_TOKEN_NOTIF_INDEX,
			&notifParams,
			sizeof(notifParams)
		));

		NVC36F_CTRL_CMD_GPFIFO_GET_WORK_SUBMIT_TOKEN_PARAMS tokenParams = {0};
		NV_CHECK(nvRmApiControl(&rm,
			ctx->hChannel,
			NVC36F_CTRL_CMD_GPFIFO_GET_WORK_SUBMIT_TOKEN,
			&tokenParams,
			sizeof(tokenParams)
		));
	}

   if (getenv("NVK_NVRM_DEBUG") != NULL) {
      NvNotification *n = ctx->notifier->map;
      fprintf(stderr, "nvrm ctx created: channel class %#x, error notifier %#x, "
              "classes copy %#x 2d %#x 3d %#x m2mf %#x compute %#x\n",
              pdev->channelClass, n[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_ERROR].info32,
              pdev->base.dev_info.cls_copy, pdev->base.dev_info.cls_eng2d,
              pdev->base.dev_info.cls_eng3d, pdev->base.dev_info.cls_m2mf,
              pdev->base.dev_info.cls_compute);
   }

   ctx->osEvent = open(rm.nodeName, O_RDWR | O_CLOEXEC);
   if (ctx->osEvent < 0) {
      vkRes = VK_ERROR_UNKNOWN;
      goto error;
   }
   struct NvRmApi rmOsEvent = rm;
   rmOsEvent.fd = ctx->osEvent;
   NV_CHECK(nvRmApiAllocOsEvent(&rmOsEvent, ctx->osEvent));

	NV0005_ALLOC_PARAMETERS eventParams = {
		.hParentClient = pdev->hClient,
		.hSrcResource = pdev->hSubdevice,
		.hClass = NV01_EVENT_OS_EVENT,
		.notifyIndex =
			NV2080_NOTIFIERS_GRAPHICS |
			NV01_EVENT_NONSTALL_INTR |
			NV01_EVENT_WITHOUT_EVENT_DATA |
			NV01_EVENT_SUBDEVICE_SPECIFIC |
			DRF_NUM(0005, _NOTIFY_INDEX, _SUBDEVICE, 0),
		.data = (NvP64)(uintptr_t)ctx->osEvent,
	};
   NV_CHECK(nvRmApiAlloc(&rm, pdev->hSubdevice, &ctx->hEvent, NV01_EVENT_OS_EVENT, &eventParams));


   nv_push_init(&ctx->push, ctx->cmdBuf->map, 0x10000 / 4, BITFIELD_BIT(SUBC_NV9097));

   *ctx_out = &ctx->base;
   return VK_SUCCESS;

error:
   nvkmd_ctx_destroy(&ctx->base);
	return vkRes;
}

static void
nvkmd_nvrm_exec_ctx_destroy(struct nvkmd_ctx *_ctx)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(ctx->base.dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   nvRmApiFree(&rm, ctx->hEvent);
   if (ctx->osEvent >= 0)
      close(ctx->osEvent);
   nvRmApiFree(&rm, ctx->subchannels.hCopy);
   nvRmApiFree(&rm, ctx->subchannels.hEng2d);
   nvRmApiFree(&rm, ctx->subchannels.hEng3d);
   nvRmApiFree(&rm, ctx->subchannels.hM2mf);
   nvRmApiFree(&rm, ctx->subchannels.hCompute);
   if (ctx->hasUserdMap)
      nvRmApiUnmapMemory(&rm, pdev->hSubdevice, ctx->hChannel, 0, &ctx->userdMap);
   nvRmApiFree(&rm, ctx->hChannel);
   nvRmApiFree(&rm, ctx->hCtxDma);
   if (ctx->sem != NULL)
      nvkmd_mem_unref(ctx->sem);
   if (ctx->cmdBuf != NULL)
      nvkmd_mem_unref(ctx->cmdBuf);
   if (ctx->gpFifo != NULL)
      nvkmd_mem_unref(ctx->gpFifo);
   if (ctx->userD != NULL)
      nvkmd_mem_unref(ctx->userD);
   if (ctx->notifier != NULL)
      nvkmd_mem_unref(ctx->notifier);

   FREE(ctx);
}

static VkResult
nvkmd_nvrm_exec_ctx_wait(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj,
                            uint32_t wait_count,
                            const struct vk_sync_wait *waits)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_flush(struct nvkmd_ctx *_ctx,
                             struct vk_object_base *log_obj)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(ctx->base.dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   NvNotification *notifiers = ctx->notifier->map;
   NvNotification *submitTokenNotifier = &notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_WORK_SUBMIT_TOKEN];
   KeplerBControlGPFifo *userD = ctx->hasUserdMap
      ? (KeplerBControlGPFifo *)ctx->userdMap.address : ctx->userD->map;
   uint64_t *semAdr = (uint64_t*)ctx->sem->map;
   uint64_t semAdrGpu = ctx->sem->va->addr;

   ctx->wSeq++;

   bool hasDoorbell = pdev->hUsermode != 0;
   if (hasDoorbell)
      write_semaphore_release(&ctx->push, semAdrGpu, ctx->wSeq, true);
   else
      write_semaphore_release_gf100(&ctx->push, semAdrGpu, (uint32_t)ctx->wSeq);

   struct nvkmd_ctx_exec semExec = {
   	.addr = ctx->cmdBuf->va->addr,
   	.size_B = 4*nv_push_dw_count(&ctx->push),
   };

   write_gp_fifo_entry(ctx, &semExec);

   userD->GPPut = ctx->gpPut;

   if (hasDoorbell) {
      volatile NvU32 *doorbell = (void*)((NvU8*)pdev->usermodeMap.address + NVC361_NOTIFY_CHANNEL_PENDING);
      *doorbell = submitTokenNotifier->info32;
   }

   /* Wait for the semaphore.  Resman does not deliver the channel's event on
    * this GPU, so the wait is a poll: spin first, because a submission often
    * finishes in microseconds, then sleep in short steps.  Sleeping through
    * poll() costs a whole millisecond per round even when asking for one, so
    * it is only used once the work has clearly not finished quickly. */
   const uint64_t target = hasDoorbell ? ctx->wSeq : (uint32_t)ctx->wSeq;
   const int spinLimit = debug_get_num_option("NVK_NVRM_SPIN", 200000);
   const uint64_t sleep_us = debug_get_num_option("NVK_NVRM_SLEEP_US", 50);
   /* A channel that hit an error never signals its semaphore, so give up
    * rather than hanging the application for ever. */
   const uint64_t timeout_us =
      debug_get_num_option("NVK_NVRM_TIMEOUT_MS", 10000) * 1000ull;
   const int64_t waitStart = os_time_get();
   for (int round = 0;; round++) {
      uint64_t rSeq = hasDoorbell ? *semAdr : *(uint32_t*)semAdr;
      if (rSeq == target) {
         break;
      }
      if (round < spinLimit) {
#if defined(__i386__) || defined(__x86_64__)
         __builtin_ia32_pause();
#endif
         continue;
      }
		int pollRes = 0;
		if (sleep_us != 0) {
			os_time_sleep(sleep_us);
		} else {
			struct pollfd pollFds[1] = {
				{
					.fd = ctx->osEvent,
					.events = POLLIN|POLLPRI
				}
			};
			pollRes = poll(pollFds, 1, 1);
		}
		if (timeout_us != 0 && (uint64_t)(os_time_get() - waitStart) > timeout_us) {
			fprintf(stderr, "nvrm: channel %#x did not finish within %" PRIu64
				" ms: rSeq %#" PRIx64 ", wSeq %#" PRIx64 ", GPGet %" PRIu32
				", GPPut %" PRIu32 ", error notifier %#x\n",
				(unsigned)ctx->hChannel, timeout_us / 1000, rSeq, ctx->wSeq,
				userD->GPGet, userD->GPPut,
				notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_ERROR].info32);
			return VK_ERROR_DEVICE_LOST;
		}
		if (getenv("NVK_NVRM_DEBUG") != NULL && (round - spinLimit) % 1000 == 0) {
			NV906F_CTRL_GET_MMU_FAULT_INFO_PARAMS faultInfo = {0};
			NV_STATUS faultRes = nvRmApiControl(&rm, ctx->hChannel, NV906F_CTRL_CMD_GET_MMU_FAULT_INFO,
				&faultInfo, sizeof(faultInfo));
			NV906F_CTRL_CMD_GET_DEFER_RC_STATE_PARAMS deferRc = {0};
			NV_STATUS deferRes = nvRmApiControl(&rm, ctx->hChannel, NV906F_CTRL_CMD_GET_DEFER_RC_STATE,
				&deferRc, sizeof(deferRc));
			fprintf(stderr, "nvrm channel: fault %#x addr %#x%08x type %#x '%s', deferRc %#x %d, "
				"error notifier info32 %#x info16 %#x\n", faultRes, faultInfo.addrHi, faultInfo.addrLo,
				faultInfo.faultType, faultInfo.faultString, deferRes, deferRc.bDeferRCPending,
				notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_ERROR].info32,
				notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_ERROR].info16);
			fprintf(stderr, "nvrm flush channel %#x: poll %d, rSeq %#" PRIx64 ", wSeq %#" PRIx64
				", GPGet %" PRIu32 ", GPPut %" PRIu32 ", put %" PRIu64 "\n", (unsigned)ctx->hChannel, pollRes, rSeq,
				ctx->wSeq, userD->GPGet, userD->GPPut, (uint64_t)ctx->gpPut);
		}
   }

   ctx->gpGet = ctx->gpPut;
   nv_push_init(&ctx->push, ctx->cmdBuf->map, 0x10000 / 4, BITFIELD_BIT(SUBC_NV9097));

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_exec(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj,
                            uint32_t exec_count,
                            const struct nvkmd_ctx_exec *execs)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   for (uint32_t i = 0; i < exec_count; i++) {
      write_gp_fifo_entry(ctx, &execs[i]);
   }

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_signal(struct nvkmd_ctx *_ctx,
                              struct vk_object_base *log_obj,
                              uint32_t signal_count,
                              const struct vk_sync_signal *signals)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   /* The flush returns once the channel has run everything before it. */
   VkResult result = nvkmd_nvrm_exec_ctx_flush(&ctx->base, log_obj);
   if (result != VK_SUCCESS)
      return result;

   for (uint32_t i = 0; i < signal_count; i++)
      nvkmd_nvrm_sync_set_signaled(signals[i].sync, signals[i].signal_value);

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_sync(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   return nvkmd_nvrm_exec_ctx_flush(&ctx->base, log_obj);
}

const struct nvkmd_ctx_ops nvkmd_nvrm_exec_ctx_ops = {
   .destroy = nvkmd_nvrm_exec_ctx_destroy,
   .wait = nvkmd_nvrm_exec_ctx_wait,
   .exec = nvkmd_nvrm_exec_ctx_exec,
   .signal = nvkmd_nvrm_exec_ctx_signal,
   .flush = nvkmd_nvrm_exec_ctx_flush,
   .sync = nvkmd_nvrm_exec_ctx_sync,
};

static VkResult
nvkmd_nvrm_create_bind_ctx(struct nvkmd_dev *_dev,
                              struct vk_object_base *log_obj,
                              struct nvkmd_ctx **ctx_out)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);

   struct nvkmd_nvrm_bind_ctx *ctx = CALLOC_STRUCT(nvkmd_nvrm_bind_ctx);
   if (ctx == NULL)
      return vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);

   ctx->base.ops = &nvkmd_nvrm_bind_ctx_ops;
   ctx->base.dev = &dev->base;

   // TODO: implement using NV_MEMORY_MAPPER

   *ctx_out = &ctx->base;

   return VK_SUCCESS;
}

static void
nvkmd_nvrm_bind_ctx_destroy(struct nvkmd_ctx *_ctx)
{
   struct nvkmd_nvrm_bind_ctx *ctx = nvkmd_nvrm_bind_ctx(_ctx);

   FREE(ctx);
}

static VkResult
nvkmd_nvrm_bind_ctx_wait(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj,
                            uint32_t wait_count,
                            const struct vk_sync_wait *waits)
{
   struct nvkmd_nvrm_bind_ctx *ctx = nvkmd_nvrm_bind_ctx(_ctx);

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_bind_ctx_flush(struct nvkmd_ctx *_ctx,
                             struct vk_object_base *log_obj)
{
   struct nvkmd_nvrm_bind_ctx *ctx = nvkmd_nvrm_bind_ctx(_ctx);

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_bind_ctx_bind(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj,
                            uint32_t bind_count,
                            const struct nvkmd_ctx_bind *binds)
{
   struct nvkmd_nvrm_bind_ctx *ctx = nvkmd_nvrm_bind_ctx(_ctx);

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_bind_ctx_signal(struct nvkmd_ctx *_ctx,
                              struct vk_object_base *log_obj,
                              uint32_t signal_count,
                              const struct vk_sync_signal *signals)
{
   struct nvkmd_nvrm_bind_ctx *ctx = nvkmd_nvrm_bind_ctx(_ctx);

   VkResult result = nvkmd_nvrm_bind_ctx_flush(&ctx->base, log_obj);
   if (result != VK_SUCCESS)
      return result;

   for (uint32_t i = 0; i < signal_count; i++)
      nvkmd_nvrm_sync_set_signaled(signals[i].sync, signals[i].signal_value);

   return VK_SUCCESS;
}

const struct nvkmd_ctx_ops nvkmd_nvrm_bind_ctx_ops = {
   .destroy = nvkmd_nvrm_bind_ctx_destroy,
   .wait = nvkmd_nvrm_bind_ctx_wait,
   .bind = nvkmd_nvrm_bind_ctx_bind,
   .signal = nvkmd_nvrm_bind_ctx_signal,
   .flush = nvkmd_nvrm_bind_ctx_flush,
};

VkResult
nvkmd_nvrm_create_ctx(struct nvkmd_dev *dev,
                         struct vk_object_base *log_obj,
                         enum nvkmd_engines engines,
                         struct nvkmd_ctx **ctx_out)
{
   if (engines == NVKMD_ENGINE_BIND) {
      return nvkmd_nvrm_create_bind_ctx(dev, log_obj, ctx_out);
   } else {
      assert(!(engines & NVKMD_ENGINE_BIND));
      return nvkmd_nvrm_create_exec_ctx(dev, log_obj, engines, ctx_out);
   }
}
