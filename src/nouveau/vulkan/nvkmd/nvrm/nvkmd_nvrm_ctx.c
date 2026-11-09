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


#define NVRM_GPFIFO_ENTRIES 0x8000
/* The release that ends each submission is written into one of these slots
 * of the context's command buffer (0x10000 bytes). */
#define NVRM_RELEASE_SLOT_DW 64
#define NVRM_RELEASE_SLOTS (0x10000 / 4 / NVRM_RELEASE_SLOT_DW)

static KeplerBControlGPFifo *
exec_ctx_userd(struct nvkmd_nvrm_exec_ctx *ctx)
{
   return ctx->hasUserdMap ? (KeplerBControlGPFifo *)ctx->userdMap.address
                           : ctx->userD->map;
}

static uint64_t
exec_ctx_done_seq(struct nvkmd_nvrm_exec_ctx *ctx)
{
   return nvkmd_nvrm_sem_read(ctx->sem, ctx->wideSem);
}

/* Waits for the channel to have run the submission numbered seq. Polled:
 * resman does not deliver the channel's event on these GPUs. Spin first, as
 * work often finishes in microseconds, then sleep in short steps. */
static VkResult
exec_ctx_wait_seq(struct nvkmd_nvrm_exec_ctx *ctx, uint64_t seq)
{
   const int spinLimit = debug_get_num_option("NVK_NVRM_SPIN", 20000);
   const uint64_t sleep_us = debug_get_num_option("NVK_NVRM_SLEEP_US", 20);
   /* A channel that hit an error never signals its semaphore, so give up
    * rather than hanging the application for ever. */
   const uint64_t timeout_us =
      debug_get_num_option("NVK_NVRM_TIMEOUT_MS", 10000) * 1000ull;
   const int64_t waitStart = os_time_get();
   for (int round = 0;; round++) {
      uint64_t rSeq = exec_ctx_done_seq(ctx);
      if (nvkmd_nvrm_seq_reached(rSeq, seq, ctx->wideSem))
         return VK_SUCCESS;
      if (round < spinLimit) {
#if defined(__i386__) || defined(__x86_64__)
         __builtin_ia32_pause();
#endif
         continue;
      }
      os_time_sleep(sleep_us);
      if (timeout_us != 0 && (uint64_t)(os_time_get() - waitStart) > timeout_us) {
         KeplerBControlGPFifo *userD = exec_ctx_userd(ctx);
         NvNotification *notifiers = ctx->notifier->map;
         fprintf(stderr, "nvrm: channel %#x did not finish within %" PRIu64
            " ms: rSeq %#" PRIx64 ", waiting for %#" PRIx64 ", wSeq %#" PRIx64
            ", GPGet %" PRIu32 ", GPPut %" PRIu32 ", error notifier %#x\n",
            (unsigned)ctx->hChannel, timeout_us / 1000, rSeq, seq, ctx->wSeq,
            userD->GPGet, userD->GPPut,
            notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_ERROR].info32);
         return VK_ERROR_DEVICE_LOST;
      }
   }
}

/* Waits until the GPFIFO has room for count more entries. */
static VkResult
exec_ctx_reserve_gpfifo(struct nvkmd_nvrm_exec_ctx *ctx, uint32_t count)
{
   KeplerBControlGPFifo *userD = exec_ctx_userd(ctx);
   const int64_t waitStart = os_time_get();
   for (;;) {
      uint32_t get = p_atomic_read(&userD->GPGet);
      uint32_t used = (ctx->gpPut + NVRM_GPFIFO_ENTRIES - get) % NVRM_GPFIFO_ENTRIES;
      if (used + count < NVRM_GPFIFO_ENTRIES - 1)
         return VK_SUCCESS;
      if ((uint64_t)(os_time_get() - waitStart) > 10000000ull) {
         fprintf(stderr, "nvrm: channel %#x GPFIFO stays full\n", (unsigned)ctx->hChannel);
         return VK_ERROR_DEVICE_LOST;
      }
      os_time_sleep(20);
   }
}

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

	ctx->gpPut = (ctx->gpPut + 1) % NVRM_GPFIFO_ENTRIES;
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
   ctx->wideSem = pdev->hUsermode != 0;

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

   /* Let the channel finish what it was given before it goes. */
   if (ctx->sem != NULL && ctx->wSeq != 0)
      exec_ctx_wait_seq(ctx, ctx->wSeq);

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

   /* The channel runs its work in order, so a signal earlier on it needs
    * nothing; anything else (another channel, the processor) is waited for
    * here, before this submission's work is handed over. */
   for (uint32_t i = 0; i < wait_count; i++) {
      if (nvkmd_nvrm_sync_ordered_before(waits[i].sync, waits[i].wait_value, ctx->sem))
         continue;
      VkResult result = nvkmd_nvrm_sync_cpu_wait(waits[i].sync, waits[i].wait_value);
      if (result != VK_SUCCESS)
         return result;
   }
   return VK_SUCCESS;
}

/* Ends a submission: the channel writes its sequence number to the
 * semaphore once everything before has run. Does not wait for that unless
 * NVK_NVRM_SYNC_SUBMIT=1 (as before, for comparisons). */
static VkResult
nvkmd_nvrm_exec_ctx_flush(struct nvkmd_ctx *_ctx,
                             struct vk_object_base *log_obj)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(ctx->base.dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   VkResult result;

   NvNotification *notifiers = ctx->notifier->map;
   NvNotification *submitTokenNotifier = &notifiers[NV_CHANNELGPFIFO_NOTIFICATION_TYPE_WORK_SUBMIT_TOKEN];
   KeplerBControlGPFifo *userD = exec_ctx_userd(ctx);
   uint64_t semAdrGpu = ctx->sem->va->addr;

   ctx->wSeq++;

   /* The slot was last used NVRM_RELEASE_SLOTS submissions ago: the channel
    * has to have read it. */
   if (ctx->wSeq > NVRM_RELEASE_SLOTS) {
      result = exec_ctx_wait_seq(ctx, ctx->wSeq - NVRM_RELEASE_SLOTS);
      if (result != VK_SUCCESS)
         return result;
   }
   const uint32_t slot = ctx->wSeq % NVRM_RELEASE_SLOTS;
   nv_push_init(&ctx->push, (uint32_t *)ctx->cmdBuf->map + slot * NVRM_RELEASE_SLOT_DW,
                NVRM_RELEASE_SLOT_DW, BITFIELD_BIT(SUBC_NV9097));

   bool hasDoorbell = pdev->hUsermode != 0;
   if (hasDoorbell)
      write_semaphore_release(&ctx->push, semAdrGpu, ctx->wSeq, true);
   else
      write_semaphore_release_gf100(&ctx->push, semAdrGpu, (uint32_t)ctx->wSeq);

   struct nvkmd_ctx_exec semExec = {
   	.addr = ctx->cmdBuf->va->addr + slot * NVRM_RELEASE_SLOT_DW * 4,
   	.size_B = 4*nv_push_dw_count(&ctx->push),
   };

   result = exec_ctx_reserve_gpfifo(ctx, 1);
   if (result != VK_SUCCESS)
      return result;
   write_gp_fifo_entry(ctx, &semExec);

   /* The entries and the pushes they name are written before the channel is
    * told about them. */
   __sync_synchronize();
   userD->GPPut = ctx->gpPut;

   if (hasDoorbell) {
      volatile NvU32 *doorbell = (void*)((NvU8*)pdev->usermodeMap.address + NVC361_NOTIFY_CHANNEL_PENDING);
      *doorbell = submitTokenNotifier->info32;
   }

   if (debug_get_bool_option("NVK_NVRM_SYNC_SUBMIT", false))
      return exec_ctx_wait_seq(ctx, ctx->wSeq);
   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_exec(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj,
                            uint32_t exec_count,
                            const struct nvkmd_ctx_exec *execs)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   /* With room for the release that ends the submission. */
   VkResult result = exec_ctx_reserve_gpfifo(ctx, exec_count + 1);
   if (result != VK_SUCCESS)
      return result;
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

   VkResult result = nvkmd_nvrm_exec_ctx_flush(&ctx->base, log_obj);
   if (result != VK_SUCCESS)
      return result;

   /* The signals happen when the channel reaches this submission. */
   for (uint32_t i = 0; i < signal_count; i++) {
      nvkmd_nvrm_sync_set_pending(signals[i].sync, ctx->sem, ctx->wideSem,
                                  ctx->wSeq, signals[i].signal_value);
   }

   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_exec_ctx_sync(struct nvkmd_ctx *_ctx,
                            struct vk_object_base *log_obj)
{
   struct nvkmd_nvrm_exec_ctx *ctx = nvkmd_nvrm_exec_ctx(_ctx);

   VkResult result = nvkmd_nvrm_exec_ctx_flush(&ctx->base, log_obj);
   if (result != VK_SUCCESS)
      return result;
   return exec_ctx_wait_seq(ctx, ctx->wSeq);
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

   /* Binds happen on the processor, at once. */
   for (uint32_t i = 0; i < wait_count; i++) {
      VkResult result = nvkmd_nvrm_sync_cpu_wait(waits[i].sync, waits[i].wait_value);
      if (result != VK_SUCCESS)
         return result;
   }
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
