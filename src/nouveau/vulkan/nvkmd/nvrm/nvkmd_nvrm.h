/*
 * Copyright © 2024 Collabora Ltd. and Red Hat Inc.
 * SPDX-License-Identifier: MIT
 */
#ifndef NVKMD_NVRM_H
#define NVKMD_NVRM_H 1

#include "nvkmd/nvkmd.h"
#include "util/vma.h"
#include "vk_sync.h"
#include "nv_push.h"
#include "util/simple_mtx.h"
#include "util/u_atomic.h"

#include <sys/types.h>

#include "nvRmApi.h"

#include "nvtypes.h"
#include "ctrl/ctrl2080/ctrl2080fb.h" // NV2080_CTRL_CMD_FB_GET_SEMAPHORE_SURFACE_LAYOUT

struct nvrm_ws_bo;
struct nvrm_ws_context;
struct nvrm_ws_device;

struct nvkmd_nvrm_pdev {
   struct nvkmd_pdev base;

   struct vk_sync_type syncobj_sync_type;
   const struct vk_sync_type *sync_types[2];

   char *devName;
   int ctlFd;
   int devFd;
   NvHandle hClient;
   NvHandle hDevice;
   NvHandle hSubdevice;
   NvHandle hUsermode;
   struct NvRmApiMapping usermodeMap;
   NvHandle hVaSpace;
   uint64_t vaStart, vaEnd; // the VA space RM set up for hVaSpace
   struct NV2080_CTRL_FB_GET_SEMAPHORE_SURFACE_LAYOUT_PARAMS semSurfLayout;
   uint32_t channelClass;

   uint32_t numClasses;
   uint32_t *classList;
};

NVKMD_DECL_SUBCLASS(pdev, nvrm);

VkResult nvkmd_nvrm_try_create_pdev(struct _drmDevice *drm_device,
                                       struct vk_object_base *log_obj,
                                       enum nvk_debug debug_flags,
                                       struct nvkmd_pdev **pdev_out);

struct nvkmd_nvrm_dev {
   struct nvkmd_dev base;
   struct hash_table *mappings;
};

NVKMD_DECL_SUBCLASS(dev, nvrm);

VkResult nvkmd_nvrm_create_dev(struct nvkmd_pdev *pdev,
                                  struct vk_object_base *log_obj,
                                  struct nvkmd_dev **dev_out);


VkResult
nvkmd_nvrm_enum_pdev(struct vk_object_base *log_obj,
                     enum nvk_debug debug_flags,
                     nvkmd_enum_pdev_visitor visitor,
                     void *arg);


struct nvkmd_nvrm_mem {
   struct nvkmd_mem base;
   NvHandle hMemoryPhys;
   bool isSystemMem;
};

NVKMD_DECL_SUBCLASS(mem, nvrm);

VkResult nvkmd_nvrm_alloc_mem(struct nvkmd_dev *dev,
                                 struct vk_object_base *log_obj,
                                 uint64_t size_B, uint64_t align_B,
                                 enum nvkmd_mem_flags flags,
                                 struct nvkmd_mem **mem_out);

VkResult nvkmd_nvrm_alloc_tiled_mem(struct nvkmd_dev *dev,
                                       struct vk_object_base *log_obj,
                                       uint64_t size_B, uint64_t align_B,
                                       uint8_t pte_kind, uint16_t tile_mode,
                                       enum nvkmd_mem_flags flags,
                                       struct nvkmd_mem **mem_out);

#define NVKMD_NVRM_HOST_PTR_ALIGNMENT 4096

VkResult nvkmd_nvrm_import_host_ptr(struct nvkmd_dev *dev,
                                    struct vk_object_base *log_obj,
                                    void *ptr, uint64_t size_B,
                                    struct nvkmd_mem **mem_out);

VkResult nvkmd_nvrm_import_scanout(struct nvkmd_dev *dev,
                                   struct vk_object_base *log_obj,
                                   struct nvkmd_scanout_info *info_out,
                                   struct nvkmd_mem **mem_out);

VkResult nvkmd_nvrm_import_dma_buf(struct nvkmd_dev *dev,
                                      struct vk_object_base *log_obj,
                                      int fd, struct nvkmd_mem **mem_out);

struct nvkmd_nvrm_va {
   struct nvkmd_va base;
   NvHandle hMemoryPhys;
   NvHandle hMemoryVirt;
};

NVKMD_DECL_SUBCLASS(va, nvrm);

VkResult nvkmd_nvrm_alloc_va(struct nvkmd_dev *dev,
                                struct vk_object_base *log_obj,
                                enum nvkmd_va_flags flags, uint8_t pte_kind,
                                uint64_t size_B, uint64_t align_B,
                                uint64_t fixed_addr, struct nvkmd_va **va_out);

struct nvkmd_nvrm_exec_ctx {
   struct nvkmd_ctx base;
   struct nvkmd_mem *notifier;
   struct nvkmd_mem *userD;
   /* Before Volta the RM owns USERD; it is reached by mapping the channel. */
   struct NvRmApiMapping userdMap;
   bool hasUserdMap;
   struct nvkmd_mem *gpFifo;
   struct nvkmd_mem *cmdBuf;
   struct nvkmd_mem *sem;
   NvHandle hCtxDma;
   NvHandle hChannel;
   struct {
	   NvHandle hCopy;
	   NvHandle hEng2d;
	   NvHandle hEng3d;
	   NvHandle hM2mf;
	   NvHandle hCompute;
   } subchannels;
   int osEvent;
   NvHandle hEvent;
   uint64_t wSeq;
   uint64_t gpGet;
   uint64_t gpPut;
   struct nv_push push;
   /* The channel's semaphore is 64-bit (Volta and later). */
   bool wideSem;
};

NVKMD_DECL_SUBCLASS(ctx, nvrm_exec);

struct nvkmd_nvrm_bind_ctx {
   struct nvkmd_ctx base;
};

NVKMD_DECL_SUBCLASS(ctx, nvrm_bind);

VkResult nvkmd_nvrm_create_ctx(struct nvkmd_dev *dev,
                                  struct vk_object_base *log_obj,
                                  enum nvkmd_engines engines,
                                  struct nvkmd_ctx **ctx_out);

struct nvkmd_nvrm_sync {
   struct vk_sync base;
   simple_mtx_t lock;
   /* The value it has reached (1 when a binary one is signalled). */
   uint64_t value;
   /* A signal submitted to a channel and not seen to have run yet: the
    * channel's semaphore (referenced while pending), whether it is 64-bit,
    * the sequence number the channel writes to it after that submission,
    * and the value it then signals. */
   struct nvkmd_mem *pendingSem;
   bool pendingWide;
   uint64_t pendingSeq;
   uint64_t pendingValue;
};

void
nvkmd_nvrm_sync_finish(struct vk_device *device,
                       struct vk_sync *sync);

static inline bool
vk_sync_type_is_nvkmd_nvrm_sync(const struct vk_sync_type *type)
{
   return type->finish == nvkmd_nvrm_sync_finish;
}

static inline struct nvkmd_nvrm_sync *
vk_sync_as_nvkmd_nvrm_sync(struct vk_sync *sync)
{
   if (!vk_sync_type_is_nvkmd_nvrm_sync(sync->type))
      return NULL;

   return container_of(sync, struct nvkmd_nvrm_sync, base);
}

struct vk_sync_type
nvkmd_nvrm_sync_get_type(struct nvkmd_nvrm_pdev *pdev);

/* Marks a sync signalled with value (1 for a binary one) once the work that
 * signals it has run. */
void
nvkmd_nvrm_sync_set_signaled(struct vk_sync *sync, uint64_t value);

/* Notes a signal submitted to the channel whose semaphore is sem: the sync
 * reaches value once the semaphore reaches seq. */
void
nvkmd_nvrm_sync_set_pending(struct vk_sync *sync, struct nvkmd_mem *sem,
                            bool wide, uint64_t seq, uint64_t value);

/* Whether work submitted after now to the channel whose semaphore is sem
 * runs after the sync reaches value: it has, or the signal that makes it is
 * earlier on that channel. */
bool
nvkmd_nvrm_sync_ordered_before(struct vk_sync *sync, uint64_t value,
                               struct nvkmd_mem *sem);

/* Waits on the processor until the sync reaches value. */
VkResult
nvkmd_nvrm_sync_cpu_wait(struct vk_sync *sync, uint64_t value);

/* Whether a channel semaphore read as current has reached seq. */
static inline bool
nvkmd_nvrm_seq_reached(uint64_t current, uint64_t seq, bool wide)
{
   if (wide)
      return current >= seq;
   return (int32_t)((uint32_t)current - (uint32_t)seq) >= 0;
}

static inline uint64_t
nvkmd_nvrm_sem_read(const struct nvkmd_mem *sem, bool wide)
{
   if (wide)
      return p_atomic_read((uint64_t *)sem->map);
   return p_atomic_read((uint32_t *)sem->map);
}


static inline void
nvkmd_nvrm_dev_api_ctl(struct nvkmd_nvrm_pdev *pdev, struct NvRmApi *rm)
{
   rm->fd = pdev->ctlFd;
   rm->hClient = pdev->hClient;
   rm->nodeName = pdev->devName;
}

#endif /* NVKMD_DRM_H */
