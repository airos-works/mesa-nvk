/*
 * Copyright © 2024 Collabora Ltd. and Red Hat Inc.
 * SPDX-License-Identifier: MIT
 */

#include "nvkmd_nvrm.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "util/os_time.h"
#include "util/u_atomic.h"

#include "vk_log.h"

#include "util/u_memory.h"
#include "util/hash_table.h"

#include <sys/ioctl.h>

#include "nv-haiku.h"

#include "class/cl003e.h" // NV01_MEMORY_SYSTEM
#include "class/cl0040.h" // NV01_MEMORY_LOCAL_USER



/* Bytes of BAR1 to leave for others: the accelerant's frame buffer mappings,
 * channels and other processes map through the same window. */
#define NVKMD_NVRM_BAR1_RESERVE_B (32ull * 1024 * 1024)

#define NVKMD_NVRM_DEFAULT_MAX_PAGE_B (2u << 20)

/* The free-memory cache: system memory up to this size per allocation, this
 * much in all, for this long after it was freed. RM's allocation of system
 * memory, its GPU range, GPU mapping and CPU mapping cost 100-300 us, and
 * freeing them another 70-110 us, most of it page table and TLB work in RM
 * (airos-works/haiku#132); reuse costs the clearing of the memory. */
#define NVKMD_NVRM_CACHE_ENTRY_MAX_B (4u << 20)
#define NVKMD_NVRM_CACHE_MAX_B (32u << 20)
#define NVKMD_NVRM_CACHE_AGE_NS (2000ll * 1000 * 1000)

static void nvkmd_nvrm_mem_release(struct nvkmd_nvrm_mem *mem);

static bool
nvkmd_nvrm_mem_cache_enabled(void)
{
   static int enabled = -1;
   if (enabled < 0) {
      const char *env = getenv("NVK_NVRM_MEM_CACHE");
      p_atomic_set(&enabled, env == NULL || atoi(env) != 0);
   }
   return enabled;
}

/* Takes what has been cached too long, or what exceeds the budget, out of the
 * cache into \a evicted. Called with dev->mutex held. */
static void
mem_cache_evict_locked(struct nvkmd_nvrm_dev *dev, int64_t now_ns,
                       uint64_t budget_B, struct list_head *evicted)
{
   list_for_each_entry_safe(struct nvkmd_nvrm_mem, mem, &dev->memCache,
                            cacheLink) {
      if (dev->memCacheBytes <= budget_B &&
          now_ns - mem->freedAt_ns < NVKMD_NVRM_CACHE_AGE_NS)
         break;
      list_del(&mem->cacheLink);
      dev->memCacheBytes -= mem->base.size_B;
      list_addtail(&mem->cacheLink, evicted);
   }
}

static void
mem_cache_release(struct list_head *evicted)
{
   list_for_each_entry_safe(struct nvkmd_nvrm_mem, mem, evicted, cacheLink)
      nvkmd_nvrm_mem_release(mem);
}

void
nvkmd_nvrm_mem_cache_drain(struct nvkmd_nvrm_dev *dev)
{
   struct list_head evicted;
   list_inithead(&evicted);
   simple_mtx_lock(&dev->mutex);
   mem_cache_evict_locked(dev, INT64_MAX / 2, 0, &evicted);
   simple_mtx_unlock(&dev->mutex);
   mem_cache_release(&evicted);
}

/* Memory of exactly this size and kind from the cache, cleared, or NULL. */
static struct nvkmd_nvrm_mem *
mem_cache_take(struct nvkmd_nvrm_dev *dev, uint64_t size_B,
               enum nvkmd_mem_flags flags, uint64_t va_align_B)
{
   struct nvkmd_nvrm_mem *found = NULL;
   struct list_head evicted;
   list_inithead(&evicted);

   simple_mtx_lock(&dev->mutex);
   mem_cache_evict_locked(dev, os_time_get_nano(), NVKMD_NVRM_CACHE_MAX_B,
                          &evicted);
   list_for_each_entry_rev(struct nvkmd_nvrm_mem, mem, &dev->memCache,
                           cacheLink) {
      if (mem->base.size_B == size_B && mem->base.flags == flags &&
          (mem->base.va->addr & (va_align_B - 1)) == 0) {
         list_del(&mem->cacheLink);
         dev->memCacheBytes -= size_B;
         found = mem;
         break;
      }
   }
   simple_mtx_unlock(&dev->mutex);
   mem_cache_release(&evicted);

   if (found == NULL)
      return NULL;

   /* New memory from RM is cleared; so is this. */
   memset(found->idleMapping->address, 0, size_B);

   struct nvkmd_va *va = found->base.va;
   nvkmd_mem_init(&dev->base, &found->base, &nvkmd_nvrm_mem_ops, flags, size_B,
                  dev->base.pdev->bind_align_B);
   found->base.va = va;
   return found;
}

/* Keeps freed memory for reuse; false if it does not qualify. */
static bool
mem_cache_put(struct nvkmd_nvrm_dev *dev, struct nvkmd_nvrm_mem *mem)
{
   if (!mem->cacheable || mem->idleMapping == NULL ||
       mem->base.size_B > NVKMD_NVRM_CACHE_ENTRY_MAX_B ||
       !nvkmd_nvrm_mem_cache_enabled())
      return false;

   struct list_head evicted;
   list_inithead(&evicted);

   const int64_t now_ns = os_time_get_nano();
   mem->freedAt_ns = now_ns;
   simple_mtx_lock(&dev->mutex);
   list_addtail(&mem->cacheLink, &dev->memCache);
   dev->memCacheBytes += mem->base.size_B;
   mem_cache_evict_locked(dev, now_ns, NVKMD_NVRM_CACHE_MAX_B, &evicted);
   simple_mtx_unlock(&dev->mutex);
   mem_cache_release(&evicted);
   return true;
}

static bool
nvkmd_nvrm_bar1_enabled(void)
{
   static int enabled = -1;
   if (enabled < 0) {
      const char *env = getenv("NVK_NVRM_BAR1");
      p_atomic_set(&enabled, env == NULL || atoi(env) != 0);
   }
   return enabled;
}

static bool
nvkmd_nvrm_bar1_has_room(struct nvkmd_nvrm_pdev *pdev, struct NvRmApi *rm,
                         uint64_t size_B)
{
   if (!nvkmd_nvrm_bar1_enabled())
      return false;

   NV2080_CTRL_FB_GET_INFO_V2_PARAMS info = {
      .fbInfoListSize = 1,
      .fbInfoList = {
         {.index = NV2080_CTRL_FB_INFO_INDEX_BAR1_AVAIL_SIZE},
      },
   };
   if (nvRmApiControl(rm, pdev->hSubdevice, NV2080_CTRL_CMD_FB_GET_INFO_V2,
                      &info, sizeof(info)) != NV_OK)
      return false;

   const uint64_t avail_B = info.fbInfoList[0].data * (uint64_t)1024;
   return avail_B >= size_B + NVKMD_NVRM_BAR1_RESERVE_B;
}

/* Largest GPU page size for video memory: NVK_NVRM_PAGES=4k, 64k or 2m. */
static uint32_t
nvkmd_nvrm_max_page_size(void)
{
   static uint32_t max_B = 0;
   if (max_B == 0) {
      const char *env = getenv("NVK_NVRM_PAGES");
      uint32_t v = NVKMD_NVRM_DEFAULT_MAX_PAGE_B;
      if (env != NULL) {
         if (strcmp(env, "4k") == 0)
            v = 4096;
         else if (strcmp(env, "64k") == 0)
            v = 64u << 10;
         else if (strcmp(env, "2m") == 0)
            v = 2u << 20;
      }
      p_atomic_set(&max_B, v);
   }
   return max_B;
}

/* Every page is a TLB entry, and with 4 KiB pages a 4K render target takes
 * 8,100 of them (airos-works/haiku#132). Video memory of 64 KiB or more gets
 * big (64 KiB) pages and from 2 MiB huge (2 MiB) pages, unless rounding up
 * to a whole huge page would waste more than an eighth.
 */
static uint32_t
nvkmd_nvrm_vidmem_page_size(uint64_t size_B)
{
   const uint32_t max_B = nvkmd_nvrm_max_page_size();
   const uint64_t huge_B = 2u << 20;
   if (max_B >= huge_B && size_B >= huge_B &&
       align64(size_B, huge_B) - size_B <= size_B / 8)
      return huge_B;
   if (max_B >= (64u << 10) && size_B >= (64u << 10))
      return 64u << 10;
   return 4096;
}

static NvU32
nvkmd_nvrm_page_size_attr(uint32_t pageSize_B)
{
   switch (pageSize_B) {
   case 64u << 10: return DRF_DEF(OS32, _ATTR, _PAGE_SIZE, _BIG);
   case 2u << 20:  return DRF_DEF(OS32, _ATTR, _PAGE_SIZE, _HUGE);
   default:        return DRF_DEF(OS32, _ATTR, _PAGE_SIZE, _4KB);
   }
}

static NvU32
nvkmd_nvrm_page_size_attr2(uint32_t pageSize_B)
{
   return pageSize_B == (2u << 20) ?
      DRF_DEF(OS32, _ATTR2, _PAGE_SIZE_HUGE, _2MB) : 0;
}

static VkResult
create_mem_or_close_bo(struct nvkmd_nvrm_dev *dev,
                       struct vk_object_base *log_obj,
                       enum nvkmd_mem_flags mem_flags,
                       NvHandle hMemoryPhys, uint64_t size_B,
                       enum nvkmd_va_flags va_flags,
                       uint8_t pte_kind, uint64_t va_align_B,
                       uint32_t pageSize_B,
                       struct nvkmd_mem **mem_out)
{
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   VkResult result;

   struct nvkmd_nvrm_mem *mem = CALLOC_STRUCT(nvkmd_nvrm_mem);
   if (mem == NULL) {
      result = vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);
      goto fail_bo;
   }

   /* Binds stay possible at any 4 KiB boundary whatever the page size:
    * nvkmd_nvrm_va.c maps an unaligned one with smaller pages. */
   nvkmd_mem_init(&dev->base, &mem->base, &nvkmd_nvrm_mem_ops,
                  mem_flags, size_B, dev->base.pdev->bind_align_B);
   mem->hMemoryPhys = hMemoryPhys;
   mem->isSystemMem = (mem_flags & NVKMD_MEM_GART) != 0;
   mem->pageSize_B = pageSize_B;

   result = nvkmd_nvrm_alloc_va_paged(&dev->base, log_obj,
                                      va_flags, pte_kind,
                                      size_B, MAX2(va_align_B, pageSize_B),
                                      0 /* fixed_addr */, pageSize_B,
                                      &mem->base.va);
   if (result != VK_SUCCESS)
      goto fail_mem;

   result = nvkmd_va_bind_mem(mem->base.va, log_obj, 0 /* va_offset_B */,
                              &mem->base, 0 /* mem_offset_B */, size_B);
   if (result != VK_SUCCESS)
      goto fail_va;

   *mem_out = &mem->base;

   return VK_SUCCESS;

fail_va:
   nvkmd_va_free(mem->base.va);
fail_mem:
   FREE(mem);
fail_bo:
   nvRmApiFree(&rm, hMemoryPhys);

   return result;
}

VkResult
nvkmd_nvrm_alloc_mem(struct nvkmd_dev *dev,
                        struct vk_object_base *log_obj,
                        uint64_t size_B, uint64_t align_B,
                        enum nvkmd_mem_flags flags,
                        struct nvkmd_mem **mem_out)
{
   return nvkmd_nvrm_alloc_tiled_mem(dev, log_obj, size_B, align_B,
                                        0 /* pte_kind */, 0 /* tile_mode */,
                                        flags, mem_out);
}

VkResult
nvkmd_nvrm_alloc_tiled_mem(struct nvkmd_dev *_dev,
                              struct vk_object_base *log_obj,
                              uint64_t size_B, uint64_t align_B,
                              uint8_t pte_kind, uint16_t tile_mode,
                              enum nvkmd_mem_flags flags,
                              struct nvkmd_mem **mem_out)
{
   if (pte_kind != 0 || tile_mode != 0) {
      fprintf(stderr, "alloc_tiled_mem(%#" PRIx64 ", %#" PRIx64 ", %#" PRIx8 ", %#" PRIx16 ", %#x)\n",
   	   size_B, align_B, pte_kind, tile_mode, flags);
   }
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   if (dev->base.pdev->debug_flags & NVK_DEBUG_FORCE_GART) {
      flags &= ~(NVKMD_MEM_LOCAL | NVKMD_MEM_VRAM);
      flags |= NVKMD_MEM_GART;
   }
   /* Memory the processor maps used to become system memory whatever was
    * asked for, so the "video memory the processor can write" type and NVK's
    * own mapped tables (descriptors, events) were really in system memory and
    * every GPU access to them crossed the bus (airos-works/haiku#108). Video
    * memory can be mapped through BAR1 instead, write-combined, as long as
    * BAR1 has room; it is shared by every process, so check what is left.
    */
   bool bar1 = false;
   if (flags & NVKMD_MEM_CAN_MAP) {
      if ((flags & (NVKMD_MEM_LOCAL | NVKMD_MEM_VRAM)) &&
          !(flags & NVKMD_MEM_SHARED) &&
          nvkmd_nvrm_bar1_has_room(pdev, &rm, align64(size_B, 64 * 1024))) {
         bar1 = true;
         flags &= ~NVKMD_MEM_GART;
      } else {
         flags &= ~(NVKMD_MEM_LOCAL | NVKMD_MEM_VRAM);
         flags |= NVKMD_MEM_GART;
      }
   }

   const uint32_t mem_align_B = _dev->pdev->bind_align_B;
   size_B = align64(size_B, mem_align_B);

   assert(util_is_power_of_two_or_zero64(align_B));
   const uint64_t va_align_B = MAX2(mem_align_B, align_B);

   enum nvkmd_va_flags va_flags = NVKMD_VA_GART;

	bool isSystemMem = (flags & NVKMD_MEM_GART) != 0;
	NvU32 hClass = isSystemMem ?
		NV01_MEMORY_SYSTEM :
		NV01_MEMORY_LOCAL_USER;

	/* RM refuses big pages in system memory that is not contiguous. */
	uint32_t pageSize_B = isSystemMem ? 4096 :
		nvkmd_nvrm_vidmem_page_size(size_B);
	const uint64_t asked_size_B = size_B;
	size_B = align64(size_B, pageSize_B);

	NV_MEMORY_ALLOCATION_PARAMS params = {
		.owner = pdev->hClient,
		.type = NVOS32_TYPE_IMAGE,
		.flags = (align_B != 0) ? NVOS32_ALLOC_FLAGS_ALIGNMENT_FORCE : 0,
		.attr = nvkmd_nvrm_page_size_attr(pageSize_B),
		.attr2 = nvkmd_nvrm_page_size_attr2(pageSize_B),
		.size = size_B,
		.alignment = MAX2(align_B, pageSize_B),
	};
	if (isSystemMem) {
		params.attr |= DRF_DEF(OS32, _ATTR, _LOCATION, _PCI);
		params.attr |= DRF_DEF(OS32, _ATTR, _COHERENCY, _CACHED);
	} else {
		params.attr |= DRF_DEF(OS32, _ATTR, _LOCATION, _VIDMEM);
		if (bar1)
			params.attr |= DRF_DEF(OS32, _ATTR, _COHERENCY, _WRITE_COMBINE);
		else
			params.attr |= DRF_DEF(OS32, _ATTR, _COHERENCY, _UNCACHED);
		params.flags |= NVOS32_ALLOC_FLAGS_PERSISTENT_VIDMEM;
	}

	if (isSystemMem && pte_kind == 0 && tile_mode == 0 &&
	    !(flags & NVKMD_MEM_SHARED) && nvkmd_nvrm_mem_cache_enabled()) {
		struct nvkmd_nvrm_mem *cached =
			mem_cache_take(dev, size_B, flags, va_align_B);
		if (cached != NULL) {
			*mem_out = &cached->base;
			return VK_SUCCESS;
		}
	}

	/* RM writes back into the parameters, so a retry starts from a copy. */
	const NV_MEMORY_ALLOCATION_PARAMS params0 = params;
	NV_MEMORY_ALLOCATION_PARAMS paramsLast = params;
	NvHandle hMemoryPhys = 0;
   NV_STATUS nvRes = nvRmApiAlloc(&rm, pdev->hDevice, &hMemoryPhys, hClass, &params);
   while (nvRes != NV_OK && pageSize_B > 4096) {
      /* No free run of whole big or huge pages: smaller pages may fit. */
      pageSize_B = pageSize_B > (64u << 10) && asked_size_B >= (64u << 10) ?
                   64u << 10 : 4096;
      size_B = align64(asked_size_B, pageSize_B);
      params = params0;
      params.attr = (params.attr & ~DRF_SHIFTMASK(NVOS32_ATTR_PAGE_SIZE)) |
                    nvkmd_nvrm_page_size_attr(pageSize_B);
      params.attr2 = (params.attr2 & ~DRF_SHIFTMASK(NVOS32_ATTR2_PAGE_SIZE_HUGE)) |
                     nvkmd_nvrm_page_size_attr2(pageSize_B);
      params.size = size_B;
      params.alignment = MAX2(align_B, pageSize_B);
      hMemoryPhys = 0;
      nvRes = nvRmApiAlloc(&rm, pdev->hDevice, &hMemoryPhys, hClass, &params);
   }
   if (nvRes != NV_OK && bar1) {
      /* Video memory is full: system memory still works, only slower. */
      flags &= ~(NVKMD_MEM_LOCAL | NVKMD_MEM_VRAM);
      flags |= NVKMD_MEM_GART;
      isSystemMem = true;
      hClass = NV01_MEMORY_SYSTEM;
      params = params0;
      params.attr = DRF_DEF(OS32, _ATTR, _PAGE_SIZE, _4KB) |
                    DRF_DEF(OS32, _ATTR, _LOCATION, _PCI) |
                    DRF_DEF(OS32, _ATTR, _COHERENCY, _CACHED);
      params.attr2 = 0;
      params.flags &= ~NVOS32_ALLOC_FLAGS_PERSISTENT_VIDMEM;
      size_B = asked_size_B;
      params.size = size_B;
      params.alignment = align_B;
      pageSize_B = 4096;
      paramsLast = params;
      hMemoryPhys = 0;
      nvRes = nvRmApiAlloc(&rm, pdev->hDevice, &hMemoryPhys, hClass, &params);
   }
   if (nvRes != NV_OK && isSystemMem && !list_is_empty(&dev->memCache)) {
      /* What the cache holds counts against the floor of free memory that
       * nvidia_rm keeps for the rest of the system (airos-works/haiku#105). */
      nvkmd_nvrm_mem_cache_drain(dev);
      params = paramsLast;
      hMemoryPhys = 0;
      nvRes = nvRmApiAlloc(&rm, pdev->hDevice, &hMemoryPhys, hClass, &params);
   }
   if (nvRes != NV_OK) {
      fprintf(stderr, "[!] nvRes: %#x (mem alloc: size %#" PRIx64 " align %#" PRIx64 " sysmem %d flags %#x)\n", nvRes, (uint64_t)size_B, (uint64_t)align_B, isSystemMem, (unsigned)flags);
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;
   }

   VkResult result =
      create_mem_or_close_bo(dev, log_obj, flags, hMemoryPhys, size_B,
                             va_flags, pte_kind, va_align_B, pageSize_B,
                             mem_out);
   if (result == VK_SUCCESS && isSystemMem && pte_kind == 0 &&
       tile_mode == 0 && !(flags & NVKMD_MEM_SHARED))
      nvkmd_nvrm_mem(*mem_out)->cacheable = true;
   return result;
}

// Wrap memory the application already owns in a resman memory object, so the
// GPU can read and write it in place: what an application's window bitmap
// needs if the GPU is to draw the frame straight into it.
VkResult
nvkmd_nvrm_import_host_ptr(struct nvkmd_dev *_dev,
                           struct vk_object_base *log_obj,
                           void *ptr, uint64_t size_B,
                           struct nvkmd_mem **mem_out)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   if (((uintptr_t)ptr & (NVKMD_NVRM_HOST_PTR_ALIGNMENT - 1)) != 0)
      return vk_error(log_obj, VK_ERROR_INVALID_EXTERNAL_HANDLE);

   const uint64_t aligned_size_B = align64(size_B, NVKMD_NVRM_HOST_PTR_ALIGNMENT);

   /* Resman needs a handle for the object; it does not make one up for an
    * OS descriptor, so keep a counter of our own.
    */
   static uint32_t next_handle = 0;
   const NvHandle hMemory = 0xdea70000 | (p_atomic_inc_return(&next_handle) & 0xffff);

   NV_STATUS nvRes = nvRmApiAllocOsDescriptor(&rm, pdev->hDevice, hMemory,
                                              ptr, aligned_size_B);
   if (nvRes != NV_OK) {
      fprintf(stderr, "nvrm: import of %p (%" PRIu64 " bytes) refused: %#x\n",
              ptr, aligned_size_B, nvRes);
      return vk_errorf(log_obj, VK_ERROR_INVALID_EXTERNAL_HANDLE,
                       "resman refused to describe %p: %#x", ptr, nvRes);
   }

   return create_mem_or_close_bo(dev, log_obj, NVKMD_MEM_GART, hMemory,
                                 aligned_size_B, NVKMD_VA_GART, 0,
                                 NVKMD_NVRM_HOST_PTR_ALIGNMENT, 4096, mem_out);
}

// Take hold of the screen's frame buffer, which the accelerant publishes
// through the driver and shares with anyone who asks. It is a linear surface
// in video memory, so a frame copied there never crosses the bus.
VkResult
nvkmd_nvrm_import_scanout(struct nvkmd_dev *_dev,
                          struct vk_object_base *log_obj,
                          struct nvkmd_scanout_info *info_out,
                          struct nvkmd_mem **mem_out)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   nv_haiku_scanout_info info;
   if (ioctl(rm.fd, NV_HAIKU_BASE + NV_HAIKU_GET_SCANOUT, &info,
             sizeof(info)) < 0)
      return vk_errorf(log_obj, VK_ERROR_INITIALIZATION_FAILED,
                       "no frame buffer has been published");

   NvHandle hMemory = 0;
   NV_STATUS nvRes = nvRmApiDupObject(&rm, pdev->hDevice, info.client,
                                      info.memory, &hMemory);
   if (nvRes != NV_OK)
      return vk_errorf(log_obj, VK_ERROR_INITIALIZATION_FAILED,
                       "resman refused to share the frame buffer: %#x", nvRes);

   info_out->width = info.width;
   info_out->height = info.height;
   info_out->row_pitch_B = info.bytes_per_row;
   info_out->size_B = info.size;

   /* A pitch surface, so the default (pitch) page kind is right. */
   return create_mem_or_close_bo(dev, log_obj, NVKMD_MEM_LOCAL, hMemory,
                                 info.size, NVKMD_VA_GART, 0 /* pte_kind */,
                                 0 /* va_align_B */,
                                 4096 /* bound whole: RM's page size */,
                                 mem_out);
}

VkResult
nvkmd_nvrm_import_dma_buf(struct nvkmd_dev *_dev,
                             struct vk_object_base *log_obj,
                             int fd, struct nvkmd_mem **mem_out)
{
   return vk_errorf(log_obj, VK_ERROR_UNKNOWN, "nvkmd_nvrm_import_dma_buf: not implemented");
}

/* Gives the memory back to RM. */
static void
nvkmd_nvrm_mem_release(struct nvkmd_nvrm_mem *mem)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(mem->base.dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   if (mem->idleMapping != NULL) {
      nvRmApiUnmapMemory(&rm, pdev->hSubdevice, mem->hMemoryPhys, 0,
                         mem->idleMapping);
      free(mem->idleMapping);
   }
   nvkmd_va_free(mem->base.va);
   nvRmApiFree(&rm, mem->hMemoryPhys);
   FREE(mem);
}

static void
nvkmd_nvrm_mem_free(struct nvkmd_mem *_mem)
{
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(mem->base.dev);

   if (!mem_cache_put(dev, mem))
      nvkmd_nvrm_mem_release(mem);
}

static VkResult
nvkmd_nvrm_mem_map(struct nvkmd_mem *_mem,
                      struct vk_object_base *log_obj,
                      enum nvkmd_mem_map_flags map_flags,
                      void *fixed_addr,
                      void **map_out)
{
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_mem->dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   /* The mapping kept from the last unmap, if there is one. */
   NvRmApiMapping *mapping = NULL;
   if (!(map_flags & NVKMD_MEM_MAP_FIXED)) {
      simple_mtx_lock(&dev->mutex);
      mapping = mem->idleMapping;
      mem->idleMapping = NULL;
      simple_mtx_unlock(&dev->mutex);
   }

   if (mapping == NULL) {
      mapping = CALLOC_STRUCT(NvRmApiMapping);
      if (mapping == NULL)
         return vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);

      NV_STATUS nvRes = nvRmApiMapMemory(&rm, pdev->hSubdevice, mem->hMemoryPhys, 0, mem->base.size_B, mem->isSystemMem, 0, mapping);
      if (nvRes != NV_OK) {
         fprintf(stderr, "[!] nvRes: %#x (mem map: size %#" PRIx64 " sysmem %d)\n", nvRes, (uint64_t)mem->base.size_B, mem->isSystemMem);
         free(mapping);
         return vk_error(log_obj, VK_ERROR_OUT_OF_DEVICE_MEMORY);
      }
   }

   simple_mtx_lock(&dev->mutex);
   struct hash_entry *entry =
      _mesa_hash_table_insert(dev->mappings, mapping->address, mapping);
   simple_mtx_unlock(&dev->mutex);
   if (entry == NULL) {
   	nvRmApiUnmapMemory(&rm, pdev->hSubdevice, mem->hMemoryPhys, 0, mapping);
      free(mapping);
      return vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);
   }

   *map_out = mapping->address;
   return VK_SUCCESS;
}

static void
nvkmd_nvrm_mem_unmap(struct nvkmd_mem *_mem,
                        enum nvkmd_mem_map_flags flags,
                        void *map)
{
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_mem->dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   simple_mtx_lock(&dev->mutex);
   struct hash_entry *ent = _mesa_hash_table_search(dev->mappings, map);
   NvRmApiMapping *mapping = ent != NULL ? ent->data : NULL;
   if (ent != NULL)
      _mesa_hash_table_remove(dev->mappings, ent);
   /* Cacheable memory keeps one mapping for the next map, and for clearing
    * the memory should it be reused (mem_cache_take()). */
   const bool keep = mapping != NULL && mem->cacheable &&
                     mem->idleMapping == NULL;
   if (keep)
      mem->idleMapping = mapping;
   simple_mtx_unlock(&dev->mutex);
   if (mapping == NULL) {
      fprintf(stderr, "[!] ent == NULL\n");
      return;
   }
   if (keep)
      return;

   NV_STATUS nvRes = nvRmApiUnmapMemory(&rm, pdev->hSubdevice, mem->hMemoryPhys, 0, mapping);
   if (nvRes != NV_OK) {
      fprintf(stderr, "[!] nvRes: %#x\n", nvRes);
   }

   free(mapping);
}

static VkResult
nvkmd_nvrm_mem_overmap(struct nvkmd_mem *_mem,
                          struct vk_object_base *log_obj,
                          enum nvkmd_mem_map_flags flags,
                          void *map)
{
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);

   fprintf(stderr, "nvkmd_nvrm_mem_overmap(%#x)\n", mem->hMemoryPhys);

   return VK_ERROR_UNKNOWN;
}

static VkResult
nvkmd_nvrm_mem_export_dma_buf(struct nvkmd_mem *_mem,
                                 struct vk_object_base *log_obj,
                                 int *fd_out)
{
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);

   return VK_ERROR_UNKNOWN;
}

static uint32_t
nvkmd_nvrm_mem_log_handle(struct nvkmd_mem *_mem)
{
   return 0;
}

const struct nvkmd_mem_ops nvkmd_nvrm_mem_ops = {
   .free = nvkmd_nvrm_mem_free,
   .map = nvkmd_nvrm_mem_map,
   .unmap = nvkmd_nvrm_mem_unmap,
   .overmap = nvkmd_nvrm_mem_overmap,
   .export_dma_buf = nvkmd_nvrm_mem_export_dma_buf,
   .log_handle = nvkmd_nvrm_mem_log_handle,
};
