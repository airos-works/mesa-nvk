/*
 * Copyright © 2024 Collabora Ltd. and Red Hat Inc.
 * SPDX-License-Identifier: MIT
 */

#include "nvkmd_nvrm.h"

#include "util/bitscan.h"
#include "util/u_debug.h"
#include "util/u_memory.h"
#include "vk_log.h"

#include <inttypes.h>
#include <stdio.h>

#include "class/cl50a0.h" // NV50_MEMORY_VIRTUAL


VkResult
nvkmd_nvrm_alloc_va(struct nvkmd_dev *_dev,
                       struct vk_object_base *log_obj,
                       enum nvkmd_va_flags flags, uint8_t pte_kind,
                       uint64_t size_B, uint64_t align_B,
                       uint64_t fixed_addr, struct nvkmd_va **va_out)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);

   //fprintf(stderr, "nvkmd_nvrm_alloc_va(%#x)\n", pte_kind);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   struct nvkmd_nvrm_va *va = CALLOC_STRUCT(nvkmd_nvrm_va);
   if (va == NULL)
      return vk_error(log_obj, VK_ERROR_OUT_OF_HOST_MEMORY);

   const uint32_t min_align_B = _dev->pdev->bind_align_B;
   size_B = align64(size_B, min_align_B);

   assert(util_is_power_of_two_or_zero64(align_B));
   align_B = MAX2(align_B, min_align_B);

   assert((fixed_addr == 0) == !(flags & NVKMD_VA_ALLOC_FIXED));

   va->base.ops = &nvkmd_nvrm_va_ops;
   va->base.dev = &dev->base;
   va->base.flags = flags;
   va->base.pte_kind = pte_kind;
   va->base.size_B = size_B;
   simple_mtx_init(&va->mutex, mtx_plain);
   util_dynarray_init(&va->bindings, NULL);

	NvHandle hMemoryVirt = 0;
	NV_MEMORY_ALLOCATION_PARAMS params = {
		.owner = pdev->hClient,
		.type = NVOS32_TYPE_IMAGE,
		.flags =
			NVOS32_ALLOC_FLAGS_VIRTUAL |
			((align_B != 0) ? NVOS32_ALLOC_FLAGS_ALIGNMENT_FORCE : 0),
		.size = size_B,
		.alignment = align_B,
		.hVASpace = pdev->hVaSpace,
	};
	/* The unbound pages of a sparse resource read as zero: RM sparsifies
	 * the range at once, and again wherever a part of it is unmapped. */
	if (flags & NVKMD_VA_SPARSE)
		params.flags |= NVOS32_ALLOC_FLAGS_SPARSE;
	/* Resman derives the page kind of a virtual allocation from these
	 * attributes, so they have to describe exactly the kind NIL picked for the
	 * image.  The kind numbering changed with Turing, hence the two tables. */
	const bool preTuring = pdev->base.dev_info.cls_eng3d < 0xc597 /* TURING_A */;
	enum {
		KIND_OTHER,	/* colour or untyped memory, the kind only says how it tiles */
		KIND_Z16,
		KIND_S8,
		KIND_S8Z24,
		KIND_Z24S8,
		KIND_ZF32,
		KIND_ZF32_X24S8,
		KIND_PITCH,
		KIND_UNKNOWN,
	} kind = KIND_UNKNOWN;

	if (preTuring) {
		/* pascal/gp100/dev_mmu.h */
		switch (pte_kind) {
			case 0x00: kind = KIND_PITCH; break;
			case 0x01: kind = KIND_Z16; break;
			case 0x11: kind = KIND_S8Z24; break;
			case 0x2a: kind = KIND_S8; break;
			case 0x46: kind = KIND_Z24S8; break;
			case 0x7b: kind = KIND_ZF32; break;
			case 0xc3: kind = KIND_ZF32_X24S8; break;
			case 0xfe: kind = KIND_OTHER; break; /* GENERIC_16BX2: block linear colour */
		}
	} else {
		/* turing/tu102/dev_mmu.h */
		switch (pte_kind) {
			case 0x00: kind = KIND_PITCH; break;
			case 0x01: kind = KIND_Z16; break;
			case 0x02: kind = KIND_S8; break;
			case 0x03: kind = KIND_S8Z24; break;
			case 0x04: kind = KIND_ZF32_X24S8; break;
			case 0x05: kind = KIND_Z24S8; break;
			case 0x06: kind = KIND_OTHER; break; /* GENERIC_MEMORY */
		}
	}

	switch (kind) {
		case KIND_PITCH:
		case KIND_OTHER:
			if (kind == KIND_OTHER) {
				params.type = NVOS32_TYPE_IMAGE;
				params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			}
			break;
		case KIND_Z16:
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _16);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FIXED);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _Z16);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_S8:
			/* Resman picks the S8 kind from the attributes (DEPTH_8 with
			 * ZS_PACKING_S8) for either type, but the Pascal one refuses
			 * NVOS32_TYPE_STENCIL with NV_ERR_INVALID_ARGUMENT: every
			 * VK_FORMAT_S8_UINT image failed there, so a GL
			 * GL_STENCIL_INDEX8 framebuffer through zink was incomplete
			 * (Summit #197). The Turing heap treats both types alike. */
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _8);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FIXED);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _S8);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_S8Z24:
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _32);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FIXED);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _S8Z24);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_Z24S8:
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _32);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FIXED);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _Z24S8);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_ZF32:
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _32);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FLOAT);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _Z32);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_ZF32_X24S8:
			params.type = NVOS32_TYPE_DEPTH;
			params.attr |= DRF_DEF(OS32, _ATTR, _DEPTH, _64);
			params.attr |= DRF_DEF(OS32, _ATTR, _FORMAT, _BLOCK_LINEAR);
			params.attr |= DRF_DEF(OS32, _ATTR, _Z_TYPE, _FLOAT);
			params.attr |= DRF_DEF(OS32, _ATTR, _ZS_PACKING, _Z32_X24S8);
			params.attr |= DRF_DEF(OS32, _ATTR, _COMPR, _NONE);
			break;
		case KIND_UNKNOWN:
			fprintf(stderr, "[!] unsupported pte_kind(%#x)\n", pte_kind);
			return VK_ERROR_UNKNOWN;
	}

   NV_STATUS nvRes = nvRmApiAlloc(&rm, pdev->hDevice, &hMemoryVirt, NV50_MEMORY_VIRTUAL, &params);
   if (nvRes != NV_OK) {
      fprintf(stderr, "[!] nvRes: %#x (VA alloc: size %#" PRIx64 " align %#" PRIx64 " kind %#x type %u flags %#x attr %#x attr2 %#x fixed %#" PRIx64 " vaflags %#x)\n",
         nvRes, (uint64_t)size_B, (uint64_t)align_B, pte_kind, params.type, params.flags, params.attr, params.attr2, (uint64_t)fixed_addr, (unsigned)flags);
   	nvkmd_va_free(&va->base);
      return VK_ERROR_UNKNOWN;
   }
   if (pte_kind != 0 && params.format != pte_kind) {
      fprintf(stderr, "[!] params.format(%#x) != pte_kind(%#x)\n", params.format, pte_kind);
      nvkmd_va_free(&va->base);
      return VK_ERROR_UNKNOWN;
   }
   va->hMemoryVirt = hMemoryVirt;
   va->base.addr = params.offset;

   *va_out = &va->base;

   return VK_SUCCESS;
}

static void
nvkmd_nvrm_va_free(struct nvkmd_va *_va)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_va->dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct nvkmd_nvrm_va *va = nvkmd_nvrm_va(_va);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   /* Freeing the range unmaps whatever is still bound in it. */
   nvRmApiFree(&rm, va->hMemoryVirt);

   util_dynarray_fini(&va->bindings);
   simple_mtx_destroy(&va->mutex);
   FREE(va);
}

/* Unmaps [va_offset_B, va_offset_B + range_B) of whatever is bound there; RM
 * splits a mapping that is unmapped in part. Called with va->mutex held. */
static VkResult
va_unbind_locked(struct nvkmd_nvrm_va *va, struct NvRmApi *rm,
                 NvHandle hDevice, uint64_t va_offset_B, uint64_t range_B)
{
   const uint64_t end_B = va_offset_B + range_B;
   VkResult result = VK_SUCCESS;

   unsigned i = 0;
   while (i < util_dynarray_num_elements(&va->bindings,
                                         struct nvkmd_nvrm_va_binding)) {
      struct nvkmd_nvrm_va_binding *b =
         util_dynarray_element(&va->bindings, struct nvkmd_nvrm_va_binding, i);
      const uint64_t b_end_B = b->va_offset_B + b->range_B;
      if (b_end_B <= va_offset_B || b->va_offset_B >= end_B) {
         i++;
         continue;
      }

      const uint64_t lo_B = MAX2(b->va_offset_B, va_offset_B);
      const uint64_t hi_B = MIN2(b_end_B, end_B);
      NV_STATUS nvRes = nvRmApiUnmapMemoryDma(rm, hDevice, va->hMemoryVirt,
                                              b->hMemoryPhys, 0,
                                              va->base.addr + lo_B,
                                              hi_B - lo_B);
      if (nvRes != NV_OK) {
         fprintf(stderr, "[!] nvRes: %#x (VA unbind: va %#" PRIx64 "+%#" PRIx64 " range %#" PRIx64 ")\n",
                 nvRes, va->base.addr, lo_B, hi_B - lo_B);
         result = VK_ERROR_UNKNOWN;
      }

      struct nvkmd_nvrm_va_binding left = *b, right = *b;
      left.range_B = lo_B - b->va_offset_B;
      right.va_offset_B = hi_B;
      right.range_B = b_end_B - hi_B;

      /* Replace the entry with the last one, then add what is left of it
       * on either side; those lie outside the range, so looking at them
       * again does nothing. */
      *b = util_dynarray_top(&va->bindings, struct nvkmd_nvrm_va_binding);
      (void)util_dynarray_pop(&va->bindings, struct nvkmd_nvrm_va_binding);
      if (left.range_B > 0)
         util_dynarray_append(&va->bindings, struct nvkmd_nvrm_va_binding, left);
      if (right.range_B > 0)
         util_dynarray_append(&va->bindings, struct nvkmd_nvrm_va_binding, right);
   }

   return result;
}

static VkResult
nvkmd_nvrm_va_bind_mem(struct nvkmd_va *_va,
                          struct vk_object_base *log_obj,
                          uint64_t va_offset_B,
                          struct nvkmd_mem *_mem,
                          uint64_t mem_offset_B,
                          uint64_t range_B)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_va->dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct nvkmd_nvrm_va *va = nvkmd_nvrm_va(_va);
   struct nvkmd_nvrm_mem *mem = nvkmd_nvrm_mem(_mem);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   NvU32 gpuMapFlags = 0;
   gpuMapFlags |= DRF_DEF(OS46, _FLAGS, _PAGE_KIND, _VIRTUAL);
   if (mem->isSystemMem) {
      gpuMapFlags |= DRF_DEF(OS46, _FLAGS, _CACHE_SNOOP, _ENABLE);
   } else {
      gpuMapFlags |= DRF_DEF(OS46, _FLAGS, _CACHE_SNOOP, _DISABLE);
   }

   const uint64_t va_addr = va->base.addr + va_offset_B;

   simple_mtx_lock(&va->mutex);

   /* A bind replaces whatever was bound there. */
   VkResult result = va_unbind_locked(va, &rm, pdev->hDevice, va_offset_B,
                                      range_B);
   if (result != VK_SUCCESS)
      goto out;

   NvU64 dmaOffset = va_offset_B;
   NV_STATUS nvRes = nvRmApiMapMemoryDma(&rm, pdev->hDevice, va->hMemoryVirt, mem->hMemoryPhys, mem_offset_B, range_B, gpuMapFlags, &dmaOffset);
   if (nvRes != NV_OK) {
      fprintf(stderr, "[!] nvRes: %#x (VA bind: va %#" PRIx64 "+%#" PRIx64 " mem off %#" PRIx64 " range %#" PRIx64 " sysmem %d)\n",
              nvRes, va->base.addr, va_offset_B, mem_offset_B, range_B,
              mem->isSystemMem);
      result = VK_ERROR_UNKNOWN;
      goto out;
   }

   struct nvkmd_nvrm_va_binding binding = {
      .va_offset_B = va_offset_B,
      .range_B = range_B,
      .hMemoryPhys = mem->hMemoryPhys,
   };
   util_dynarray_append(&va->bindings, struct nvkmd_nvrm_va_binding, binding);

   if (dmaOffset != va_addr) {
      fprintf(stderr, "[!] mapped at %#" PRIx64 ", expected %#" PRIx64 "\n",
              (uint64_t)dmaOffset, va_addr);
   }

out:
   simple_mtx_unlock(&va->mutex);
   return result;
}

static VkResult
nvkmd_nvrm_va_unbind(struct nvkmd_va *_va,
                        struct vk_object_base *log_obj,
                        uint64_t va_offset_B,
                        uint64_t range_B)
{
   struct nvkmd_nvrm_dev *dev = nvkmd_nvrm_dev(_va->dev);
   struct nvkmd_nvrm_pdev *pdev = nvkmd_nvrm_pdev(dev->base.pdev);
   struct nvkmd_nvrm_va *va = nvkmd_nvrm_va(_va);

   struct NvRmApi rm;
   nvkmd_nvrm_dev_api_ctl(pdev, &rm);

   simple_mtx_lock(&va->mutex);
   VkResult result = va_unbind_locked(va, &rm, pdev->hDevice, va_offset_B,
                                      range_B);
   simple_mtx_unlock(&va->mutex);
   return result;
}

const struct nvkmd_va_ops nvkmd_nvrm_va_ops = {
   .free = nvkmd_nvrm_va_free,
   .bind_mem = nvkmd_nvrm_va_bind_mem,
   .unbind = nvkmd_nvrm_va_unbind,
};
