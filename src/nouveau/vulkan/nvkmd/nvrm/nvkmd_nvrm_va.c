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

   nvRmApiFree(&rm, va->hMemoryVirt);

   FREE(va);
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
   NvU64 dmaOffset = va_offset_B;
   NV_STATUS nvRes = nvRmApiMapMemoryDma(&rm, pdev->hDevice, va->hMemoryVirt, mem->hMemoryPhys, mem_offset_B, range_B, gpuMapFlags, &dmaOffset);
   if (nvRes != NV_OK) {
      fprintf(stderr, "[!] nvRes: %#x (VA bind: va %#" PRIx64 "+%#" PRIx64 " mem off %#" PRIx64 " range %#" PRIx64 " sysmem %d)\n", nvRes, va->base.addr, va_offset_B, mem_offset_B, range_B, mem->isSystemMem);
      return VK_ERROR_UNKNOWN;
   }
   if (dmaOffset != va->base.addr + va_offset_B) {
      fprintf(stderr, "[!] mapped at %#"PRIx64", expected %#"PRIx64"\n",
         (uint64_t)dmaOffset, va->base.addr + va_offset_B);
   }

   va->hMemoryPhys = mem->hMemoryPhys;

   return VK_SUCCESS;
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

   NV_STATUS nvRes = nvRmApiUnmapMemoryDma(&rm, pdev->hDevice, va->hMemoryVirt, va->hMemoryPhys, 0, va_offset_B);
   if (nvRes != NV_OK)
      return VK_ERROR_UNKNOWN;

   va->hMemoryPhys = 0;

   return VK_SUCCESS;
}

const struct nvkmd_va_ops nvkmd_nvrm_va_ops = {
   .free = nvkmd_nvrm_va_free,
   .bind_mem = nvkmd_nvrm_va_bind_mem,
   .unbind = nvkmd_nvrm_va_unbind,
};
