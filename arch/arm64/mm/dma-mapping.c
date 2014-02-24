/*
 * SWIOTLB-based DMA API implementation
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/gfp.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/vmalloc.h>
#include <linux/swiotlb.h>
#include <linux/io.h>

#include <asm/cacheflush.h>

struct dma_map_ops *dma_ops;
EXPORT_SYMBOL(dma_ops);

static void *arm64_swiotlb_alloc_coherent(struct device *dev, size_t size,
					  dma_addr_t *dma_handle, gfp_t flags,
					  struct dma_attrs *attrs)
{
	if (dev == NULL) {
		WARN(1, "Use an actual device structure for DMA allocation\n");
		return NULL;
	}

	if (IS_ENABLED(CONFIG_ZONE_DMA32) &&
	    dev->coherent_dma_mask <= DMA_BIT_MASK(32))
		flags |= GFP_DMA32;
	if (IS_ENABLED(CONFIG_CMA)) {
		unsigned long pfn;

		size = PAGE_ALIGN(size);
		pfn = dma_alloc_from_contiguous(dev, size >> PAGE_SHIFT,
							get_order(size));
		if (!pfn)
			return NULL;

		*dma_handle = phys_to_dma(dev, __pfn_to_phys(pfn));
		return page_address(pfn_to_page(pfn));
	} else {
		return swiotlb_alloc_coherent(dev, size, dma_handle, flags);
	}
}

static void arm64_swiotlb_free_coherent(struct device *dev, size_t size,
					void *vaddr, dma_addr_t dma_handle,
					struct dma_attrs *attrs)
{
	if (dev == NULL) {
		WARN(1, "Use an actual device structure for DMA allocation\n");
		return;
	}

	if (IS_ENABLED(CONFIG_CMA)) {
		phys_addr_t paddr = dma_to_phys(dev, dma_handle);

		dma_release_from_contiguous(dev,
					__phys_to_pfn(paddr),
					size >> PAGE_SHIFT);
	} else {
		swiotlb_free_coherent(dev, size, vaddr, dma_handle);
	}
}

static pgprot_t __get_dma_pgprot(pgprot_t prot, struct dma_attrs *attrs)
{
	if (dma_get_attr(DMA_ATTR_WRITE_COMBINE, attrs))
		prot = pgprot_writecombine(prot);
	/* if non-consistent just pass back what was given */
	else if (!dma_get_attr(DMA_ATTR_NON_CONSISTENT, attrs))
		prot = pgprot_dmacoherent(prot);

	return prot;
}

static void *arm64_swiotlb_alloc_noncoherent(struct device *dev, size_t size,
					     dma_addr_t *dma_handle, gfp_t flags,
					     struct dma_attrs *attrs)
{
	struct page *page, **map;
	void *ptr, *coherent_ptr;
	int order, i;
	pgprot_t prot = __get_dma_pgprot(pgprot_default, attrs);

	size = PAGE_ALIGN(size);
	order = get_order(size);

	ptr = arm64_swiotlb_alloc_coherent(dev, size, dma_handle, flags, attrs);
	if (!ptr)
		goto no_mem;
	map = kmalloc(sizeof(struct page *) << order, flags & ~GFP_DMA);
	if (!map)
		goto no_map;

	/* remove any dirty cache lines on the kernel alias */
	__dma_flush_range(ptr, ptr + size);

	/* create a coherent mapping */
	page = virt_to_page(ptr);
	for (i = 0; i < (size >> PAGE_SHIFT); i++)
		map[i] = page + i;
	coherent_ptr = vmap(map, size >> PAGE_SHIFT, VM_MAP, prot);
	kfree(map);
	if (!coherent_ptr)
		goto no_map;

	return coherent_ptr;

no_map:
	swiotlb_free_coherent(dev, size, ptr, *dma_handle);
no_mem:
	*dma_handle = ~0;
	return NULL;
}

static void arm64_swiotlb_free_noncoherent(struct device *dev, size_t size,
					   void *vaddr, dma_addr_t dma_handle,
					   struct dma_attrs *attrs)
{
	void *swiotlb_addr = phys_to_virt(dma_to_phys(dev, dma_handle));

	vunmap(vaddr);
	swiotlb_free_coherent(dev, size, swiotlb_addr, dma_handle);
}

static dma_addr_t arm64_swiotlb_map_page(struct device *dev,
					 struct page *page,
					 unsigned long offset, size_t size,
					 enum dma_data_direction dir,
					 struct dma_attrs *attrs)
{
	dma_addr_t dev_addr;

	dev_addr = swiotlb_map_page(dev, page, offset, size, dir, attrs);
	__dma_map_area(phys_to_virt(dma_to_phys(dev, dev_addr)), size, dir);

	return dev_addr;
}


static void arm64_swiotlb_unmap_page(struct device *dev, dma_addr_t dev_addr,
				     size_t size, enum dma_data_direction dir,
				     struct dma_attrs *attrs)
{
	__dma_unmap_area(phys_to_virt(dma_to_phys(dev, dev_addr)), size, dir);
	swiotlb_unmap_page(dev, dev_addr, size, dir, attrs);
}

static int arm64_swiotlb_map_sg_attrs(struct device *dev,
				      struct scatterlist *sgl, int nelems,
				      enum dma_data_direction dir,
				      struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i, ret;

	ret = swiotlb_map_sg_attrs(dev, sgl, nelems, dir, attrs);
	for_each_sg(sgl, sg, ret, i)
		__dma_map_area(phys_to_virt(dma_to_phys(dev, sg->dma_address)),
			       sg->length, dir);

	return ret;
}

static void arm64_swiotlb_unmap_sg_attrs(struct device *dev,
					 struct scatterlist *sgl, int nelems,
					 enum dma_data_direction dir,
					 struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nelems, i)
		__dma_unmap_area(phys_to_virt(dma_to_phys(dev, sg->dma_address)),
				 sg->length, dir);
	swiotlb_unmap_sg_attrs(dev, sgl, nelems, dir, attrs);
}

static void arm64_swiotlb_sync_single_for_cpu(struct device *dev,
					      dma_addr_t dev_addr,
					      size_t size,
					      enum dma_data_direction dir)
{
	__dma_unmap_area(phys_to_virt(dma_to_phys(dev, dev_addr)), size, dir);
	swiotlb_sync_single_for_cpu(dev, dev_addr, size, dir);
}

static void arm64_swiotlb_sync_single_for_device(struct device *dev,
						 dma_addr_t dev_addr,
						 size_t size,
						 enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dev_addr, size, dir);
	__dma_map_area(phys_to_virt(dma_to_phys(dev, dev_addr)), size, dir);
}

static void arm64_swiotlb_sync_sg_for_cpu(struct device *dev,
					  struct scatterlist *sgl, int nelems,
					  enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nelems, i)
		__dma_unmap_area(phys_to_virt(dma_to_phys(dev, sg->dma_address)),
				 sg->length, dir);
	swiotlb_sync_sg_for_cpu(dev, sgl, nelems, dir);
}

static void arm64_swiotlb_sync_sg_for_device(struct device *dev,
					     struct scatterlist *sgl,
					     int nelems,
					     enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	swiotlb_sync_sg_for_device(dev, sgl, nelems, dir);
	for_each_sg(sgl, sg, nelems, i)
		__dma_map_area(phys_to_virt(dma_to_phys(dev, sg->dma_address)),
			       sg->length, dir);
}

static void *arm64_dma_remap(struct device *dev, void *cpu_addr,
			dma_addr_t handle, size_t size,
			struct dma_attrs *attrs)
{
	struct page *page = phys_to_page(dma_to_phys(dev, handle));
	pgprot_t prot = __get_dma_pgprot(PAGE_KERNEL, attrs);
	unsigned long offset = handle & ~PAGE_MASK;
	struct vm_struct *area;
	unsigned long addr;

	size = PAGE_ALIGN(size + offset);

	/*
	 * DMA allocation can be mapped to user space, so lets
	 * set VM_USERMAP flags too.
	 */
	area = get_vm_area(size, VM_USERMAP);
	if (!area)
		return NULL;

	addr = (unsigned long)area->addr;
	area->phys_addr = __pfn_to_phys(page_to_pfn(page));

	if (ioremap_page_range(addr, addr + size, area->phys_addr, prot)) {
		vunmap((void *)addr);
		return NULL;
	}
	return (void *)addr + offset;
}

static void arm64_dma_unremap(struct device *dev, void *remapped_addr,
				size_t size)
{
	struct vm_struct *area;

	remapped_addr = (void *)((unsigned long)remapped_addr & PAGE_MASK);

	area = find_vm_area(remapped_addr);
	if (!area) {
		WARN(1, "trying to free invalid coherent area: %p\n",
			remapped_addr);
		return;
	}
	vunmap(remapped_addr);
}

struct dma_map_ops noncoherent_swiotlb_dma_ops = {
	.alloc = arm64_swiotlb_alloc_noncoherent,
	.free = arm64_swiotlb_free_noncoherent,
	.map_page = arm64_swiotlb_map_page,
	.unmap_page = arm64_swiotlb_unmap_page,
	.map_sg = arm64_swiotlb_map_sg_attrs,
	.unmap_sg = arm64_swiotlb_unmap_sg_attrs,
	.sync_single_for_cpu = arm64_swiotlb_sync_single_for_cpu,
	.sync_single_for_device = arm64_swiotlb_sync_single_for_device,
	.sync_sg_for_cpu = arm64_swiotlb_sync_sg_for_cpu,
	.sync_sg_for_device = arm64_swiotlb_sync_sg_for_device,
	.dma_supported = swiotlb_dma_supported,
	.mapping_error = swiotlb_dma_mapping_error,
	.remap = arm64_dma_remap,
	.unremap = arm64_dma_unremap,
};
EXPORT_SYMBOL(noncoherent_swiotlb_dma_ops);

struct dma_map_ops coherent_swiotlb_dma_ops = {
	.alloc = arm64_swiotlb_alloc_coherent,
	.free = arm64_swiotlb_free_coherent,
	.map_page = swiotlb_map_page,
	.unmap_page = swiotlb_unmap_page,
	.map_sg = swiotlb_map_sg_attrs,
	.unmap_sg = swiotlb_unmap_sg_attrs,
	.sync_single_for_cpu = swiotlb_sync_single_for_cpu,
	.sync_single_for_device = swiotlb_sync_single_for_device,
	.sync_sg_for_cpu = swiotlb_sync_sg_for_cpu,
	.sync_sg_for_device = swiotlb_sync_sg_for_device,
	.dma_supported = swiotlb_dma_supported,
	.mapping_error = swiotlb_dma_mapping_error,
	.remap = arm64_dma_remap,
	.unremap = arm64_dma_unremap,
};
EXPORT_SYMBOL(coherent_swiotlb_dma_ops);

void __init arm64_swiotlb_init(void)
{
	dma_ops = &noncoherent_swiotlb_dma_ops;
	swiotlb_init(1);
}

#define PREALLOC_DMA_DEBUG_ENTRIES	4096

static int __init dma_debug_do_init(void)
{
	dma_debug_init(PREALLOC_DMA_DEBUG_ENTRIES);
	return 0;
}
fs_initcall(dma_debug_do_init);
