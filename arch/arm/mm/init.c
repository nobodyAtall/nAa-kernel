/*
 *  linux/arch/arm/mm/init.c
 *
 *  Copyright (C) 1995-2005 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/swap.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/mman.h>
#include <linux/nodemask.h>
#include <linux/initrd.h>
#ifdef CONFIG_MEMORY_HOTPLUG
#include <linux/memory_hotplug.h>
#endif
#include <linux/sort.h>

#include <asm/mach-types.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/sizes.h>
#include <asm/tlb.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "mm.h"

static unsigned long phys_initrd_start __initdata = 0;
static unsigned long phys_initrd_size __initdata = 0;

static void __init early_initrd(char **p)
{
	unsigned long start, size;

	start = memparse(*p, p);
	if (**p == ',') {
		size = memparse((*p) + 1, p);

		phys_initrd_start = start;
		phys_initrd_size = size;
	}
}
__early_param("initrd=", early_initrd);

static int __init parse_tag_initrd(const struct tag *tag)
{
	printk(KERN_WARNING "ATAG_INITRD is deprecated; "
		"please update your bootloader.\n");
	phys_initrd_start = __virt_to_phys(tag->u.initrd.start);
	phys_initrd_size = tag->u.initrd.size;
	return 0;
}

__tagtable(ATAG_INITRD, parse_tag_initrd);

static int __init parse_tag_initrd2(const struct tag *tag)
{
	phys_initrd_start = tag->u.initrd.start;
	phys_initrd_size = tag->u.initrd.size;
	return 0;
}

__tagtable(ATAG_INITRD2, parse_tag_initrd2);

/*
 * This keeps memory configuration data used by a couple memory
 * initialization functions, as well as show_mem() for the skipping
 * of holes in the memory map.  It is populated by arm_add_memory().
 */
struct meminfo meminfo;

void show_mem(void)
{
	int free = 0, total = 0, reserved = 0;
	int shared = 0, cached = 0, slab = 0, node, i;
	struct meminfo * mi = &meminfo;

	printk("Mem-info:\n");
	show_free_areas();
	for_each_online_node(node) {
		pg_data_t *n = NODE_DATA(node);
		struct page *map = pgdat_page_nr(n, 0) - n->node_start_pfn;

		for_each_nodebank (i,mi,node) {
			struct membank *bank = &mi->bank[i];
			unsigned int pfn1, pfn2;
			struct page *page, *end;

			pfn1 = bank_pfn_start(bank);
			pfn2 = bank_pfn_end(bank);

			page = map + pfn1;
			end  = map + pfn2;

			do {
				total++;
				if (PageReserved(page))
					reserved++;
				else if (PageSwapCache(page))
					cached++;
				else if (PageSlab(page))
					slab++;
				else if (!page_count(page))
					free++;
				else
					shared += page_count(page) - 1;
				page++;
			} while (page < end);
		}
	}

	printk("%d pages of RAM\n", total);
	printk("%d free pages\n", free);
	printk("%d reserved pages\n", reserved);
	printk("%d slab pages\n", slab);
	printk("%d pages shared\n", shared);
	printk("%d pages swap cached\n", cached);
}

/*
 * FIXME: We really want to avoid allocating the bootmap bitmap
 * over the top of the initrd.  Hopefully, this is located towards
 * the start of a bank, so if we allocate the bootmap bitmap at
 * the end, we won't clash.
 */
static unsigned int __init
find_bootmap_pfn(int node, struct meminfo *mi, unsigned int bootmap_pages)
{
	unsigned int start_pfn, i, bootmap_pfn;

	start_pfn   = PAGE_ALIGN(__pa(_end)) >> PAGE_SHIFT;
	bootmap_pfn = 0;

	for_each_nodebank(i, mi, node) {
		struct membank *bank = &mi->bank[i];
		unsigned int start, end;

		start = bank_pfn_start(bank);
		end   = bank_pfn_end(bank);

		if (end < start_pfn)
			continue;

		if (start < start_pfn)
			start = start_pfn;

		if (end <= start)
			continue;

		if (end - start >= bootmap_pages) {
			bootmap_pfn = start;
			break;
		}
	}

	if (bootmap_pfn == 0)
		BUG();

	return bootmap_pfn;
}

static int __init check_initrd(struct meminfo *mi)
{
	int initrd_node = -2;
#ifdef CONFIG_BLK_DEV_INITRD
	unsigned long end = phys_initrd_start + phys_initrd_size;

	/*
	 * Make sure that the initrd is within a valid area of
	 * memory.
	 */
	if (phys_initrd_size) {
		unsigned int i;

		initrd_node = -1;

		for (i = 0; i < mi->nr_banks; i++) {
			struct membank *bank = &mi->bank[i];
			if (bank_phys_start(bank) <= phys_initrd_start &&
			    end <= bank_phys_end(bank))
				initrd_node = bank->node;
		}
	}

	if (initrd_node == -1) {
		printk(KERN_ERR "INITRD: 0x%08lx+0x%08lx extends beyond "
		       "physical memory - disabling initrd\n",
		       phys_initrd_start, phys_initrd_size);
		phys_initrd_start = phys_initrd_size = 0;
	}
#endif

	return initrd_node;
}

static inline void map_memory_bank(struct membank *bank)
{
#ifdef CONFIG_MMU
	struct map_desc map;

	map.pfn = bank_pfn_start(bank);
	map.virtual = __phys_to_virt(bank_phys_start(bank));
	map.length = bank_phys_size(bank);
	map.type = MT_MEMORY;

	create_mapping(&map);
#endif
}

static unsigned long __init bootmem_init_node(int node, struct meminfo *mi)
{
	unsigned long start_pfn, end_pfn, boot_pfn;
	unsigned int boot_pages;
	pg_data_t *pgdat;
	int i;

	start_pfn = -1UL;
	end_pfn = 0;

	/*
	 * Calculate the pfn range, and map the memory banks for this node.
	 */
	for_each_nodebank(i, mi, node) {
		struct membank *bank = &mi->bank[i];
		unsigned long start, end;

		start = bank_pfn_start(bank);
		end = bank_pfn_end(bank);

#if defined(CONFIG_FLATMEM) && !defined(CONFIG_HOLES_IN_ZONE)
		/*
		 * The VM code assumes that hole end addresses are aligned if
		 * CONFIG_HOLES_IN_ZONE is not enabled. This results in
		 * panics since we free unused memmap entries on ARM.
		 * This check shouldn't be necessary for the last bank's end
		 * address, since the VM code accounts for the total zone size.
		 */
		if ((i < (mi->nr_banks - 1)) &&
		    (end & (MAX_ORDER_NR_PAGES - 1))) {
			pr_err("Memory bank[%d] not aligned to 0x%x bytes.\n"
			       "\tMake bank end address align with MAX_ORDER\n"
			       "\tor enable option CONFIG_HOLES_IN_ZONE.\n",
			       i, __pfn_to_phys(MAX_ORDER_NR_PAGES));
			BUG();
		}
#endif

		if (start_pfn > start)
			start_pfn = start;
		if (end_pfn < end)
			end_pfn = end;

		map_memory_bank(bank);
	}

	/*
	 * If there is no memory in this node, ignore it.
	 */
	if (end_pfn == 0)
		return end_pfn;

	/*
	 * Allocate the bootmem bitmap page.
	 */
	boot_pages = bootmem_bootmap_pages(end_pfn - start_pfn);
	boot_pfn = find_bootmap_pfn(node, mi, boot_pages);

	/*
	 * Initialise the bootmem allocator for this node, handing the
	 * memory banks over to bootmem.
	 */
	node_set_online(node);
	pgdat = NODE_DATA(node);
	init_bootmem_node(pgdat, boot_pfn, start_pfn, end_pfn);

	for_each_nodebank(i, mi, node) {
		struct membank *bank = &mi->bank[i];
		free_bootmem_node(pgdat, bank_phys_start(bank), bank_phys_size(bank));
		memory_present(node, bank_pfn_start(bank), bank_pfn_end(bank));
	}

	/*
	 * Reserve the bootmem bitmap for this node.
	 */
	reserve_bootmem_node(pgdat, boot_pfn << PAGE_SHIFT,
			     boot_pages << PAGE_SHIFT, BOOTMEM_DEFAULT);

	return end_pfn;
}

static void __init bootmem_reserve_initrd(int node)
{
#ifdef CONFIG_BLK_DEV_INITRD
	pg_data_t *pgdat = NODE_DATA(node);
	int res;

	res = reserve_bootmem_node(pgdat, phys_initrd_start,
			     phys_initrd_size, BOOTMEM_EXCLUSIVE);

	if (res == 0) {
		initrd_start = __phys_to_virt(phys_initrd_start);
		initrd_end = initrd_start + phys_initrd_size;
	} else {
		printk(KERN_ERR
			"INITRD: 0x%08lx+0x%08lx overlaps in-use "
			"memory region - disabling initrd\n",
			phys_initrd_start, phys_initrd_size);
	}
#endif
}

static void __init bootmem_free_node(int node, struct meminfo *mi)
{
	unsigned long zone_size[MAX_NR_ZONES], zhole_size[MAX_NR_ZONES];
	unsigned long start_pfn, end_pfn;
	pg_data_t *pgdat = NODE_DATA(node);
	int i;

	start_pfn = pgdat->bdata->node_min_pfn;
	end_pfn = pgdat->bdata->node_low_pfn;

	/*
	 * initialise the zones within this node.
	 */
	memset(zone_size, 0, sizeof(zone_size));
	memset(zhole_size, 0, sizeof(zhole_size));

	/*
	 * The size of this node has already been determined.  If we need
	 * to do anything fancy with the allocation of this memory to the
	 * zones, now is the time to do it.
	 */
	zone_size[0] = end_pfn - start_pfn;

	/*
	 * For each bank in this node, calculate the size of the holes.
	 *  holes = node_size - sum(bank_sizes_in_node)
	 */
	zhole_size[0] = zone_size[0];
	for_each_nodebank(i, mi, node)
		zhole_size[0] -= bank_pfn_size(&mi->bank[i]);

	/*
	 * Adjust the sizes according to any special requirements for
	 * this machine type.
	 */
	arch_adjust_zones(node, zone_size, zhole_size);

	free_area_init_node(node, zone_size, start_pfn, zhole_size);
}

#ifdef CONFIG_MEMORY_HOTPLUG
static void map_reserved_memory(void)
{
	struct map_desc map;

	map.pfn = (movable_reserved_start >> PAGE_SHIFT);
	map.virtual = __phys_to_virt(movable_reserved_start);
	map.length = movable_reserved_size;
	map.type = MT_MEMORY;

	create_mapping(&map);
}
#endif

#ifndef CONFIG_SPARSEMEM
int pfn_valid(unsigned long pfn)
{
	struct meminfo *mi = &meminfo;
	unsigned int left = 0, right = mi->nr_banks;

	do {
		unsigned int mid = (right + left) / 2;
		struct membank *bank = &mi->bank[mid];

		if (pfn < bank_pfn_start(bank))
			right = mid;
		else if (pfn >= bank_pfn_end(bank))
			left = mid + 1;
		else
			return 1;
	} while (left < right);
	return 0;
}
EXPORT_SYMBOL(pfn_valid);
#endif

static int __init meminfo_cmp(const void *_a, const void *_b)
{
	const struct membank *a = _a, *b = _b;
	long cmp = bank_pfn_start(a) - bank_pfn_start(b);
	return cmp < 0 ? -1 : cmp > 0 ? 1 : 0;
}

void __init bootmem_init(void)
{
	struct meminfo *mi = &meminfo;
	unsigned long memend_pfn = 0;
	int node, initrd_node;

	sort(&mi->bank, mi->nr_banks, sizeof(mi->bank[0]), meminfo_cmp, NULL);

	/*
	 * Locate which node contains the ramdisk image, if any.
	 */
	initrd_node = check_initrd(mi);

	/*
	 * Run through each node initialising the bootmem allocator.
	 */
	for_each_node(node) {
		unsigned long end_pfn = bootmem_init_node(node, mi);

		/*
		 * Reserve any special node zero regions.
		 */
		if (node == 0)
			reserve_node_zero(NODE_DATA(node));

		/*
		 * If the initrd is in this node, reserve its memory.
		 */
		if (node == initrd_node)
			bootmem_reserve_initrd(node);

		/*
		 * Remember the highest memory PFN.
		 */
		if (end_pfn > memend_pfn)
			memend_pfn = end_pfn;
	}

	/*
	 * sparse_init() needs the bootmem allocator up and running.
	 */
	sparse_init();

	/*
	 * Now free memory in each node - free_area_init_node needs
	 * the sparse mem_map arrays initialized by sparse_init()
	 * for memmap_init_zone(), otherwise all PFNs are invalid.
	 */
	for_each_node(node)
		bootmem_free_node(node, mi);

#ifdef CONFIG_MEMORY_HOTPLUG
	if (movable_reserved_size) {
		memend_pfn = (movable_reserved_start + movable_reserved_size)
			>> PAGE_SHIFT;
		map_reserved_memory();
	}
#endif
	high_memory = __va((memend_pfn << PAGE_SHIFT) - 1) + 1;

	/*
	 * This doesn't seem to be used by the Linux memory manager any
	 * more, but is used by ll_rw_block.  If we can get rid of it, we
	 * also get rid of some of the stuff above as well.
	 *
	 * Note: max_low_pfn and max_pfn reflect the number of _pages_ in
	 * the system, not the maximum PFN.
	 */
	max_pfn = max_low_pfn = memend_pfn - PHYS_PFN_OFFSET;
}

static inline int free_area(unsigned long pfn, unsigned long end, char *s)
{
	unsigned int pages = 0, size = (end - pfn) << (PAGE_SHIFT - 10);

	for (; pfn < end; pfn++) {
		struct page *page = pfn_to_page(pfn);
		ClearPageReserved(page);
		init_page_count(page);
		__free_page(page);
		pages++;
	}

	if (size && s)
		printk(KERN_INFO "Freeing %s memory: %dK\n", s, size);

	return pages;
}

static inline void
free_memmap(int node, unsigned long start_pfn, unsigned long end_pfn)
{
	struct page *start_pg, *end_pg;
	unsigned long pg, pgend;

	/*
	 * Convert start_pfn/end_pfn to a struct page pointer.
	 */
	start_pg = pfn_to_page(start_pfn);
	end_pg = pfn_to_page(end_pfn);

	/*
	 * Convert to physical addresses, and
	 * round start upwards and end downwards.
	 */
	pg = PAGE_ALIGN(__pa(start_pg));
	pgend = __pa(end_pg) & PAGE_MASK;

	/*
	 * If there are free pages between these,
	 * free the section of the memmap array.
	 */
	if (pg < pgend)
		free_bootmem_node(NODE_DATA(node), pg, pgend - pg);
}

/*
 * The mem_map array can get very big.  Free the unused area of the memory map.
 */
static void __init free_unused_memmap_node(int node, struct meminfo *mi)
{
	unsigned long bank_start, prev_bank_end = 0;
	unsigned int i;

	/*
	 * This relies on each bank being in address order. The banks
	 * are sorted previously in bootmem_init().
	 */
	for_each_nodebank(i, mi, node) {
		struct membank *bank = &mi->bank[i];

		bank_start = bank_pfn_start(bank);
		if (bank_start < prev_bank_end) {
			printk(KERN_ERR "MEM: unordered memory banks.  "
				"Not freeing memmap.\n");
			break;
		}

		/*
		 * If we had a previous bank, and there is a space
		 * between the current bank and the previous, free it.
		 */
		if (prev_bank_end && prev_bank_end != bank_start)
			free_memmap(node, prev_bank_end, bank_start);

		prev_bank_end = bank_pfn_end(bank);
	}
}

/*
 * mem_init() marks the free areas in the mem_map and tells us how much
 * memory is free.  This is done after various parts of the system have
 * claimed their memory after the kernel image.
 */
void __init mem_init(void)
{
	unsigned int codesize, datasize, initsize;
	int i, node;

#ifndef CONFIG_DISCONTIGMEM
	max_mapnr   = virt_to_page(high_memory) - mem_map;
#endif

	/* this will put all unused low memory onto the freelists */
	for_each_online_node(node) {
		pg_data_t *pgdat = NODE_DATA(node);

		free_unused_memmap_node(node, &meminfo);

		if (pgdat->node_spanned_pages != 0)
			totalram_pages += free_all_bootmem_node(pgdat);
	}

#ifdef CONFIG_SA1111
	/* now that our DMA memory is actually so designated, we can free it */
	totalram_pages += free_area(PHYS_PFN_OFFSET,
				    __phys_to_pfn(__pa(swapper_pg_dir)), NULL);
#endif

	/*
	 * Since our memory may not be contiguous, calculate the
	 * real number of pages we have in this system
	 */
	printk(KERN_INFO "Memory:");
	num_physpages = 0;
	for (i = 0; i < meminfo.nr_banks; i++) {
		num_physpages += bank_pfn_size(&meminfo.bank[i]);
		printk(" %ldMB", bank_phys_size(&meminfo.bank[i]) >> 20);
	}
	printk(" = %luMB total\n", num_physpages >> (20 - PAGE_SHIFT));

	codesize = _etext - _text;
	datasize = _end - _data;
	initsize = __init_end - __init_begin;

	printk(KERN_NOTICE "Memory: %luKB available (%dK code, "
		"%dK data, %dK init)\n",
		(unsigned long) nr_free_pages() << (PAGE_SHIFT-10),
		codesize >> 10, datasize >> 10, initsize >> 10);

	if (PAGE_SIZE >= 16384 && num_physpages <= 128) {
		extern int sysctl_overcommit_memory;
		/*
		 * On a machine this small we won't get
		 * anywhere without overcommit, so turn
		 * it on by default.
		 */
		sysctl_overcommit_memory = OVERCOMMIT_ALWAYS;
	}
}

void free_initmem(void)
{
	if (!machine_is_integrator() && !machine_is_cintegrator())
		totalram_pages += free_area(__phys_to_pfn(__pa(__init_begin)),
					    __phys_to_pfn(__pa(__init_end)),
					    "init");
}

#ifdef CONFIG_MEMORY_HOTPLUG
int arch_add_memory(int nid, u64 start, u64 size)
{
	struct pglist_data *pgdata = NODE_DATA(nid);
	struct zone *zone = pgdata->node_zones + ZONE_MOVABLE;
	unsigned long start_pfn = start >> PAGE_SHIFT;
	unsigned long nr_pages = size >> PAGE_SHIFT;
	int ret;

	ret = __add_pages(nid, zone, start_pfn, nr_pages);
	if (ret)
		return ret;
	return platform_physical_add_pages(start_pfn, nr_pages);
}

int arch_physical_remove_memory(u64 start, u64 size)
{
	unsigned long start_pfn = start >> PAGE_SHIFT;
	unsigned long nr_pages = size >> PAGE_SHIFT;

	return platform_physical_remove_pages(start_pfn, nr_pages);
}

int arch_physical_low_power_memory(u64 start, u64 size)
{
	unsigned long start_pfn = start >> PAGE_SHIFT;
	unsigned long nr_pages = size >> PAGE_SHIFT;

	return platform_physical_low_power_pages(start_pfn, nr_pages);
}
#endif

#ifdef CONFIG_BLK_DEV_INITRD

static int keep_initrd;

void free_initrd_mem(unsigned long start, unsigned long end)
{
	if (!keep_initrd)
		totalram_pages += free_area(__phys_to_pfn(__pa(start)),
					    __phys_to_pfn(__pa(end)),
					    "initrd");
}

static int __init keepinitrd_setup(char *__unused)
{
	keep_initrd = 1;
	return 1;
}

__setup("keepinitrd", keepinitrd_setup);
#endif
