// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2009 Sunplus Core Technology Co., Ltd.
 *  Chen Liqin <liqin.chen@sunplusct.com>
 *  Lennox Wu <lennox.wu@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2020 FORTH-ICS/CARV
 *  Nick Kossifidis <mick@ics.forth.gr>
 */

#include <linux/acpi.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/memblock.h>
#include <linux/sched.h>
#include <linux/console.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/sched/task.h>
#include <linux/smp.h>
#include <linux/efi.h>
#include <linux/crash_dump.h>
#include <linux/panic_notifier.h>
#include <linux/jump_label.h>
#include <linux/gcd.h>

#include <asm/acpi.h>
#include <asm/alternative.h>
#include <asm/cacheflush.h>
#include <asm/cpufeature.h>
#include <asm/early_ioremap.h>
#include <asm/pgtable.h>
#include <asm/setup.h>
#include <asm/set_memory.h>
#include <asm/ptrace.h>
#include <asm/sections.h>
#include <asm/sbi.h>
#include <asm/switch_to.h>
#include <asm/tlbflush.h>
#include <asm/thread_info.h>
#include <asm/kasan.h>
#include <asm/efi.h>
#include <asm/sonata.h>

#include "head.h"

/*
 * The lucky hart to first increment this variable will boot the other cores.
 * This is used before the kernel initializes the BSS so it can't be in the
 * BSS.
 */
atomic_t hart_lottery __section(".sdata")
#ifdef CONFIG_XIP_KERNEL
= ATOMIC_INIT(0xC001BEEF)
#endif
;
unsigned long boot_cpu_hartid;
EXPORT_SYMBOL_GPL(boot_cpu_hartid);

/*
 * Place kernel memory regions on the resource tree so that
 * kexec-tools can retrieve them from /proc/iomem. While there
 * also add "System RAM" regions for compatibility with other
 * archs, and the rest of the known regions for completeness.
 */
static struct resource kimage_res = { .name = "Kernel image", };
static struct resource code_res = { .name = "Kernel code", };
static struct resource data_res = { .name = "Kernel data", };
static struct resource rodata_res = { .name = "Kernel rodata", };
static struct resource bss_res = { .name = "Kernel bss", };
#ifdef CONFIG_CRASH_DUMP
static struct resource elfcorehdr_res = { .name = "ELF Core hdr", };
#endif

static int num_standard_resources;
static struct resource *standard_resources;

static int __init add_resource(struct resource *parent,
				struct resource *res)
{
	int ret = 0;

	ret = insert_resource(parent, res);
	if (ret < 0) {
		pr_err("Failed to add a %s resource at %llx\n",
			res->name, (unsigned long long) res->start);
		return ret;
	}

	return 1;
}

static int __init add_kernel_resources(void)
{
	int ret = 0;

	/*
	 * The memory region of the kernel image is continuous and
	 * was reserved on setup_bootmem, register it here as a
	 * resource, with the various segments of the image as
	 * child nodes.
	 */

	code_res.start = __pa_symbol(_text);
	code_res.end = __pa_symbol(_etext) - 1;
	code_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;

	rodata_res.start = __pa_symbol(__start_rodata);
	rodata_res.end = __pa_symbol(__end_rodata) - 1;
	rodata_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;

	data_res.start = __pa_symbol(_data);
	data_res.end = __pa_symbol(_edata) - 1;
	data_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;

	bss_res.start = __pa_symbol(__bss_start);
	bss_res.end = __pa_symbol(__bss_stop) - 1;
	bss_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;

	kimage_res.start = code_res.start;
	kimage_res.end = bss_res.end;
	kimage_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;

	ret = add_resource(&iomem_resource, &kimage_res);
	if (ret < 0)
		return ret;

	ret = add_resource(&kimage_res, &code_res);
	if (ret < 0)
		return ret;

	ret = add_resource(&kimage_res, &rodata_res);
	if (ret < 0)
		return ret;

	ret = add_resource(&kimage_res, &data_res);
	if (ret < 0)
		return ret;

	ret = add_resource(&kimage_res, &bss_res);

	return ret;
}

static void __init init_resources(void)
{
	struct memblock_region *region = NULL;
	struct resource *res = NULL;
	struct resource *mem_res = NULL;
	size_t mem_res_sz = 0;
	int num_resources = 0, res_idx = 0, non_resv_res = 0;
	int ret = 0;

	/* + 1 as memblock_alloc() might increase memblock.reserved.cnt */
	num_resources = memblock.memory.cnt + memblock.reserved.cnt + 1;
	res_idx = num_resources - 1;

	mem_res_sz = num_resources * sizeof(*mem_res);
	mem_res = memblock_alloc_or_panic(mem_res_sz, SMP_CACHE_BYTES);

	/*
	 * Start by adding the reserved regions, if they overlap
	 * with /memory regions, insert_resource later on will take
	 * care of it.
	 */
	ret = add_kernel_resources();
	if (ret < 0)
		goto error;

#ifdef CONFIG_CRASH_DUMP
	if (elfcorehdr_size > 0) {
		elfcorehdr_res.start = elfcorehdr_addr;
		elfcorehdr_res.end = elfcorehdr_addr + elfcorehdr_size - 1;
		elfcorehdr_res.flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
		add_resource(&iomem_resource, &elfcorehdr_res);
	}
#endif

	for_each_reserved_mem_region(region) {
		res = &mem_res[res_idx--];

		res->name = "Reserved";
		res->flags = IORESOURCE_MEM | IORESOURCE_EXCLUSIVE;
		res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
		res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;

		/*
		 * Ignore any other reserved regions within
		 * system memory.
		 */
		if (memblock_is_memory(res->start)) {
			/* Re-use this pre-allocated resource */
			res_idx++;
			continue;
		}

		ret = add_resource(&iomem_resource, res);
		if (ret < 0)
			goto error;
	}

	/* Add /memory regions to the resource tree */
	for_each_mem_region(region) {
		res = &mem_res[res_idx--];
		non_resv_res++;

		if (unlikely(memblock_is_nomap(region))) {
			res->name = "Reserved";
			res->flags = IORESOURCE_MEM | IORESOURCE_EXCLUSIVE;
		} else {
			res->name = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
		}

		res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
		res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;

		ret = add_resource(&iomem_resource, res);
		if (ret < 0)
			goto error;
	}

	num_standard_resources = non_resv_res;
	standard_resources = &mem_res[res_idx + 1];

	/* Clean-up any unused pre-allocated resources */
	if (res_idx >= 0)
		memblock_free(mem_res, (res_idx + 1) * sizeof(*mem_res));
	return;

 error:
	/* Better an empty resource tree than an inconsistent one */
	release_child_resources(&iomem_resource);
	memblock_free(mem_res, mem_res_sz);
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; i++) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "Reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

static void __init parse_dtb(void)
{
	/* Early scan of device tree from init memory */
	if (early_init_dt_scan(dtb_early_va, dtb_early_pa)) {
		const char *name = of_flat_dt_get_machine_name();

		if (name) {
			pr_info("Machine model: %s\n", name);
			dump_stack_set_arch_desc("%s (DT)", name);
		}
	} else {
		pr_err("No DTB passed to the kernel\n");
	}
}

#if defined(CONFIG_RISCV_COMBO_SPINLOCKS)
DEFINE_STATIC_KEY_TRUE(qspinlock_key);
EXPORT_SYMBOL(qspinlock_key);
#endif

static void __init riscv_spinlock_init(void)
{
	char *using_ext = NULL;

	if (IS_ENABLED(CONFIG_RISCV_TICKET_SPINLOCKS)) {
		pr_info("Ticket spinlock: enabled\n");
		return;
	}

	if (IS_ENABLED(CONFIG_RISCV_ISA_ZABHA) &&
	    IS_ENABLED(CONFIG_RISCV_ISA_ZACAS) &&
	    IS_ENABLED(CONFIG_TOOLCHAIN_HAS_ZACAS) &&
	    riscv_isa_extension_available(NULL, ZABHA) &&
	    riscv_isa_extension_available(NULL, ZACAS)) {
		using_ext = "using Zabha";
	} else if (riscv_isa_extension_available(NULL, ZICCRSE)) {
		using_ext = "using Ziccrse";
	}
#if defined(CONFIG_RISCV_COMBO_SPINLOCKS)
	else {
		static_branch_disable(&qspinlock_key);
		pr_info("Ticket spinlock: enabled\n");
		return;
	}
#endif

	if (!using_ext)
		pr_err("Queued spinlock without Zabha or Ziccrse");
	else
		pr_info("Queued spinlock %s: enabled\n", using_ext);
}

#ifdef CONFIG_RISCV_M_MODE
/*
 * PMP offset address translation for XIP binaries.
 *
 * Programs PMP entries 0-2 to create a virtual address space:
 *   Entry 0: OFF, pmpaddr0 = text_vaddr >> 2 (TOR lower bound for entry 1)
 *   Entry 1: TOR R+X, maps virtual text to flash via offset
 *   Entry 2: TOR R+W+X, maps virtual data to RAM via offset
 *
 * These have higher priority than the catch-all on entry 6 (set in head.S).
 */
void riscv_pmp_xlate_setup(unsigned long text_vaddr, unsigned long data_vaddr,
			   unsigned long data_end_vaddr,
			   unsigned long textpos_phys, unsigned long datapos_phys)
{
	unsigned long cfg;

	/* TOR boundaries */
	csr_write(CSR_PMPADDR0 + 0, text_vaddr >> 2);
	csr_write(CSR_PMPADDR0 + 1, data_vaddr >> 2);
	csr_write(CSR_PMPADDR0 + 2, data_end_vaddr >> 2);

	/* Translation offsets */
	csr_write(CSR_PMPOFFSET0 + 1, textpos_phys - text_vaddr);
	csr_write(CSR_PMPOFFSET0 + 2, datapos_phys - data_vaddr);

	/* pmpcfg0: byte0=OFF, byte1=TOR|R|X, byte2=TOR|R|W|X, byte3=preserve */
	cfg = csr_read(CSR_PMPCFG0);
	cfg &= 0xff000000UL;
	cfg |= (PMP_A_TOR | PMP_R | PMP_W | PMP_X) << 16;	/* entry 2 */
	cfg |= (PMP_A_TOR | PMP_R | PMP_X) << 8;		/* entry 1 */
	/* entry 0: byte0 = 0 (OFF) */
	csr_write(CSR_PMPCFG0, cfg);

	/* Store translation info in mm for access_ok validation */
	current->mm->context.pmp_text_offset = textpos_phys - text_vaddr;
	current->mm->context.pmp_data_offset = datapos_phys - data_vaddr;
	current->mm->context.pmp_data_vaddr = data_vaddr;
	current->mm->context.pmp_data_end = data_end_vaddr;

	if (0) pr_info("PMP xlate: text %lx->%lx data %lx->%lx\n",
		text_vaddr, textpos_phys, data_vaddr, datapos_phys);
}

void riscv_pmp_xlate_clear(void)
{
	unsigned long cfg;

	/* Clear pmpcfg0 bytes 0-2 (entries 0-2), preserve byte 3 (entry 3) */
	cfg = csr_read(CSR_PMPCFG0);
	cfg &= 0xff000000UL;
	csr_write(CSR_PMPCFG0, cfg);

	/* Clear offsets */
	csr_write(CSR_PMPOFFSET0 + 1, 0);
	csr_write(CSR_PMPOFFSET0 + 2, 0);
}

/*
 * Fork data+stack for PMP xlate process.
 * Allocates new physical RAM for the child's data region and copies
 * the parent's data+stack contents. Both share text XIP from flash.
 */
int riscv_pmp_fork_data(struct mm_struct *child_mm, struct mm_struct *parent_mm)
{
	unsigned long size, parent_phys, child_phys;
	int order;
	struct page *pages;

	size = parent_mm->context.pmp_data_end - parent_mm->context.pmp_data_vaddr;
	parent_phys = parent_mm->context.pmp_data_vaddr +
		      parent_mm->context.pmp_data_offset;

	order = get_order(size);
	pages = alloc_pages(GFP_KERNEL, order);
	if (!pages)
		return -ENOMEM;

	child_phys = (unsigned long)page_address(pages);
	memcpy((void *)child_phys, (void *)parent_phys, size);

	child_mm->context.pmp_data_offset = child_phys -
					    child_mm->context.pmp_data_vaddr;
	child_mm->context.pmp_data_phys = child_phys;
	child_mm->context.pmp_data_alloc_order = order;

	if (0) pr_info("PMP fork: pid=%d parent_data=%lx child_data=%lx size=%lx\n",
		current->pid, parent_phys, child_phys, size);

	return 0;
}

/*
 * Restore PMP address translation for the current process.
 * Called from ret_from_exception when returning to user mode,
 * to ensure PMP entries match the current mm after context switch.
 */
void riscv_pmp_xlate_switch(struct pt_regs *regs)
{
	struct mm_struct *mm = current->mm;
	unsigned long cfg;
	static unsigned long last_data_offset;

	if (!mm || !mm->context.pmp_xlate_virt)
		return;

	if (mm->context.pmp_data_offset != last_data_offset) {
		if (0) pr_info("PMP switch: pid=%d data_off=%lx->%lx epc=%lx ra=%lx sp=%lx\n",
			current->pid, last_data_offset,
			mm->context.pmp_data_offset,
			regs->epc, regs->ra, regs->sp);
		last_data_offset = mm->context.pmp_data_offset;
	}

	csr_write(CSR_PMPADDR0 + 0, mm->context.pmp_xlate_virt >> 2);
	csr_write(CSR_PMPADDR0 + 1, mm->context.pmp_data_vaddr >> 2);
	csr_write(CSR_PMPADDR0 + 2, mm->context.pmp_data_end >> 2);

	csr_write(CSR_PMPOFFSET0 + 1, mm->context.pmp_text_offset);
	csr_write(CSR_PMPOFFSET0 + 2, mm->context.pmp_data_offset);

	cfg = csr_read(CSR_PMPCFG0);
	cfg &= 0xff000000UL;
	cfg |= (PMP_A_TOR | PMP_R | PMP_W | PMP_X) << 16;
	cfg |= (PMP_A_TOR | PMP_R | PMP_X) << 8;
	csr_write(CSR_PMPCFG0, cfg);

	/* I-cache may have stale entries from old PMP translation */
	asm volatile ("fence.i" ::: "memory");
}

/*
 * Switch PMP address translation to match a task's mm.
 * Called from __switch_to in entry.S during context switch,
 * so that kernel code running after switch (e.g. signal delivery)
 * accesses the correct physical memory via PMP offsets.
 */
void riscv_pmp_switch_task(struct task_struct *next)
{
	struct mm_struct *mm = next->mm;
	unsigned long cfg;

	if (!mm || !mm->context.pmp_xlate_virt)
		return;

	csr_write(CSR_PMPADDR0 + 0, mm->context.pmp_xlate_virt >> 2);
	csr_write(CSR_PMPADDR0 + 1, mm->context.pmp_data_vaddr >> 2);
	csr_write(CSR_PMPADDR0 + 2, mm->context.pmp_data_end >> 2);

	csr_write(CSR_PMPOFFSET0 + 1, mm->context.pmp_text_offset);
	csr_write(CSR_PMPOFFSET0 + 2, mm->context.pmp_data_offset);

	cfg = csr_read(CSR_PMPCFG0);
	cfg &= 0xff000000UL;
	cfg |= (PMP_A_TOR | PMP_R | PMP_W | PMP_X) << 16;
	cfg |= (PMP_A_TOR | PMP_R | PMP_X) << 8;
	csr_write(CSR_PMPCFG0, cfg);

	asm volatile ("fence.i" ::: "memory");
}
#endif /* CONFIG_RISCV_M_MODE */

extern void __init init_rt_signal_env(void);

#ifndef CONFIG_MMU
asmlinkage void riscv_sonata_do_stack_switch(unsigned long new_sp,
					     unsigned long new_base);

void __visible __used riscv_sonata_stack_switch_finish(unsigned long new_base)
{
	current->stack = (void *)new_base;
	set_task_stack_end_magic(current);
#ifdef CONFIG_RISCV_M_MODE
	__switch_to_pmp_guard(current);
#endif
	pr_info("Sonata: switched kernel stack to HyperRAM [%px - %px]\n",
		(void *)new_base,
		(void *)(new_base + THREAD_SIZE));
}
EXPORT_SYMBOL_GPL(riscv_sonata_stack_switch_finish);

static void (*__initdata __used sonata_stack_switch_finish_ptr)
	(unsigned long) = riscv_sonata_stack_switch_finish;

unsigned long __init riscv_sonata_prepare_stack_switch(void)
{
	if (!of_machine_is_compatible("lowrisc,sonata"))
		return 0;

	memset((void *)SONATA_HYPERRAM_STACK_BASE, 0, THREAD_SIZE);
	pr_info("Sonata: preparing stack switch to HyperRAM\n");
	return SONATA_HYPERRAM_STACK_BASE;
}
#else
unsigned long __init riscv_sonata_prepare_stack_switch(void) { return 0; }
void riscv_sonata_stack_switch_finish(unsigned long new_base) { }
#endif

/* Expected .text checksum — patched into vmlinux by scripts/patch_text_checksum.py */
u32 expected_text_checksum __section(".data") = 0;

void __init setup_arch(char **cmdline_p)
{
	/* Checksum kernel .text to detect HyperRAM corruption */
	{
		const u32 *p = (const u32 *)_stext;
		const u32 *end = (const u32 *)_etext;
		u32 sum = 0;

		while (p < end)
			sum += *p++;
		if (expected_text_checksum && sum != expected_text_checksum)
			pr_emerg("TEXT CHECKSUM MISMATCH: got 0x%08x expected 0x%08x\n",
				 sum, expected_text_checksum);
		else if (expected_text_checksum)
			pr_info("text checksum: 0x%08x OK (%u bytes)\n",
				sum, (unsigned int)(_etext - _stext));
		else
			pr_info("text checksum: 0x%08x (%u bytes) [no expected value]\n",
				sum, (unsigned int)(_etext - _stext));
	}

	/*
	 * Sonata pinmux: route SPI0 signals to microSD card pins.
	 *   0x80005052 ← 1  microsd_cmd  → spi_0_copi
	 *   0x80005053 ← 1  microsd_clk  → spi_0_sclk
	 *   0x80005054 ← 1  microsd_dat3 → spi_0_cs[1]
	 *   0x80005843 ← 2  spi_0_cipo   ← microsd_dat0
	 */
	{
		void __iomem *pinmux = ioremap(0x80005000, 0x1000);

		if (pinmux) {
			writeb(1, pinmux + 0x052);
			writeb(1, pinmux + 0x053);
			writeb(1, pinmux + 0x054);
			writeb(2, pinmux + 0x843);
			iounmap(pinmux);
			pr_info("Sonata pinmux: microSD routed to SPI0\n");
		}
	}

	parse_dtb();
	setup_initial_init_mm(_stext, _etext, _edata, _end);

	*cmdline_p = boot_command_line;

	early_ioremap_setup();
	sbi_init();
	jump_label_init();
	parse_early_param();

	efi_init();
	paging_init();

	/* Parse the ACPI tables for possible boot-time configuration */
	acpi_boot_table_init();

	if (acpi_disabled)
		unflatten_device_tree();

	misc_mem_init();

	init_resources();

#ifdef CONFIG_KASAN
	kasan_init();
#endif

#ifdef CONFIG_SMP
	setup_smp();
#endif

	if (!acpi_disabled) {
		acpi_init_rintc_map();
		acpi_map_cpus_to_nodes();
	}

	riscv_init_cbo_blocksizes();
	riscv_fill_hwcap();
	apply_boot_alternatives();

	/*
	 * Protect kernel text with PMP, locked R+X.
	 * Must be after apply_boot_alternatives() which patches .text.
	 *
	 * PMP entry layout (7 entries available, entry 7 disabled in HW):
	 *   0-2: Reserved for XIP offset translation (binfmt_elf_pmp)
	 *   3:   OFF, pmpaddr = _stext >> 2 (TOR bottom for entry 4)
	 *   4:   TOR, locked R+X (kernel text protection)
	 *   5:   NAPOT, locked no-access (stack guard, updated on switch)
	 *   6:   NAPOT, RWX catch-all (set in head.S)
	 */
#ifdef CONFIG_RISCV_M_MODE
	{
		unsigned long etext_aligned = (unsigned long)_etext;
		etext_aligned = (etext_aligned + PAGE_SIZE - 1) & PAGE_MASK;

		/* Entry 3: TOR bottom (cfg=OFF, just sets the address) */
		csr_write(CSR_PMPADDR0 + 3, (unsigned long)_stext >> 2);

		/* Entry 4: TOR top, locked R+X */
		csr_write(CSR_PMPADDR0 + 4, etext_aligned >> 2);

		/* pmpcfg1 byte 0 = L|R|X|TOR = 0x8D (entry 4), preserve rest */
		unsigned long cfg = csr_read(CSR_PMPCFG0 + 1);
		cfg &= ~0xffUL;
		cfg |= 0x8dUL;  /* L|R|X|TOR */
		csr_write(CSR_PMPCFG0 + 1, cfg);

		pr_info("PMP: text protected [%px - %px] (locked R+X TOR)\n",
			_stext, (void *)etext_aligned);

		/*
		 * ePMP (Smepmp): enable Rule Locking Bypass so we can
		 * update the locked stack guard entry on context switch.
		 */
		csr_write(CSR_MSECCFG, MSECCFG_RLB);

		/*
		 * PMP entry 5: locked NAPOT no-access guard at bottom
		 * of current (idle) task stack.  128 bytes.
		 */
		{
			unsigned long guard = (unsigned long)current->stack;
			csr_write(CSR_PMPADDR0 + 5,
				  (guard >> 2) | PMP_GUARD_NAPOT_MASK);
			cfg = csr_read(CSR_PMPCFG0 + 1);
			cfg &= ~(0xffUL << 8);
			cfg |= ((unsigned long)PMP_GUARD_CFG << 8);
			csr_write(CSR_PMPCFG0 + 1, cfg);
			pr_info("PMP: stack guard [%px - %px] (ePMP RLB)\n",
				(void *)guard,
				(void *)(guard + PMP_GUARD_SIZE));
		}
	}
#endif

	init_rt_signal_env();

	if (IS_ENABLED(CONFIG_RISCV_ISA_ZICBOM) &&
	    riscv_isa_extension_available(NULL, ZICBOM))
		riscv_noncoherent_supported();
	riscv_set_dma_cache_alignment();

	riscv_user_isa_enable();
	riscv_spinlock_init();

	if (!IS_ENABLED(CONFIG_RISCV_ISA_ZBB) || !riscv_isa_extension_available(NULL, ZBB))
		static_branch_disable(&efficient_ffs_key);
}

bool arch_cpu_is_hotpluggable(int cpu)
{
	return cpu_has_hotplug(cpu);
}

void free_initmem(void)
{
	if (IS_ENABLED(CONFIG_STRICT_KERNEL_RWX)) {
		set_kernel_memory(lm_alias(__init_begin), lm_alias(__init_end), set_memory_rw_nx);
		if (IS_ENABLED(CONFIG_64BIT))
			set_kernel_memory(__init_begin, __init_end, set_memory_nx);
	}

	free_initmem_default(POISON_FREE_INITMEM);
}

static int dump_kernel_offset(struct notifier_block *self,
			      unsigned long v, void *p)
{
	pr_emerg("Kernel Offset: 0x%lx from 0x%lx\n",
		 kernel_map.virt_offset,
		 KERNEL_LINK_ADDR);

	return 0;
}

static struct notifier_block kernel_offset_notifier = {
	.notifier_call = dump_kernel_offset
};

static int __init register_kernel_offset_dumper(void)
{
	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE))
		atomic_notifier_chain_register(&panic_notifier_list,
					       &kernel_offset_notifier);

	return 0;
}
device_initcall(register_kernel_offset_dumper);
