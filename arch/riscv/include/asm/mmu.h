/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 */


#ifndef _ASM_RISCV_MMU_H
#define _ASM_RISCV_MMU_H

#ifndef __ASSEMBLER__

typedef struct {
#ifndef CONFIG_MMU
	unsigned long	end_brk;
	unsigned long	pmp_xlate_virt;	/* nonzero = PMP offset translation active */
	unsigned long	pmp_text_offset;/* phys = virt + offset (text/flash region) */
	unsigned long	pmp_data_offset;/* phys = virt + offset (data/RAM region) */
	unsigned long	pmp_data_vaddr;	/* boundary between text and data regions */
	unsigned long	pmp_data_end;	/* end of data region (virtual) */
	unsigned long	pmp_data_phys;	/* nonzero = fork'd data alloc (needs freeing) */
	unsigned int	pmp_data_alloc_order; /* page order of above allocation */
#else
	atomic_long_t id;
#endif
	void *vdso;
#ifdef CONFIG_SMP
	/* A local icache flush is needed before user execution can resume. */
	cpumask_t icache_stale_mask;
	/* Force local icache flush on all migrations. */
	bool force_icache_flush;
#endif
#ifdef CONFIG_BINFMT_ELF_FDPIC
	unsigned long exec_fdpic_loadmap;
	unsigned long interp_fdpic_loadmap;
#endif
	unsigned long flags;
#ifdef CONFIG_RISCV_ISA_SUPM
	u8 pmlen;
#endif
} mm_context_t;

/* Lock the pointer masking mode because this mm is multithreaded */
#define MM_CONTEXT_LOCK_PMLEN	0

#define cntx2asid(cntx)		((cntx) & SATP_ASID_MASK)
#define cntx2version(cntx)	((cntx) & ~SATP_ASID_MASK)

void __meminit create_pgd_mapping(pgd_t *pgdp, uintptr_t va, phys_addr_t pa, phys_addr_t sz,
				  pgprot_t prot);

#ifdef CONFIG_RISCV_M_MODE
void riscv_pmp_xlate_setup(unsigned long text_vaddr, unsigned long data_vaddr,
			   unsigned long data_end_vaddr,
			   unsigned long textpos_phys, unsigned long datapos_phys);
void riscv_pmp_xlate_clear(void);
int riscv_pmp_fork_data(struct mm_struct *child_mm, struct mm_struct *parent_mm);
struct pt_regs;
void riscv_pmp_xlate_switch(struct pt_regs *regs);
#endif

#endif /* __ASSEMBLER__ */

#endif /* _ASM_RISCV_MMU_H */
