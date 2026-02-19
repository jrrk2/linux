/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_MM_HOOKS_H
#define _ASM_RISCV_MM_HOOKS_H

#include <asm/mmu.h>

static inline int arch_dup_mmap(struct mm_struct *oldmm,
				struct mm_struct *mm)
{
	return 0;
}

static inline void arch_exit_mmap(struct mm_struct *mm)
{
#if defined(CONFIG_RISCV_M_MODE) && !defined(CONFIG_MMU)
	if (mm->context.pmp_xlate_virt)
		riscv_pmp_xlate_clear();
#endif
}

static inline bool arch_vma_access_permitted(struct vm_area_struct *vma,
		bool write, bool execute, bool foreign)
{
	return true;
}

#endif	/* _ASM_RISCV_MM_HOOKS_H */
