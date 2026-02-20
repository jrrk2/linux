// SPDX-License-Identifier: GPL-2.0
/*
 * binfmt_elf_pmp.c - Static ELF loader for nommu RISC-V with PMP translation
 *
 * Loads regular static ELF binaries using PMP offset address translation.
 * Text executes XIP from flash, data is copied to RAM. The PMP hardware
 * translates the ELF's virtual addresses to physical flash/RAM addresses.
 */

#define pr_fmt(fmt)	"elf_pmp: " fmt

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sched/task_stack.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/binfmts.h>
#include <linux/elf.h>
#include <linux/personality.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/ptrace.h>

#include <asm/elf.h>
#include <asm/cacheflush.h>
#include <asm/mmu.h>

#define STACK_ALLOC_SIZE	(PAGE_SIZE * 2)

static int load_elf_pmp_binary(struct linux_binprm *bprm);

static struct linux_binfmt elf_pmp_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_elf_pmp_binary,
};

/*
 * Read ELF program headers from the file.
 */
static struct elf_phdr *load_phdrs(const struct elfhdr *ehdr, struct file *file)
{
	struct elf_phdr *phdrs;
	size_t size;
	ssize_t ret;
	loff_t pos;

	size = sizeof(*phdrs) * ehdr->e_phnum;
	if (size == 0 || size > 65536)
		return NULL;

	phdrs = kmalloc(size, GFP_KERNEL);
	if (!phdrs)
		return NULL;

	pos = ehdr->e_phoff;
	ret = kernel_read(file, phdrs, size, &pos);
	if (ret != size) {
		kfree(phdrs);
		return NULL;
	}

	return phdrs;
}

/*
 * Set up the stack with argc, argv[], envp[], and auxv[].
 * Follows the nommu pattern from binfmt_elf_fdpic.c.
 */
static int create_elf_pmp_tables(struct linux_binprm *bprm,
				 struct mm_struct *mm,
				 unsigned long e_entry,
				 unsigned long phdr_addr,
				 int e_phnum)
{
	const struct cred *cred = current_cred();
	unsigned long sp, csp, nitems;
	unsigned long __user *argv, *envp;
	char __user *p;
	size_t len;
	int loop;
	int ei_index;
	elf_addr_t *elf_info;

	sp = mm->start_stack;

	/* transfer arg/env strings to stack */
	if (transfer_args_to_stack(bprm, &sp) < 0)
		return -EFAULT;
	sp &= ~15;

	/*
	 * Calculate space for auxv, argv, envp, argc.
	 * Auxv entries: PAGESZ, PHDR, PHENT, PHNUM, ENTRY, UID, EUID,
	 *               GID, EGID, SECURE, NULL = 11 entries
	 */
#define DLINFO_ITEMS 10

	nitems = 1 + DLINFO_ITEMS;	/* AT_NULL + entries */

	csp = sp;
	sp -= nitems * 2 * sizeof(unsigned long);
	sp -= (bprm->envc + 1) * sizeof(char *);	/* envp[] */
	sp -= (bprm->argc + 1) * sizeof(char *);	/* argv[] */
	sp -= 1 * sizeof(unsigned long);		/* argc */

	csp -= sp & 15UL;
	sp -= sp & 15UL;

	/* Build auxv in saved_auxv buffer */
	elf_info = (elf_addr_t *)mm->saved_auxv;

#define NEW_AUX_ENT(id, val) \
	do { *elf_info++ = id; *elf_info++ = val; } while (0)

	NEW_AUX_ENT(AT_PAGESZ, PAGE_SIZE);
	NEW_AUX_ENT(AT_PHDR, phdr_addr);
	NEW_AUX_ENT(AT_PHENT, sizeof(struct elf_phdr));
	NEW_AUX_ENT(AT_PHNUM, e_phnum);
	NEW_AUX_ENT(AT_ENTRY, e_entry);
	NEW_AUX_ENT(AT_UID, from_kuid_munged(cred->user_ns, cred->uid));
	NEW_AUX_ENT(AT_EUID, from_kuid_munged(cred->user_ns, cred->euid));
	NEW_AUX_ENT(AT_GID, from_kgid_munged(cred->user_ns, cred->gid));
	NEW_AUX_ENT(AT_EGID, from_kgid_munged(cred->user_ns, cred->egid));
	NEW_AUX_ENT(AT_SECURE, bprm->secureexec);
#undef NEW_AUX_ENT

	/* AT_NULL and clear rest */
	memset(elf_info, 0, (char *)mm->saved_auxv +
	       sizeof(mm->saved_auxv) - (char *)elf_info);
	elf_info += 2;

	ei_index = elf_info - (elf_addr_t *)mm->saved_auxv;
	csp -= ei_index * sizeof(elf_addr_t);

	/* Copy auxv to stack */
	if (copy_to_user((void __user *)csp, mm->saved_auxv,
			 ei_index * sizeof(elf_addr_t)))
		return -EFAULT;

	/* Allocate room for argv[] and envp[] */
	csp -= (bprm->envc + 1) * sizeof(unsigned long);
	envp = (unsigned long __user *)csp;
	csp -= (bprm->argc + 1) * sizeof(unsigned long);
	argv = (unsigned long __user *)csp;

	/* Push argc */
	csp -= sizeof(unsigned long);
	if (put_user(bprm->argc, (unsigned long __user *)csp))
		return -EFAULT;

	/* Fill in argv[] */
	mm->arg_start = mm->start_stack -
		(MAX_ARG_PAGES * PAGE_SIZE - bprm->p);
	p = (char __user *)mm->arg_start;
	for (loop = bprm->argc; loop > 0; loop--) {
		if (put_user((unsigned long)p, argv++))
			return -EFAULT;
		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (put_user(0UL, argv))
		return -EFAULT;
	mm->arg_end = (unsigned long)p;

	/* Fill in envp[] */
	mm->env_start = (unsigned long)p;
	for (loop = bprm->envc; loop > 0; loop--) {
		if (put_user((unsigned long)p, envp++))
			return -EFAULT;
		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (put_user(0UL, envp))
		return -EFAULT;
	mm->env_end = (unsigned long)p;

	mm->start_stack = (unsigned long)sp;
	return 0;
}

static int load_elf_pmp_binary(struct linux_binprm *bprm)
{
	struct elfhdr *ehdr = (struct elfhdr *)bprm->buf;
	struct elf_phdr *phdrs, *text_phdr = NULL, *data_phdr = NULL;
	unsigned long textpos, datapos, data_alloc;
	unsigned long text_vaddr, data_vaddr, data_end_vaddr;
	unsigned long phdr_addr;
	unsigned long stack_size;
	int retval, i;
	ssize_t result;
	struct pt_regs *regs;
	struct mm_struct *mm;

	/* Validate ELF header */
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0)
		return -ENOEXEC;
	if (ehdr->e_type != ET_EXEC)
		return -ENOEXEC;
	if (!elf_check_arch(ehdr))
		return -ENOEXEC;

	/* Load program headers */
	phdrs = load_phdrs(ehdr, bprm->file);
	if (!phdrs)
		return -ENOEXEC;

	/* Find text and data PT_LOAD segments, reject PT_INTERP */
	for (i = 0; i < ehdr->e_phnum; i++) {
		if (phdrs[i].p_type == PT_INTERP) {
			pr_err("dynamic linking not supported\n");
			retval = -ENOEXEC;
			goto out_free;
		}
		if (phdrs[i].p_type != PT_LOAD)
			continue;

		if (phdrs[i].p_flags & PF_X) {
			if (text_phdr) {
				pr_err("multiple executable segments\n");
				retval = -ENOEXEC;
				goto out_free;
			}
			text_phdr = &phdrs[i];
		} else if (phdrs[i].p_flags & PF_W) {
			if (data_phdr) {
				pr_err("multiple writable segments\n");
				retval = -ENOEXEC;
				goto out_free;
			}
			data_phdr = &phdrs[i];
		}
	}

	if (!text_phdr) {
		pr_err("no executable segment found\n");
		retval = -ENOEXEC;
		goto out_free;
	}

	/* Flush all traces of the currently running executable */
	retval = begin_new_exec(bprm);
	if (retval)
		goto out_free;

	set_personality(PER_LINUX_32BIT);
	setup_new_exec(bprm);

	/*
	 * Map text segment XIP from flash.
	 * On romfs/phram, vm_mmap returns the physical flash address.
	 */
	textpos = vm_mmap(bprm->file, 0, text_phdr->p_filesz,
			  PROT_READ | PROT_EXEC, MAP_PRIVATE,
			  text_phdr->p_offset);
	if (!textpos || IS_ERR_VALUE(textpos)) {
		retval = textpos ? (int)textpos : -ENOMEM;
		pr_err("unable to mmap text, errno %d\n", retval);
		goto out_free;
	}

	text_vaddr = text_phdr->p_vaddr;

	/*
	 * Allocate RAM for data + BSS + stack.
	 */
	stack_size = STACK_ALLOC_SIZE;
	if (bprm->argc + bprm->envc > 0)
		stack_size += PAGE_SIZE * MAX_ARG_PAGES;
	stack_size += (bprm->argc + 1) * sizeof(char *);
	stack_size += (bprm->envc + 1) * sizeof(char *);

	if (data_phdr) {
		data_alloc = data_phdr->p_memsz + stack_size;
		data_alloc = PAGE_ALIGN(data_alloc);

		datapos = vm_mmap(NULL, 0, data_alloc,
				  PROT_READ | PROT_WRITE | PROT_EXEC,
				  MAP_PRIVATE, 0);
		if (!datapos || IS_ERR_VALUE(datapos)) {
			retval = datapos ? (int)datapos : -ENOMEM;
			pr_err("unable to allocate data, errno %d\n", retval);
			vm_munmap(textpos, text_phdr->p_filesz);
			goto out_free;
		}

		/* Read data from file */
		if (data_phdr->p_filesz > 0) {
			result = read_code(bprm->file, datapos,
					   data_phdr->p_offset,
					   data_phdr->p_filesz);
			if (IS_ERR_VALUE(result)) {
				retval = (int)result;
				pr_err("unable to read data, errno %d\n", retval);
				vm_munmap(textpos, text_phdr->p_filesz);
				vm_munmap(datapos, data_alloc);
				goto out_free;
			}
		}

		/* Zero BSS */
		if (data_phdr->p_memsz > data_phdr->p_filesz) {
			if (clear_user((void __user *)(datapos + data_phdr->p_filesz),
				       data_phdr->p_memsz - data_phdr->p_filesz)) {
				retval = -EFAULT;
				vm_munmap(textpos, text_phdr->p_filesz);
				vm_munmap(datapos, data_alloc);
				goto out_free;
			}
		}

		data_vaddr = data_phdr->p_vaddr;
		data_end_vaddr = data_vaddr + data_alloc;
	} else {
		/* Text-only binary, allocate just a stack */
		data_alloc = PAGE_ALIGN(stack_size);
		datapos = vm_mmap(NULL, 0, data_alloc,
				  PROT_READ | PROT_WRITE | PROT_EXEC,
				  MAP_PRIVATE, 0);
		if (!datapos || IS_ERR_VALUE(datapos)) {
			retval = datapos ? (int)datapos : -ENOMEM;
			vm_munmap(textpos, text_phdr->p_filesz);
			goto out_free;
		}

		data_vaddr = PAGE_ALIGN(text_vaddr + text_phdr->p_memsz);
		data_end_vaddr = data_vaddr + data_alloc;
	}

	/* Set up PMP address translation */
	riscv_pmp_xlate_setup(text_vaddr, data_vaddr, data_end_vaddr,
			      textpos, datapos);

	/* Set up mm fields using virtual addresses */
	mm = current->mm;
	mm->start_code = text_vaddr;
	mm->end_code = text_vaddr + text_phdr->p_filesz;

	if (data_phdr) {
		mm->start_data = data_vaddr;
		mm->end_data = data_vaddr + data_phdr->p_filesz;
		mm->start_brk = data_vaddr + data_phdr->p_memsz;
	} else {
		mm->start_data = data_vaddr;
		mm->end_data = data_vaddr;
		mm->start_brk = data_vaddr;
	}
	mm->brk = (mm->start_brk + 3) & ~3;
	mm->context.end_brk = data_vaddr + data_alloc - stack_size;
	mm->context.pmp_xlate_virt = text_vaddr;

	/* Stack at top of data allocation (virtual addresses) */
	mm->start_stack = ((data_end_vaddr + 3) & ~3) - 4;

	/* Program header virtual address for auxv */
	if (text_phdr->p_offset <= ehdr->e_phoff &&
	    ehdr->e_phoff < text_phdr->p_offset + text_phdr->p_filesz)
		phdr_addr = text_vaddr + (ehdr->e_phoff - text_phdr->p_offset);
	else
		phdr_addr = 0;

	/* Zero the stack area */
	if (clear_user((void __user *)(data_end_vaddr - stack_size), stack_size)) {
		retval = -EFAULT;
		goto out_free;
	}

	/* Set up stack tables */
	retval = create_elf_pmp_tables(bprm, mm, ehdr->e_entry,
				       phdr_addr, ehdr->e_phnum);
	if (retval < 0)
		goto out_free;

	set_binfmt(&elf_pmp_format);

	flush_icache_user_range(mm->start_code, mm->end_code);

	kfree(phdrs);

	regs = current_pt_regs();
	finalize_exec(bprm);

	if (0) pr_info("%s: TEXT=%lx-%lx DATA=%lx-%lx BSS=%lx BRK=%lx STACK=%lx\n",
		bprm->filename,
		mm->start_code, mm->end_code,
		mm->start_data, mm->end_data,
		mm->start_brk, mm->context.end_brk,
		mm->start_stack);

	start_thread(regs, ehdr->e_entry, mm->start_stack);
	return 0;

out_free:
	kfree(phdrs);
	return retval;
}

static int __init init_elf_pmp_binfmt(void)
{
	register_binfmt(&elf_pmp_format);
	return 0;
}
core_initcall(init_elf_pmp_binfmt);
