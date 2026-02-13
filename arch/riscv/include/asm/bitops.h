/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 */

#ifndef _ASM_RISCV_BITOPS_H
#define _ASM_RISCV_BITOPS_H

#ifndef _LINUX_BITOPS_H
#error "Only <linux/bitops.h> can be included directly"
#endif /* _LINUX_BITOPS_H */

#include <linux/compiler.h>
#include <linux/irqflags.h>
#include <asm/barrier.h>
#include <asm/bitsperlong.h>

#if !(defined(CONFIG_RISCV_ISA_ZBB) && defined(CONFIG_TOOLCHAIN_HAS_ZBB)) || defined(NO_ALTERNATIVE)
#include <asm-generic/bitops/__ffs.h>
#include <asm-generic/bitops/__fls.h>
#include <asm-generic/bitops/ffs.h>
#include <asm-generic/bitops/fls.h>

#else
#define __HAVE_ARCH___FFS
#define __HAVE_ARCH___FLS
#define __HAVE_ARCH_FFS
#define __HAVE_ARCH_FLS

#include <asm-generic/bitops/__ffs.h>
#include <asm-generic/bitops/__fls.h>
#include <asm-generic/bitops/ffs.h>
#include <asm-generic/bitops/fls.h>

#include <asm/alternative-macros.h>
#include <asm/hwcap.h>

#if (BITS_PER_LONG == 64)
#define CTZW	"ctzw "
#define CLZW	"clzw "
#elif (BITS_PER_LONG == 32)
#define CTZW	"ctz "
#define CLZW	"clz "
#else
#error "Unexpected BITS_PER_LONG"
#endif

static __always_inline __attribute_const__ unsigned long variable__ffs(unsigned long word)
{
	asm goto(ALTERNATIVE("j %l[legacy]", "nop", 0,
				      RISCV_ISA_EXT_ZBB, 1)
			  : : : : legacy);

	asm volatile (".option push\n"
		      ".option arch,+zbb\n"
		      "ctz %0, %1\n"
		      ".option pop\n"
		      : "=r" (word) : "r" (word) :);

	return word;

legacy:
	return generic___ffs(word);
}

/**
 * __ffs - find first set bit in a long word
 * @word: The word to search
 *
 * Undefined if no set bit exists, so code should check against 0 first.
 */
#define __ffs(word)				\
	(__builtin_constant_p(word) ?		\
	 (unsigned long)__builtin_ctzl(word) :	\
	 variable__ffs(word))

static __always_inline __attribute_const__ unsigned long variable__fls(unsigned long word)
{
	asm goto(ALTERNATIVE("j %l[legacy]", "nop", 0,
				      RISCV_ISA_EXT_ZBB, 1)
			  : : : : legacy);

	asm volatile (".option push\n"
		      ".option arch,+zbb\n"
		      "clz %0, %1\n"
		      ".option pop\n"
		      : "=r" (word) : "r" (word) :);

	return BITS_PER_LONG - 1 - word;

legacy:
	return generic___fls(word);
}

/**
 * __fls - find last set bit in a long word
 * @word: the word to search
 *
 * Undefined if no set bit exists, so code should check against 0 first.
 */
#define __fls(word)							\
	(__builtin_constant_p(word) ?					\
	 (unsigned long)(BITS_PER_LONG - 1 - __builtin_clzl(word)) :	\
	 variable__fls(word))

static __always_inline __attribute_const__ int variable_ffs(int x)
{
	asm goto(ALTERNATIVE("j %l[legacy]", "nop", 0,
				      RISCV_ISA_EXT_ZBB, 1)
			  : : : : legacy);

	if (!x)
		return 0;

	asm volatile (".option push\n"
		      ".option arch,+zbb\n"
		      CTZW "%0, %1\n"
		      ".option pop\n"
		      : "=r" (x) : "r" (x) :);

	return x + 1;

legacy:
	return generic_ffs(x);
}

/**
 * ffs - find first set bit in a word
 * @x: the word to search
 *
 * This is defined the same way as the libc and compiler builtin ffs routines.
 *
 * ffs(value) returns 0 if value is 0 or the position of the first set bit if
 * value is nonzero. The first (least significant) bit is at position 1.
 */
#define ffs(x) (__builtin_constant_p(x) ? __builtin_ffs(x) : variable_ffs(x))

static __always_inline int variable_fls(unsigned int x)
{
	asm goto(ALTERNATIVE("j %l[legacy]", "nop", 0,
				      RISCV_ISA_EXT_ZBB, 1)
			  : : : : legacy);

	if (!x)
		return 0;

	asm volatile (".option push\n"
		      ".option arch,+zbb\n"
		      CLZW "%0, %1\n"
		      ".option pop\n"
		      : "=r" (x) : "r" (x) :);

	return 32 - x;

legacy:
	return generic_fls(x);
}

/**
 * fls - find last set bit in a word
 * @x: the word to search
 *
 * This is defined in a similar way as ffs, but returns the position of the most
 * significant set bit.
 *
 * fls(value) returns 0 if value is 0 or the position of the last set bit if
 * value is nonzero. The last (most significant) bit is at position 32.
 */
#define fls(x)							\
({								\
	typeof(x) x_ = (x);					\
	__builtin_constant_p(x_) ?				\
	 ((x_ != 0) ? (32 - __builtin_clz(x_)) : 0)		\
	 :							\
	 variable_fls(x_);					\
})

#endif /* !(defined(CONFIG_RISCV_ISA_ZBB) && defined(CONFIG_TOOLCHAIN_HAS_ZBB)) || defined(NO_ALTERNATIVE) */

#include <asm-generic/bitops/ffz.h>
#include <asm-generic/bitops/fls64.h>
#include <asm-generic/bitops/sched.h>

#include <asm/arch_hweight.h>

#include <asm-generic/bitops/const_hweight.h>

/*
 * IRQ-safe atomic bitops for single-hart cores without the A extension.
 * Disabling interrupts guarantees atomicity since there is no other hart.
 */

static __always_inline int arch_test_and_set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags, __res, __mask = BIT_MASK(nr);
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	__res = *__p;
	*__p = __res | __mask;
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
	return (__res & __mask) != 0;
}

static __always_inline int arch_test_and_clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags, __res, __mask = BIT_MASK(nr);
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	__res = *__p;
	*__p = __res & ~__mask;
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
	return (__res & __mask) != 0;
}

static __always_inline int arch_test_and_change_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags, __res, __mask = BIT_MASK(nr);
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	__res = *__p;
	*__p = __res ^ __mask;
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
	return (__res & __mask) != 0;
}

static __always_inline void arch_set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags;
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	*__p |= BIT_MASK(nr);
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
}

static __always_inline void arch_clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags;
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	*__p &= ~BIT_MASK(nr);
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
}

static __always_inline void arch_change_bit(int nr, volatile unsigned long *addr)
{
	unsigned long __flags;
	volatile unsigned long *__p = &addr[BIT_WORD(nr)];

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	*__p ^= BIT_MASK(nr);
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
}

static __always_inline int arch_test_and_set_bit_lock(
	unsigned long nr, volatile unsigned long *addr)
{
	return arch_test_and_set_bit(nr, addr);
}

static __always_inline void arch_clear_bit_unlock(
	unsigned long nr, volatile unsigned long *addr)
{
	arch_clear_bit(nr, addr);
}

static __always_inline void arch___clear_bit_unlock(
	unsigned long nr, volatile unsigned long *addr)
{
	arch_clear_bit_unlock(nr, addr);
}

static __always_inline bool arch_xor_unlock_is_negative_byte(unsigned long mask,
		volatile unsigned long *addr)
{
	unsigned long __flags, __res;

	raw_local_irq_save(__flags);
	__cmpxchg_fence();
	__res = *addr;
	*addr = __res ^ mask;
	__cmpxchg_fence();
	raw_local_irq_restore(__flags);
	return (__res & BIT(7)) != 0;
}

#include <asm-generic/bitops/instrumented-atomic.h>
#include <asm-generic/bitops/instrumented-lock.h>

#include <asm-generic/bitops/non-atomic.h>
#include <asm-generic/bitops/le.h>
#include <asm-generic/bitops/ext2-atomic.h>

#endif /* _ASM_RISCV_BITOPS_H */
