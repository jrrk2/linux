/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Regents of the University of California
 *
 * IRQ-safe (non-AMO/LR/SC) cmpxchg/xchg for single-hart cores without A ext.
 * Uses fence rw,rw to ensure HyperRAM write buffers are flushed.
 */

#ifndef _ASM_RISCV_CMPXCHG_H
#define _ASM_RISCV_CMPXCHG_H

#include <linux/bug.h>
#include <linux/irqflags.h>

/*
 * Memory fence to ensure all prior reads/writes are visible.
 * Needed on systems with write-buffered external RAM (e.g., HyperRAM).
 */
#define __cmpxchg_fence()	__asm__ __volatile__("fence rw, rw" ::: "memory")

#define __xchg(ptr, new, size)						\
({									\
	__typeof__(*(ptr)) __ret;					\
	unsigned long __flags;						\
	switch (size) {							\
	case 1: {							\
		volatile u8 *__p = (volatile u8 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		*__p = (u8)(unsigned long)(new);			\
		__cmpxchg_fence();						\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	case 2: {							\
		volatile u16 *__p = (volatile u16 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		*__p = (u16)(unsigned long)(new);			\
		__cmpxchg_fence();						\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	case 4: {							\
		volatile u32 *__p = (volatile u32 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		*__p = (u32)(unsigned long)(new);			\
		__cmpxchg_fence();						\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	default:							\
		BUILD_BUG();						\
		__ret = (__typeof__(*(ptr)))0;				\
	}								\
	__ret;								\
})

#define arch_xchg_relaxed(ptr, x)	__xchg((ptr), (x), sizeof(*(ptr)))
#define arch_xchg_acquire(ptr, x)	__xchg((ptr), (x), sizeof(*(ptr)))
#define arch_xchg_release(ptr, x)	__xchg((ptr), (x), sizeof(*(ptr)))
#define arch_xchg(ptr, x)		__xchg((ptr), (x), sizeof(*(ptr)))

#define xchg32(ptr, x)							\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 4);				\
	arch_xchg((ptr), (x));						\
})

#define __cmpxchg(ptr, old, new, size)					\
({									\
	__typeof__(*(ptr)) __ret;					\
	unsigned long __flags;						\
	switch (size) {							\
	case 1: {							\
		volatile u8 *__p = (volatile u8 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		if ((u8)(unsigned long)__ret == (u8)(unsigned long)(old)) { \
			*__p = (u8)(unsigned long)(new);		\
			__cmpxchg_fence();					\
		}							\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	case 2: {							\
		volatile u16 *__p = (volatile u16 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		if ((u16)(unsigned long)__ret == (u16)(unsigned long)(old)) { \
			*__p = (u16)(unsigned long)(new);		\
			__cmpxchg_fence();					\
		}							\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	case 4: {							\
		volatile u32 *__p = (volatile u32 *)(ptr);		\
		raw_local_irq_save(__flags);				\
		__cmpxchg_fence();						\
		__ret = (__typeof__(*(ptr)))(unsigned long)*__p;		\
		if ((u32)(unsigned long)__ret == (u32)(unsigned long)(old)) { \
			*__p = (u32)(unsigned long)(new);		\
			__cmpxchg_fence();					\
		}							\
		raw_local_irq_restore(__flags);				\
		break;							\
	}								\
	default:							\
		BUILD_BUG();						\
		__ret = (__typeof__(*(ptr)))0;				\
	}								\
	__ret;								\
})

#define arch_cmpxchg_relaxed(ptr, o, n)	__cmpxchg((ptr), (o), (n), sizeof(*(ptr)))
#define arch_cmpxchg_acquire(ptr, o, n)	__cmpxchg((ptr), (o), (n), sizeof(*(ptr)))
#define arch_cmpxchg_release(ptr, o, n)	__cmpxchg((ptr), (o), (n), sizeof(*(ptr)))
#define arch_cmpxchg(ptr, o, n)		__cmpxchg((ptr), (o), (n), sizeof(*(ptr)))

#define arch_cmpxchg_local(ptr, o, n)	arch_cmpxchg_relaxed((ptr), (o), (n))

#define arch_cmpxchg64(ptr, o, n)					\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg((ptr), (o), (n));					\
})

#define arch_cmpxchg64_local(ptr, o, n)					\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg_relaxed((ptr), (o), (n));				\
})

#define arch_cmpxchg64_relaxed(ptr, o, n)				\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg_relaxed((ptr), (o), (n));				\
})

#define arch_cmpxchg64_acquire(ptr, o, n)				\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg_acquire((ptr), (o), (n));				\
})

#define arch_cmpxchg64_release(ptr, o, n)				\
({									\
	BUILD_BUG_ON(sizeof(*(ptr)) != 8);				\
	arch_cmpxchg_release((ptr), (o), (n));				\
})

#endif /* _ASM_RISCV_CMPXCHG_H */
