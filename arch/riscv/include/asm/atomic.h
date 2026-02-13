/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2007 Red Hat, Inc. All Rights Reserved.
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 *
 * IRQ-safe (non-AMO) atomics for single-hart cores without the A extension.
 */

#ifndef _ASM_RISCV_ATOMIC_H
#define _ASM_RISCV_ATOMIC_H

#ifdef CONFIG_GENERIC_ATOMIC64
# include <asm-generic/atomic64.h>
#else
# if (__riscv_xlen < 64)
#  error "64-bit atomics require XLEN to be at least 64"
# endif
#endif

#include <asm/cmpxchg.h>
#include <linux/irqflags.h>

#define __atomic_acquire_fence()	__cmpxchg_fence()
#define __atomic_release_fence()	__cmpxchg_fence()

static __always_inline int arch_atomic_read(const atomic_t *v)
{
	int val;
	__cmpxchg_fence();
	val = READ_ONCE(v->counter);
	return val;
}
static __always_inline void arch_atomic_set(atomic_t *v, int i)
{
	WRITE_ONCE(v->counter, i);
	__cmpxchg_fence();
}

#ifndef CONFIG_GENERIC_ATOMIC64
#define ATOMIC64_INIT(i) { (i) }
static __always_inline s64 arch_atomic64_read(const atomic64_t *v)
{
	return READ_ONCE(v->counter);
}
static __always_inline void arch_atomic64_set(atomic64_t *v, s64 i)
{
	WRITE_ONCE(v->counter, i);
}
#endif

/*
 * IRQ-safe atomic operations for single-hart systems.
 * Disabling interrupts guarantees atomicity since there is no other hart.
 */

/* Fire-and-forget ops: add, sub, and, or, xor */
#define ATOMIC_OP(op, c_op, I, c_type, prefix)				\
static __always_inline							\
void arch_atomic##prefix##_##op(c_type i, atomic##prefix##_t *v)	\
{									\
	unsigned long __flags;						\
	raw_local_irq_save(__flags);					\
	__cmpxchg_fence();						\
	v->counter = v->counter c_op (I);				\
	__cmpxchg_fence();						\
	raw_local_irq_restore(__flags);					\
}

#ifdef CONFIG_GENERIC_ATOMIC64
#define ATOMIC_OPS(op, c_op, I)						\
	ATOMIC_OP(op, c_op, I, int,   )
#else
#define ATOMIC_OPS(op, c_op, I)						\
	ATOMIC_OP(op, c_op, I, int,   )				\
	ATOMIC_OP(op, c_op, I, s64, 64)
#endif

ATOMIC_OPS(add, +,  i)
ATOMIC_OPS(sub, +, -i)
ATOMIC_OPS(and, &,  i)
ATOMIC_OPS( or, |,  i)
ATOMIC_OPS(xor, ^,  i)

#undef ATOMIC_OP
#undef ATOMIC_OPS

/* Fetch ops (return OLD value) and return ops (return NEW value) */
#define ATOMIC_FETCH_OP(op, c_op, I, c_type, prefix)			\
static __always_inline							\
c_type arch_atomic##prefix##_fetch_##op##_relaxed(c_type i,		\
					     atomic##prefix##_t *v)	\
{									\
	c_type __ret;							\
	unsigned long __flags;						\
	raw_local_irq_save(__flags);					\
	__cmpxchg_fence();						\
	__ret = v->counter;						\
	v->counter = __ret c_op (I);					\
	__cmpxchg_fence();						\
	raw_local_irq_restore(__flags);					\
	return __ret;							\
}									\
static __always_inline							\
c_type arch_atomic##prefix##_fetch_##op(c_type i, atomic##prefix##_t *v) \
{									\
	return arch_atomic##prefix##_fetch_##op##_relaxed(i, v);	\
}

#define ATOMIC_OP_RETURN(op, c_op, I, c_type, prefix)			\
static __always_inline							\
c_type arch_atomic##prefix##_##op##_return_relaxed(c_type i,		\
					      atomic##prefix##_t *v)	\
{									\
	return arch_atomic##prefix##_fetch_##op##_relaxed(i, v) c_op (I); \
}									\
static __always_inline							\
c_type arch_atomic##prefix##_##op##_return(c_type i, atomic##prefix##_t *v) \
{									\
	return arch_atomic##prefix##_fetch_##op(i, v) c_op (I);	\
}

#ifdef CONFIG_GENERIC_ATOMIC64
#define ATOMIC_OPS(op, c_op, I)						\
        ATOMIC_FETCH_OP( op, c_op, I, int,   )				\
        ATOMIC_OP_RETURN(op, c_op, I, int,   )
#else
#define ATOMIC_OPS(op, c_op, I)						\
        ATOMIC_FETCH_OP( op, c_op, I, int,   )				\
        ATOMIC_OP_RETURN(op, c_op, I, int,   )				\
        ATOMIC_FETCH_OP( op, c_op, I, s64, 64)				\
        ATOMIC_OP_RETURN(op, c_op, I, s64, 64)
#endif

ATOMIC_OPS(add, +,  i)
ATOMIC_OPS(sub, +, -i)

#define arch_atomic_add_return_relaxed	arch_atomic_add_return_relaxed
#define arch_atomic_sub_return_relaxed	arch_atomic_sub_return_relaxed
#define arch_atomic_add_return		arch_atomic_add_return
#define arch_atomic_sub_return		arch_atomic_sub_return

#define arch_atomic_fetch_add_relaxed	arch_atomic_fetch_add_relaxed
#define arch_atomic_fetch_sub_relaxed	arch_atomic_fetch_sub_relaxed
#define arch_atomic_fetch_add		arch_atomic_fetch_add
#define arch_atomic_fetch_sub		arch_atomic_fetch_sub

#ifndef CONFIG_GENERIC_ATOMIC64
#define arch_atomic64_add_return_relaxed	arch_atomic64_add_return_relaxed
#define arch_atomic64_sub_return_relaxed	arch_atomic64_sub_return_relaxed
#define arch_atomic64_add_return		arch_atomic64_add_return
#define arch_atomic64_sub_return		arch_atomic64_sub_return

#define arch_atomic64_fetch_add_relaxed	arch_atomic64_fetch_add_relaxed
#define arch_atomic64_fetch_sub_relaxed	arch_atomic64_fetch_sub_relaxed
#define arch_atomic64_fetch_add		arch_atomic64_fetch_add
#define arch_atomic64_fetch_sub		arch_atomic64_fetch_sub
#endif

#undef ATOMIC_OPS

#ifdef CONFIG_GENERIC_ATOMIC64
#define ATOMIC_OPS(op, c_op, I)						\
        ATOMIC_FETCH_OP(op, c_op, I, int,   )
#else
#define ATOMIC_OPS(op, c_op, I)						\
        ATOMIC_FETCH_OP(op, c_op, I, int,   )				\
        ATOMIC_FETCH_OP(op, c_op, I, s64, 64)
#endif

ATOMIC_OPS(and, &, i)
ATOMIC_OPS( or, |, i)
ATOMIC_OPS(xor, ^, i)

#define arch_atomic_fetch_and_relaxed	arch_atomic_fetch_and_relaxed
#define arch_atomic_fetch_or_relaxed	arch_atomic_fetch_or_relaxed
#define arch_atomic_fetch_xor_relaxed	arch_atomic_fetch_xor_relaxed
#define arch_atomic_fetch_and		arch_atomic_fetch_and
#define arch_atomic_fetch_or		arch_atomic_fetch_or
#define arch_atomic_fetch_xor		arch_atomic_fetch_xor

#ifndef CONFIG_GENERIC_ATOMIC64
#define arch_atomic64_fetch_and_relaxed	arch_atomic64_fetch_and_relaxed
#define arch_atomic64_fetch_or_relaxed	arch_atomic64_fetch_or_relaxed
#define arch_atomic64_fetch_xor_relaxed	arch_atomic64_fetch_xor_relaxed
#define arch_atomic64_fetch_and		arch_atomic64_fetch_and
#define arch_atomic64_fetch_or		arch_atomic64_fetch_or
#define arch_atomic64_fetch_xor		arch_atomic64_fetch_xor
#endif

#undef ATOMIC_OPS
#undef ATOMIC_FETCH_OP
#undef ATOMIC_OP_RETURN

/* fetch_add_unless: add unless the value equals u */
static __always_inline int arch_atomic_fetch_add_unless(atomic_t *v, int a, int u)
{
	int prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev != u) {
		v->counter = prev + a;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev;
}
#define arch_atomic_fetch_add_unless arch_atomic_fetch_add_unless

#ifndef CONFIG_GENERIC_ATOMIC64
static __always_inline s64 arch_atomic64_fetch_add_unless(atomic64_t *v, s64 a, s64 u)
{
	s64 prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev != u) {
		v->counter = prev + a;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev;
}
#define arch_atomic64_fetch_add_unless arch_atomic64_fetch_add_unless
#endif

static __always_inline bool arch_atomic_inc_unless_negative(atomic_t *v)
{
	int prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev >= 0) {
		v->counter = prev + 1;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev >= 0;
}
#define arch_atomic_inc_unless_negative arch_atomic_inc_unless_negative

static __always_inline bool arch_atomic_dec_unless_positive(atomic_t *v)
{
	int prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev <= 0) {
		v->counter = prev - 1;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev <= 0;
}
#define arch_atomic_dec_unless_positive arch_atomic_dec_unless_positive

static __always_inline int arch_atomic_dec_if_positive(atomic_t *v)
{
	int prev, dec;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	dec = prev - 1;
	if (dec >= 0) {
		v->counter = dec;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return dec;
}
#define arch_atomic_dec_if_positive arch_atomic_dec_if_positive

#ifndef CONFIG_GENERIC_ATOMIC64
static __always_inline bool arch_atomic64_inc_unless_negative(atomic64_t *v)
{
	s64 prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev >= 0) {
		v->counter = prev + 1;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev >= 0;
}
#define arch_atomic64_inc_unless_negative arch_atomic64_inc_unless_negative

static __always_inline bool arch_atomic64_dec_unless_positive(atomic64_t *v)
{
	s64 prev;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	if (prev <= 0) {
		v->counter = prev - 1;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return prev <= 0;
}
#define arch_atomic64_dec_unless_positive arch_atomic64_dec_unless_positive

static __always_inline s64 arch_atomic64_dec_if_positive(atomic64_t *v)
{
	s64 prev, dec;
	unsigned long flags;

	raw_local_irq_save(flags);
	__cmpxchg_fence();
	prev = v->counter;
	dec = prev - 1;
	if (dec >= 0) {
		v->counter = dec;
		__cmpxchg_fence();
	}
	raw_local_irq_restore(flags);
	return dec;
}
#define arch_atomic64_dec_if_positive	arch_atomic64_dec_if_positive
#endif

#endif /* _ASM_RISCV_ATOMIC_H */
