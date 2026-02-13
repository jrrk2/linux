// SPDX-License-Identifier: GPL-2.0
/*
 * 64-bit unsigned division helper for RV32.
 *
 * GCC emits calls to __udivdi3 for 64-bit division on 32-bit targets.
 * Provide an implementation using the kernel's div64_u64().
 */

#include <linux/math64.h>
#include <linux/export.h>

unsigned long long __udivdi3(unsigned long long a, unsigned long long b)
{
	return div64_u64(a, b);
}
EXPORT_SYMBOL(__udivdi3);
