/*
 * Minimal init for Sonata nommu Linux.
 * No libc — just raw RISC-V syscalls.
 */

static long sys_write(int fd, const char *buf, long len)
{
	register long a0 asm("a0") = fd;
	register long a1 asm("a1") = (long)buf;
	register long a2 asm("a2") = len;
	register long a7 asm("a7") = 64; /* __NR_write */

	asm volatile("ecall"
		: "+r"(a0)
		: "r"(a1), "r"(a2), "r"(a7)
		: "memory");
	return a0;
}

static void sys_pause(void)
{
	register long a7 asm("a7") = 29; /* __NR_pause */

	asm volatile("ecall" : : "r"(a7) : "memory");
}

static int strlen(const char *s)
{
	int n = 0;
	while (*s++) n++;
	return n;
}

static void puts(const char *s)
{
	sys_write(1, s, strlen(s));
}

void _start(void) __attribute__((noreturn));
void _start(void)
{
	puts("\n");
	puts("========================================\n");
	puts("  Sonata Linux - lowRISC Ibex RV32IMC\n");
	puts("  nommu, M-mode, single hart\n");
	puts("========================================\n");
	puts("\n");
	puts("Init running! Kernel boot successful.\n");
	puts("\n");

	/* Init must never exit — PID 1 dying panics the kernel */
	puts("Init idle.\n");
	for (;;)
		sys_pause();
}
