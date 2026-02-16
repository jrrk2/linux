/*
 * Mandelbrot fractal demo for /dev/fb0 on Sonata Linux.
 *
 * Fixed-point arithmetic ported from the bare-metal fractal_fixed.c.
 * Built with musl libc as a static PIE binary.
 *
 * Usage: fb_fractal
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define LCD_WIDTH  160
#define LCD_HEIGHT 128

#define FP_EXP  12
#define FP_MANT 15
#define MAKE_FP(i, f, f_bits) ((i << FP_EXP) | (f << (FP_EXP - f_bits)))
#define MAX_ITERS 50

typedef unsigned short u16;
typedef int i32;

/* ---- Fixed-point arithmetic (from fractal_fixed.c) ---- */

static i32 fp_clamp(i32 x)
{
	if (x < 0 && x < -(1 << FP_MANT))
		return -(1 << FP_MANT);
	if (x > 0 && x >= (1 << FP_MANT))
		return (1 << FP_MANT) - 1;
	return x;
}

static i32 fp_mul(i32 a, i32 b) { return fp_clamp((a * b) >> FP_EXP); }
static i32 fp_add(i32 a, i32 b) { return fp_clamp(a + b); }

static int mandel_iters(i32 cr, i32 ci)
{
	i32 zr = cr, zi = ci;

	for (int i = 0; i < MAX_ITERS; i++) {
		i32 zr2 = fp_mul(zr, zr);
		i32 zi2 = fp_mul(zi, zi);
		if (zr2 + zi2 > MAKE_FP(4, 0, 0))
			return i;
		zi = fp_add(fp_mul(zr, zi) * 2, ci);
		zr = fp_add(zr2 - zi2, cr);
	}
	return MAX_ITERS;
}

/*
 * Palette: 51 entries, BGR565 format.
 * Copied directly from the bare-metal fractal_palette.c.
 */
static const u16 palette[51] = {
	0x91e7, 0x7ca7, 0x5ca7, 0x44a7, 0x3ca9, 0x3cab, 0x3cad, 0x3caf,
	0x3cb1, 0x3cb2, 0x3c52, 0x3bf2, 0x3b92, 0x3b52, 0x3b12, 0x3ab2,
	0x3a72, 0x3a32, 0x39f2, 0x41f2, 0x49f2, 0x51f2, 0x59f2, 0x59f2,
	0x61f2, 0x69f2, 0x71f2, 0x79f2, 0x79f2, 0x81f2, 0x89f2, 0x91f2,
	0x91f2, 0x91f1, 0x91f1, 0x91f0, 0x91ef, 0x91ef, 0x91ee, 0x91ed,
	0x91ed, 0x91ec, 0x91eb, 0x91eb, 0x91ea, 0x91ea, 0x91e9, 0x91e9,
	0x91e8, 0x91e7, 0x91e7,
};

static u16 framebuf[LCD_WIDTH * LCD_HEIGHT];

int main(void)
{
	puts("fb_fractal: computing Mandelbrot set...");

	/* Compute fractal — same viewport as bare-metal demo */
	i32 start_real = -MAKE_FP(1, 0x3, 2);
	i32 start_imag = MAKE_FP(1, 0, 0);
	i32 inc = MAKE_FP(0, 0x40, 12);

	i32 ci = start_imag;
	for (int y = 0; y < LCD_HEIGHT; y++) {
		i32 cr = start_real;
		for (int x = 0; x < LCD_WIDTH; x++) {
			int iters = mandel_iters(cr, ci);
			framebuf[y * LCD_WIDTH + x] = palette[iters];
			cr += inc;
		}
		ci -= inc;
	}

	puts("fb_fractal: writing to /dev/fb0...");

	int fd = open("/dev/fb0", O_WRONLY);
	if (fd < 0) {
		perror("fb_fractal: open /dev/fb0");
		return 1;
	}

	ssize_t total = LCD_WIDTH * LCD_HEIGHT * 2;
	ssize_t written = 0;
	while (written < total) {
		ssize_t ret = write(fd, (char *)framebuf + written,
				    total - written);
		if (ret <= 0) {
			perror("fb_fractal: write");
			return 1;
		}
		written += ret;
	}

	close(fd);
	puts("fb_fractal: done!");
	return 0;
}
