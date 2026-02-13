make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j8 vmlinux
python3 scripts/patch_text_checksum.py vmlinux
