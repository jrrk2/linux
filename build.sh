make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j8 Image.lz4
python3 scripts/patch_text_checksum.py vmlinux
cp arch/riscv/boot/Image.lz4 /srv/tftp/
