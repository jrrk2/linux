#!/usr/bin/env python3
"""Compute the additive u32 checksum of the .text section from vmlinux.

This should match the checksum printed by setup_arch() at boot time.
Usage: python3 scripts/text_checksum.py vmlinux
"""
import struct
import sys
from elftools.elf.elffile import ELFFile

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} vmlinux", file=sys.stderr)
        sys.exit(1)

    with open(sys.argv[1], 'rb') as f:
        elf = ELFFile(f)
        text = elf.get_section_by_name('.text')
        if text is None:
            print("No .text section found", file=sys.stderr)
            sys.exit(1)

        data = text.data()
        # Pad to 4-byte alignment
        if len(data) % 4:
            data += b'\x00' * (4 - len(data) % 4)

        checksum = 0
        for (word,) in struct.iter_unpack('<I', data):
            checksum = (checksum + word) & 0xFFFFFFFF

        print(f"text checksum: 0x{checksum:08x} ({len(text.data())} bytes)")

if __name__ == '__main__':
    main()
