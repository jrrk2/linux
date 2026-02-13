#!/usr/bin/env python3
"""Compute .text checksum and patch it into the expected_text_checksum symbol.

The symbol must be in .data (not .text) so patching it doesn't change the
checksum.  After patching, the kernel's setup_arch() will verify the .text
section at boot and report OK or MISMATCH.

Usage: python3 scripts/patch_text_checksum.py vmlinux
"""
import struct
import sys
from elftools.elf.elffile import ELFFile


def find_symbol(elf, name):
    """Return (vaddr, size) for a symbol, searching all symtab sections."""
    for section in elf.iter_sections():
        if not hasattr(section, 'get_symbol_by_name'):
            continue
        syms = section.get_symbol_by_name(name)
        if syms:
            sym = syms[0]
            return sym['st_value'], sym['st_size']
    return None, None


def vaddr_to_offset(elf, vaddr):
    """Convert a virtual address to a file offset using program headers."""
    for seg in elf.iter_segments():
        if seg['p_type'] != 'PT_LOAD':
            continue
        start = seg['p_vaddr']
        end = start + seg['p_filesz']
        if start <= vaddr < end:
            return seg['p_offset'] + (vaddr - start)
    return None


def compute_text_checksum(elf):
    """Compute additive u32 checksum of .text section."""
    text = elf.get_section_by_name('.text')
    if text is None:
        print("error: no .text section found", file=sys.stderr)
        sys.exit(1)

    data = text.data()
    if len(data) % 4:
        data += b'\x00' * (4 - len(data) % 4)

    checksum = 0
    for (word,) in struct.iter_unpack('<I', data):
        checksum = (checksum + word) & 0xFFFFFFFF

    return checksum, len(text.data())


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} vmlinux", file=sys.stderr)
        sys.exit(1)

    vmlinux = sys.argv[1]

    # First pass: read-only, compute checksum and find symbol
    with open(vmlinux, 'rb') as f:
        elf = ELFFile(f)
        checksum, text_size = compute_text_checksum(elf)

        vaddr, sym_size = find_symbol(elf, 'expected_text_checksum')
        if vaddr is None:
            print("error: symbol 'expected_text_checksum' not found",
                  file=sys.stderr)
            sys.exit(1)

        file_offset = vaddr_to_offset(elf, vaddr)
        if file_offset is None:
            print(f"error: cannot map vaddr 0x{vaddr:08x} to file offset",
                  file=sys.stderr)
            sys.exit(1)

        # Verify the symbol is NOT in .text
        text = elf.get_section_by_name('.text')
        text_start = text['sh_addr']
        text_end = text_start + text['sh_size']
        if text_start <= vaddr < text_end:
            print("error: expected_text_checksum is in .text — "
                  "must be in .data", file=sys.stderr)
            sys.exit(1)

    # Second pass: patch the value
    with open(vmlinux, 'r+b') as f:
        f.seek(file_offset)
        old_val = struct.unpack('<I', f.read(4))[0]
        f.seek(file_offset)
        f.write(struct.pack('<I', checksum))

    print(f"text checksum: 0x{checksum:08x} ({text_size} bytes)")
    print(f"patched expected_text_checksum at file offset 0x{file_offset:x} "
          f"(vaddr 0x{vaddr:08x}): 0x{old_val:08x} -> 0x{checksum:08x}")


if __name__ == '__main__':
    main()
