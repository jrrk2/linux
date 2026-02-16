#!/bin/bash
# Load kernel via J-Link and boot, capturing serial output.
# Usage: ./jlink_boot.sh [vmlinux] [serial_host:port] [gdb_host:port]

VMLINUX="${1:-/home/jonathan/vivado-risc-v/linux-stable/vmlinux}"
SERIAL="${2:-192.168.1.198:4567}"
GDB_TARGET="${3:-192.168.1.198:2331}"
GDB="/home/jonathan/vivado-risc-v/workspace/gcc/riscv/bin/riscv32-unknown-elf-gdb"
SERIAL_LOG="/tmp/serial_boot.log"

SERIAL_HOST="${SERIAL%%:*}"
SERIAL_PORT="${SERIAL##*:}"

# Always start serial capture first
> "$SERIAL_LOG"
nc -w 300 "$SERIAL_HOST" "$SERIAL_PORT" >> "$SERIAL_LOG" 2>&1 &
NC_PID=$!
echo "Serial capture started (PID $NC_PID) -> $SERIAL_LOG"
sleep 1

# Load and boot via GDB
echo "Loading $VMLINUX via J-Link at $GDB_TARGET..."
$GDB -batch \
  -ex "file $VMLINUX" \
  -ex "target remote $GDB_TARGET" \
  -ex "monitor halt" \
  -ex "monitor reset" \
  -ex "load" \
  -ex "monitor reset" \
  -ex "stepi" \
  -ex "stepi" \
  -ex "stepi" \
  -ex "stepi" \
  -ex "stepi" \
  -ex "set \$pc = 0x40000000" \
  -ex "continue" \
  2>&1 | grep -v "no version"

# Serial capture continues in background
echo ""
echo "Kernel running. Serial log: tail -f $SERIAL_LOG"
echo "Serial capture PID: $NC_PID"
