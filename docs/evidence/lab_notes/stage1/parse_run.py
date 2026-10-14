"""Turn a J-Link Commander watchpoint-loop log into one table row per halt.

Usage: python3 parse_run.py runA.log > runA_table.txt

Each loop iteration in the .jlink script is: g, WaitHalt 300, MoE, h, then
reads of GPIOA/GPIOB MODER and ODR. A row reports why the core stopped, the
PC, the four registers, and whether any SD pin (SD1 PB0, SD2 PA1, SD3 PA2,
SD4 PA0) was an output driving high after that write.
"""

import re
import sys

SD_PINS = {"A": [(0, "SD4 PA0"), (1, "SD2 PA1"), (2, "SD3 PA2")], "B": [(0, "SD1 PB0")]}


def reason_of(block):
    # The last split chunk is the final resume after cleanup, not a loop step.
    if "J-Link>MoE" not in block:
        return "final resume (not a loop iteration)"
    if "halted due to data breakpoint" in block:
        return "watchpoint"
    if "halted due to vector catch" in block:
        return "vector catch"
    if "CPU is not halted" in block:
        return "no write within 300 ms (halted by debugger)"
    return "?"


def main():
    with open(sys.argv[1]) as f:
        text = f.read()
    print("n | stop reason | PC | GPIOA_MODER | GPIOA_ODR | GPIOB_MODER | GPIOB_ODR | SD output high")
    for n, block in enumerate(text.split("J-Link>g\n")[1:], 1):
        pc = re.search(r"PC = ([0-9A-F]{8})", block)
        regs = dict(re.findall(r"(4800[0-9A-F]{4}) = ([0-9A-F]{8})", block))
        moder_a, odr_a, moder_b, odr_b = (
            int(regs.get(addr, "0"), 16) for addr in ("48000000", "48000014", "48000400", "48000414")
        )
        high = [
            name
            for port, moder, odr in (("A", moder_a, odr_a), ("B", moder_b, odr_b))
            for pin, name in SD_PINS[port]
            if (moder >> (2 * pin)) & 3 == 1 and (odr >> pin) & 1
        ]
        print(
            f"{n} | {reason_of(block)} | {pc.group(1) if pc else '?'} | {moder_a:08X} | {odr_a:08X} | "
            f"{moder_b:08X} | {odr_b:08X} | {', '.join(high) or 'none'}"
        )


if __name__ == "__main__":
    main()
