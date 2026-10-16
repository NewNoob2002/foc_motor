# ISR segments and CCM relocation: software evidence

Scope: task c24e1f37ea76, based on `c9ace9d`. No hardware access, flashing,
reset, or debugger operation. These are build/static-analysis results, not
measured execution time. The powered mode-4 target of about 2200 cycles and
the 3400-cycle ISR budget still require a later authorized bench run.

## Implementation

`dma_isr` reads DWT CYCCNT at entry, directly before and after its callback,
and at the existing execution-time endpoint. The report-window maxima are
`pre_max`, `step_max`, and `post_max`; the existing single-writer snapshot and
reset request cover all three. A skipped callback contributes zero pre/step
and attributes the elapsed time to post. The separate maxima need not come
from the same interrupt. Unsigned differences handle one CYCCNT wrap.

The 1 Hz report prints `segments pre/step/post max N/N/N cyc` only while the
DWT window is valid. There are two additional inline counter reads, unsigned
subtractions, and three max/store operations; no new calls or division.
Disassembly confirms that the counter reads are register loads. As before,
the execution endpoint precedes the final statistics/overrun bookkeeping and
IRQ epilogue; these figures are not the complete exception-entry/return cost.
Actual instrumentation overhead has not been measured on hardware.

The board now exposes SRAM1+SRAM2 as 22 KiB `sram0` and defines a separate
10 KiB `CCM` region at `0x10000000`. The upstream 32 KiB sram0 previously
included the CCM alias. Addresses/sizes and I-code/D-code access were checked
against the repository's STM32G431 datasheet, **DS12589 Rev 1, section 3.5,
page 18**, with the bus matrix in section 3.6, page 19. The SDK device header
also defines `CCMSRAM_BASE=0x10000000`, `CCMSRAM_SIZE=0x2800`.
Board comments cite RM0440's memory map/embedded SRAM and DS12589.

The specification's CPU-only restriction applies to the `0x10000000`
window. DS12589 explicitly says the `0x20005800` alias is accessible by
CPU/DMA1/DMA2 through the System bus. Excluding the alias prevents overlapping
allocations; no DMA buffers or other data/bss are relocated.

Zephyr v4.4.2's `doc/kernel/code-relocation.rst`, the
`code_relocation_nocopy` sample, `cmake/modules/extensions.cmake`,
`gen_relocate_app.py`, and memory-region linker support were inspected.
`CONFIG_CODE_DATA_RELOCATION=y` plus the ARM-selected
`CONFIG_ARCH_HAS_CODE_DATA_RELOCATION=y` is sufficient. The SRAM-specific
MPU option targets `__ram_text_reloc_*`, not this CCM region, and is left off.
CCM is outside the configured `0x20000000` SRAM MPU region, in the executable
Code area of the Cortex-M default map; Zephyr's MPU enable uses PRIVDEFENA.
Stack protection and the existing MPU configuration remain enabled.

Relocated TEXT:

- `src/foc_ctrl.c`, `src/motor_ident.c`, `src/power_guard.c`;
- all four `lib/foc/src/*.c` files;
- attributed ISR wrapper/body and ISR helpers in `src/pwm_adc.c`, including
  the generated out-of-line atomic helper;
- `encoder_count()` from `src/encoder.c`, needed by every interrupt.

The first whole-file build exceeded CCM by 744 bytes. Per-function section
attributes in `pwm_adc.c` keep setup and thread APIs in Flash. Zephyr's
section filter selects these attributes and compiler atomic helpers.
`NOKEEP` retains ordinary garbage collection. No pure-C FOC source changed.
RODATA remains in Flash: there is no hardware measurement justifying its
optional relocation. The disassembly contains two Flash constant references:
a 12-byte default-duty constant in `foc_current_step`'s invalid-VBUS branch
and the 20-byte identification target table in `motor_ident_step`. Neither
is an instruction call into Flash; powered mode 4 does not use these loads.

## Image, startup, and call audit

`build/arm-ccm/zephyr/linker.cmd` contains:

```text
.ccm_text_reloc ... > CCM AT > FLASH
__ccm_text_reloc_start = 0x10000000
__ccm_text_reloc_end   = 0x10002360
__ccm_text_reloc_size  = 0x2360 (9056)
__ccm_text_rom_start   = 0x080001bc
```

The generated `zephyr/code_relocation.c` copies 9056 bytes using
`arch_early_memcpy`. Startup ordering is:

```text
z_prep_c -> arch_data_copy -> data_copy_xip_relocation -> arch_early_memcpy
         -> z_arm_interrupt_init -> z_cstart
         -> main/report_loop -> sincos_init -> foc_bench_run
         -> foc_ctrl_init -> pwm_adc_init(foc_ctrl_step)
```

The boot-source ordering and final disassembly agree: `z_prep_c` calls
`arch_data_copy` at `0x08007b02`, before `z_cstart` at `0x08007b0a`.
`data_copy_xip_relocation` at `0x0800b134` loads size `0x2360`, source
`0x080001bc`, and destination `0x10000000`. Thus the copy precedes both the
boot bench and IRQ enable. The boot bench remains built and its calls resolve
to CCM through linker veneers; it was not executed in this hardware-free task.

Required `arm-zephyr-eabi-nm` results:

| Symbol | Address |
| --- | --- |
| encoder_count | 0x10000000 |
| foc_ctrl_step | 0x1000014c |
| foc_current_step | 0x10000f2c |
| foc_current_step_ff | 0x10001010 |
| foc_current_step_lim | 0x1000112c |
| foc_pi_step | 0x10001226 |
| foc_svpwm | 0x10001294 |
| foc_clarke | 0x10001374 |
| foc_inv_park | 0x100013c8 |
| foc_park | 0x100013e2 |
| motor_ident_step | 0x10001550 |
| dma_isr | 0x10001fdc |
| adc_buf (6-byte DMA destination) | 0x20001838 |
| foc_ctrl_cmd | 0x200015a8 |

All direct branches/calls reachable from `dma_isr` and `foc_ctrl_step` stay
in CCM. The sole indirect call is `dma_isr`'s `blx r2` at `0x100020ea`;
`pwm_adc_init` assigns the callback to `foc_ctrl_step` before enabling IRQ.
All CCM-to-Flash calls found in the complete relocated section are:

| Call site | Destination | Context |
| --- | --- | --- |
| foc_ctrl_init, 0x100000c0 | memset, 0x0800edd0, through veneer 0x10002350 | Startup/boot bench only |
| foc_ctrl_init, 0x100000e4 | memset, 0x0800edd0, through veneer 0x10002350 | Startup/boot bench only |
| foc_ctrl_snapshot, 0x1000012e | memcpy, 0x0800edb4, through veneer 0x10002358 | Report thread only |

These intentional cold-path library calls are outside the per-period ISR
call graph. No libm, kernel, device, logging, allocation, or other Flash
function is called by that graph. Atomic operations are inline instructions
or relocated helpers.

## Resources and CI

| Quantity | Baseline | CCM build | Change |
| --- | ---: | ---: | ---: |
| GNU size text | 76244 | 76640 | +396 |
| GNU size data | 564 | 564 | 0 |
| GNU size bss | 10764 | 10776 | +12 |
| Flash, linker usage | 76812 | 77208 | +396 |
| SRAM1/2, linker usage including padding/stacks | 11392 | 11392 | 0 |
| CCM, linker usage including veneers | 0 | 9056 | +9056 |

CCM has 1184 bytes free (88.44% used). SRAM1/2 has 11136 bytes free
(50.57% used); `_image_ram_end=0x20002c80`, with the region ending at
`0x20005800`. The 12 extra statistics bytes consume existing alignment slack.
The separate generated `SRAM0` region aliases the ordinary `RAM` region and
reports zero additional allocations; it is not extra physical memory.

GNU size classifies `.ccm_text_reloc` as executable **text**, so the existing
`text+data` Flash gate already charges its Flash load copy. CI now additionally
checks CCM against 10240 bytes and tightens SRAM data+bss from 24576 to
22528 bytes. The Flash limit stays 98304 bytes. Linker region limits also
check physical occupancy including alignment; the existing size sums do not
include every linker padding byte. No limit was increased, and no diagnostic
or verification-policy file was relaxed.

The exact workflow gate body was extracted and executed with only its build
path changed from `build/arm/` to `build/arm-ccm/`. Observed output:

```text
budgets: flash 77204/98304 B; SRAM data+bss 11340/22528 B; CCM 9056/10240 B
```

All artifact, allocation-symbol, four shutdown hog, heap, stack, MPU stack
protection, and BUILD_VERSION checks passed (exit 0). The Zephyr baseline
commit check passed; its checkout has no local changes.

## Commands and observed verification

Build and test commands (all successful final runs):

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-ccm-baseline . -- -DBUILD_VERSION=v4.4.2
cmake -S tests/host/foc -B build/host-ccm
cmake --build build/host-ccm
ctest --test-dir build/host-ccm --output-on-failure
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-ccm . -- -DBUILD_VERSION=v4.4.2
clang-format --dry-run --Werror src/pwm_adc.c src/pwm_adc.h src/main.c
clang-tidy -p build/ccm-tidy src/pwm_adc.c src/main.c
bash build/arm-ccm/ci-gates.sh
git diff --check
```

The unchanged host suite passes **9/9**, with warnings-as-errors and
ASan/UBSan. The ARM build passes with Zephyr v4.4.2
`dccb09599635bdff17633fa7e9dab014b91dce90` and SDK 1.0.1. Formatting and
whitespace checks pass. `clang-format` and `clang-tidy` are version 23.1.1.

The analysis-only compilation database retains the ARM target, includes, and
defines; it adds `--target=arm-zephyr-eabi` and removes only the five
Clang-unsupported GCC flags documented in earlier repository evidence:
`-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`,
`--param=min-pagesize=0`, and `-specs=picolibc.specs`. No build flags or check
configuration were changed. clang-tidy exits 0, with the known stack-macro
branch-clone diagnostic at `main.c:56` (the specification calls it line 55),
37 Zephyr logging-macro signed-bitwise diagnostics, and the additional
preexisting warnings listed below.

Additional inspection commands used SDK `arm-zephyr-eabi-size` (summary and
`-A`), `arm-zephyr-eabi-nm` (`-n` and `-S --size-sort`), and
`arm-zephyr-eabi-objdump -d`. A local Python audit asserted every required
symbol's CCM address, DMA buffer placement, physical region sizes, and walked
the disassembly call graph from the ISR/control roots. It passed with no
Flash target on the per-period path. `pdftotext -layout` extracted the local
datasheet; `rg` and file reads traced source/callers, Zephyr relocation,
startup, MPU, and generated artifacts. Logs and analysis-only artifacts are
under ignored `build/`.

The requested command-block query produced:

```sh
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm build/arm-ccm/zephyr/zephyr.elf | grep foc_ctrl_cmd
# 200015a8 B foc_ctrl_cmd
```

## Deviations and remaining verification

- No real boot bench, segment timing, powered WCET, or HIL result can be
  reported because hardware operations are explicitly prohibited. Static
  startup and bench-call checks are complete; runtime behavior is unverified.
- clang-tidy's requested exception list is not fully met: `pwm_adc.c` has
  **10 preexisting** `bugprone-signed-bitwise` diagnostics from GPIO hog
  flags (4), `STM32_CLOCK_INFO` (4), and `IRQ_DIRECT_CONNECT`/
  `IRQ_ZERO_LATENCY` (2). Running `git show HEAD:src/pwm_adc.c` as a baseline
  translation unit with the same analysis settings reproduces all 10.
  These were neither hidden nor disabled; vendor code and unrelated BSP
  clock/IRQ construction were preserved.
- RM0440 was not present locally. Three official ST download URLs were
  attempted with `curl -L --fail --max-time`; each returned HTTP 567.
  Therefore direct RM0440 inspection is incomplete. Addresses, sizes,
  bus access, and the DMA-accessible alias are corroborated by local
  DS12589 sections 3.5/3.6 and SDK headers; zero-wait CCM execution remains
  the specification's hardware premise, not a measurement from this task.
- The agent-rule reference `/home/gtc/.pi/agent/references/embedded-development.md`
  is absent. Its read failed; no matching replacement was found in the
  available agent directories. Repository rules and Zephyr source were read.
- `encoder_count` relocation extends the named-file list to avoid a Flash
  call on every ISR. PWM per-function relocation is the specified capacity
  fallback. RODATA relocation was optional and was not selected.
