# Stage 8 report-stack and fatal-shutdown software evidence

Base: `4d7d897`. This change addresses the reported run-3 stack overflow in firmware `75821a6`; the incident details below come from the bench report, not a new hardware run. No flashing, debugger access, or hardware state change was performed for this fix.

The report thread previously had a 1024-byte stack. After a completed mode-8 map, the first successful `cog_process()` path overflowed it. Zephyr halted with the power stage still enabled because neither the zero-latency DMA ISR nor the timer supervisor could run during the fault halt.

## Changes

- `REPORT_STACK_SIZE` is 2048. `CONFIG_MAIN_STACK_SIZE` stays 512.
- `CONFIG_INIT_STACKS=y` and `CONFIG_THREAD_STACK_INFO=y` enable `k_thread_stack_space_get()`. The report thread emits `report stack unused <bytes> B (<context>)` once per full periodic report and immediately after each `dump_map`, `dump_ident`, and `dump_capture` returns. These are historical unused-stack measurements, not instantaneous free-space readings. API failures are logged with their return code. The measurement helper is out of line and runs outside the dump/report frames.
- CI does not pin either measurement option. Its existing stack and resource gates are unchanged; neither `.github/workflows/ci.yml` nor verification policy was edited.
- `pwm_adc_fatal_off()` first sets PRIMASK, then reuses `power_off()` to drive SD1-SD3 low before making PC0-PC2 analog. It then clears TIM1 BDTR.MOE and CR1.CEN. Stopping TIM1 avoids further ADC triggers when control must never resume. SD4/PC3 remain untouched.
- PRIMASK is intentional: on Cortex-M4, Zephyr's `arch_irq_lock()` sets BASEPRI and does not mask the zero-latency DMA ISR. The fatal shutdown must also prevent rearming if invoked through SVC by `k_oops()`. Interrupts remain masked until reset.
- The application supplies the strong `void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)` override. Its first C statement calls the BSP shutdown; it then invokes `LOG_PANIC()`, logs `fatal: power stage off (reason %u)`, and calls the nonreturning `k_fatal_halt(reason)`. The handler declaration itself has no nonreturn attribute in Zephyr v4.4.2.
- `foc_ctrl_cmd.crash` is appended at +48; existing offsets are compile-time asserted unchanged. At its next ordinary loop iteration, the report thread clears `0x43524153` (`CRAS`) and invokes `k_oops()`. It does not poll during a blocking dump. The boot command-layout line includes `crash +48`. The ISR never reads the new field.
- The FOC ISR and its per-period helpers are unchanged. The only `foc_ctrl.c` edit extends its compile-time layout assertion. Fatal shutdown and stack telemetry stay in flash; CCM remains 9696 bytes.

## Zephyr fault-path review

Inspected the installed v4.4.2 source at `/home/gtc/zephyrproject/zephyr`:

1. `arch/arm/core/cortex_m/fault.c`: `z_arm_fault()` retrieves the exception frame, classifies/prints the fault through `fault_handle()`, handles any recoverable case, copies the frame, and calls `z_arm_fatal_error()`.
2. `arch/arm/core/fatal.c`: `z_arm_fatal_error()` prints the exception register frame (`CONFIG_EXCEPTION_DEBUG=y`), optionally unwinds the stack, then calls `z_fatal_error()`.
3. `kernel/fatal.c`: `z_fatal_error()` locks normal IRQs, logs the fatal reason and current thread, calls `coredump()`, and unconditionally calls `k_sys_fatal_error_handler()`. Coredump is disabled in this build. Thread abort is only considered if the application handler returns; ours never returns.
4. `k_oops()` expands to the ARM runtime-exception SVC. `arch/arm/core/cortex_m/svc.S` dispatches SVC 2 to `z_do_kernel_oops()` in `arch/arm/core/fatal.c`, which forwards the reason through `z_arm_fatal_error()` and the same unconditional application-handler call. The ELF shows the crash field cleared at offset 48 followed by reason 3 and `svc 2`.

Shutdown therefore follows Zephyr's fault classification and diagnostic logging. This change does not establish a bound on that preceding latency. There is an earlier optional hook usable without modifying Zephyr: enable `CONFIG_EXCEPTION_DUMP_HOOK` and register a callback with `arch_exception_set_dump_hook()` (`include/zephyr/arch/exception.h`). `EXCEPTION_DUMP` calls it before each diagnostic is logged. That output hook is not enabled/registered by this change; it depends on execution reaching a diagnostic, so it is not a universal exception-entry hook. Zephyr itself was not patched.

## Verification commands and results

All build outputs and raw verification logs are local under the ignored `build/` directory.

```sh
cmake -S tests/host/foc -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host
(cd build/host && ctest)
```

Result: **12/12 passed**, 0 failed, 31.30 s; compile flags include `-Werror -fsanitize=address,undefined`. No build warnings.

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
export ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr
west build -p always -b foc_motor -d build/arm . -- -DBUILD_VERSION=v4.4.2
west build -p always -b foc_motor -d build/arm-su . -- -DBUILD_VERSION=v4.4.2 -DEXTRA_CFLAGS=-fstack-usage
```

Both builds pass with **zero warnings** using SDK 1.0.1. The exact `Check artifacts and budgets` run block was extracted from `.github/workflows/ci.yml` and executed with `bash -e -o pipefail`:

```text
budgets: flash 86292/98304 B; SRAM data+bss 17596/22528 B; CCM 9696/10240 B
Check artifacts and budgets rc 0
```

These are the CI `size` totals. The linker region report separately shows FLASH 86296 B and RAM 17664 B because it accounts for alignment/region layout. Main stack remains 512; all existing CI config, artifact, allocation-symbol, and GPIO-hog gates pass. Generated `.config` confirms both stack-measurement options are `y`.

```sh
clang-format -i src/main.c src/pwm_adc.c src/pwm_adc.h src/foc_ctrl.c src/foc_ctrl.h
clang-format --dry-run --Werror src/main.c src/pwm_adc.c src/pwm_adc.h src/foc_ctrl.c src/foc_ctrl.h
git diff --check
```

Formatting and whitespace checks pass. Ran `clang-tidy -p build/verification/tidy` on `src/main.c`, `src/pwm_adc.c`, and `src/foc_ctrl.c`. The temporary compile database derives from `build/arm/compile_commands.json`, removes `-fno-printf-return-value`, `-fno-reorder-functions`, `-mfp16-format=ieee`, and `-fno-defer-pop`, and adds `--target=arm-none-eabi -Wno-unused-command-line-argument` without changing repository policy. All three return 0. Visible diagnostics: 53 signed-bitwise warnings from Zephyr logging macros plus one existing branch-clone warning from `K_THREAD_STACK_DEFINE` in `main.c`; 10 signed-bitwise macro warnings from existing GPIO assertions/IRQ macros in `pwm_adc.c`; none in `foc_ctrl.c`. No first-party analyzer or compiler errors. A final tidy recheck initially overlapped a pristine ARM rebuild and failed because generated `autoconf.h` was temporarily absent; rerunning all three checks after the builds completed returned 0 as above.

## Stack estimate

`build/arm-su/CMakeFiles/app.dir/src/main.c.su` reports:

| Frame | Bytes |
|---|---:|
| `report_loop.part.0` | 288 |
| `report` | 672 |
| `dump_capture` | 424 |
| `dump_ident` | 296 |
| `dump_map` | 256 |
| `report_stack_space` | 104 |
| `report_speed` | 72 |
| `cog_process` (includes inlined gain fit) | 240 |

The dump/report functions are sequential and must not all be added together. Disassembly shows `report_loop` popping its own 8-byte frame before tail-branching to `report_loop.part.0`; `z_thread_entry` retains 8 bytes.

A conservative logging chain from compiler frame maxima is:

```text
z_thread_entry 8
  + report_loop.part.0 288
  + report 672
  + z_impl_z_log_msg_static_create 136
  + cbprintf_package_convert 168
  + z_cbprintf_cpy 16
  + memcpy 8
= 1296 B
```

The prebuilt SDK libm has no generated `.su` files here. Disassembled prologues give 16 B for `_sinf`/`_cosf`, 40 B for `__rem_pio2f`, and 408 B for `__kernel_rem_pio2f` (36 B integer saves + 16 B FP saves + 356 B locals). Its `ldexpf`/`floorf` callees have no additional stack frame. Counting even that large-argument reduction path gives `8 + 288 + 256 + 240 + 16 + 40 + 408 = 1256 B` for map processing. The actual map angles are much smaller, but the larger path is included in the estimate.

Starting from the larger 1296 B estimate, allow 8 B aligned TLS, 104 B for an extended FP exception frame, and up to 4 B exception alignment: **1412 B**. Conservatively charging the full **128 B FP MPU guard again** gives **1540/2048 B**, with **508 B remaining**. The ELF actually reserves `report_stack` as 2176 B (`0x880`), including guard/alignment padding; the extra guard charge in this comparison is deliberately conservative.

This is a call-chain estimate, not a proven whole-program upper bound or a target measurement. The new unused-stack lines must still be collected on the board after the successful libm map path and all dump types.

## ELF shutdown and command-layout evidence

Using the SDK `arm-zephyr-eabi-objdump -d --disassemble=<symbol>` and `arm-zephyr-eabi-nm` on `build/arm/zephyr/zephyr.elf`:

```text
08005c44 T k_sys_fatal_error_handler
20002808 B foc_ctrl_cmd
20002d80 00000880 B report_stack
```

The strong handler calls `pwm_adc_fatal_off`, then `z_impl_log_panic`, emits the fatal message, and calls `k_fatal_halt`. `pwm_adc_fatal_off` contains `cpsid i`, a call through `__power_off_veneer` to the existing CCM helper, BDTR bit-15 clear, and CR1 bit-0 clear. `power_off` writes PB0 BRR, PA1/PA2 BRR, GPIOC MODER, and updates the local `powered` flag through inline DMB/LDREX/STREX instructions. Neither it nor the fatal BSP function calls kernel, device, logging, or control APIs.

The command block is 52 B (`0x34`). **`crash` is at `0x20002838`** (`0x20002808 + 48`). Addresses apply only to this ELF and must be obtained again after a rebuild that changes layout.

Still unverified: actual shutdown pin timing, post-fatal register state, UART panic output, the crash hook on target, and measured report-stack headroom. These require a separately authorized bench run. The fix does not replace the independent hardware shutdown still tracked by R-02.
