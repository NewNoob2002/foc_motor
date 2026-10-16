cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(foc_motor LANGUAGES C)

target_sources(app PRIVATE src/main.c src/encoder.c src/analog.c src/pwm_adc.c src/foc_bench.c src/foc_ctrl.c src/power_guard.c src/motor_ident.c src/sincos.c src/foc_speed.c src/cog_map.c src/cog_proc.c)

# lib/foc: the same pure-C sources the host tests build (tests/host/foc).
# tests/foc_vectors: the fixed inputs shared with tests/host/foc/vec_compare.
set(FOC_SOURCES
  lib/foc/src/foc_transform.c
  lib/foc/src/foc_pi.c
  lib/foc/src/foc_svpwm.c
  lib/foc/src/foc_current.c
  tests/foc_vectors/foc_vectors.c)
target_sources(app PRIVATE ${FOC_SOURCES})
target_include_directories(app PRIVATE src lib/foc/include tests/foc_vectors)

# TEXT only: adc_buf and all other DMA/state buffers must stay in SRAM1/2.
# Zephyr generates the flash load image and copies it in arch_data_copy(),
# before z_cstart(), the boot bench, and pwm_adc_init(). NOKEEP preserves GC.
zephyr_code_relocate(FILES
  src/power_guard.c src/foc_speed.c
  lib/foc/src/foc_transform.c lib/foc/src/foc_pi.c
  lib/foc/src/foc_svpwm.c lib/foc/src/foc_current.c
  LOCATION CCM_TEXT NOKEEP)
# Thread interfaces and boot initialization stay in flash; every step target is CCM.
zephyr_code_relocate(FILES src/foc_ctrl.c
  FILTER "^(?!.*[.](ctrl_once|foc_ctrl_(init|snapshot|capture_get|capture_release|ident_get|ident_release|map_get|map_release))$).*"
  LOCATION CCM_TEXT NOKEEP)
zephyr_code_relocate(FILES src/cog_map.c
  FILTER "^(?!.*[.]cog_map_(start|release)$).*" LOCATION CCM_TEXT NOKEEP)
# Alignment validation, resistance fit, reset and result copy run once per identification in flash.
zephyr_code_relocate(FILES src/motor_ident.c
  FILTER "^(?!.*[.](ident_flash|align_delta|motor_ident_start|motor_ident_result_copy)$).*" LOCATION CCM_TEXT NOKEEP)
# Whole pwm_adc.c exceeds CCM capacity. Select attributed ISR helpers and
# the compiler's out-of-line atomic helpers; keep setup/thread APIs in flash.
zephyr_code_relocate(FILES src/pwm_adc.c
  FILTER ".*[.](ccm_hot|atomic_.*)$" LOCATION CCM_TEXT NOKEEP)
# encoder_count() is the ISR's only out-of-line encoder access.
zephyr_code_relocate(FILES src/encoder.c FILTER ".*[.]encoder_count$" LOCATION CCM_TEXT NOKEEP)
# double math is software-emulated on the single-precision M4F FPU.
set_source_files_properties(${FOC_SOURCES} src/foc_ctrl.c src/power_guard.c src/motor_ident.c src/foc_speed.c src/cog_map.c src/cog_proc.c PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)
