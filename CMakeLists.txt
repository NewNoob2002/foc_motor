cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(foc_motor LANGUAGES C)

target_sources(app PRIVATE src/main.c src/encoder.c src/analog.c src/pwm_adc.c src/foc_bench.c src/foc_ctrl.c src/power_guard.c src/motor_ident.c src/sincos.c)

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
# double math is software-emulated on the single-precision M4F FPU.
set_source_files_properties(${FOC_SOURCES} src/foc_ctrl.c src/power_guard.c src/motor_ident.c PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)
