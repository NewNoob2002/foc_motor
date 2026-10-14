cmake_minimum_required(VERSION 3.20)
project(foc_host_tests LANGUAGES C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  message(FATAL_ERROR "Host tests require GCC or Clang")
endif()

# Off by default so CI stays green while the PI/SVPWM exercises are open;
# turn on to check your own implementation (docs/study/foc_study_guide.md).
option(FOC_EXERCISES "Register PI, SVPWM and closed-loop simulation tests" OFF)

set(FOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/foc)
set(FOC_FLAGS -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer)

enable_testing()

# Same sources the Zephyr firmware will compile; no RTOS or board headers.
add_library(foc STATIC
  ${FOC_DIR}/src/foc_transform.c
  ${FOC_DIR}/src/foc_pi.c
  ${FOC_DIR}/src/foc_svpwm.c)
target_include_directories(foc PUBLIC ${FOC_DIR}/include)
target_compile_features(foc PUBLIC c_std_11)
# double math is software-emulated on the single-precision M4F FPU.
target_compile_options(foc PRIVATE ${FOC_FLAGS} -Wdouble-promotion)

function(foc_host_executable name source)
  add_executable(${name} ${source})
  target_compile_options(${name} PRIVATE ${FOC_FLAGS})
  target_link_options(${name} PRIVATE -fsanitize=address,undefined)
  target_link_libraries(${name} PRIVATE foc m)
endfunction()

function(foc_host_test name)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
endfunction()

foc_host_executable(foc_host_smoke smoke.c)
foc_host_executable(test_transform test_transform.c)
foc_host_executable(test_pi test_pi.c)
foc_host_executable(test_svpwm test_svpwm.c)
foc_host_executable(sim_current_loop sim/sim_current_loop.c)
foc_host_executable(sim_svpwm sim/sim_svpwm.c)
foc_host_executable(sim_speed_loop sim/sim_speed_loop.c)

foc_host_test(foc_host_smoke)
foc_host_test(test_transform)
if(FOC_EXERCISES)
  foc_host_test(test_pi)
  foc_host_test(test_svpwm)
  foc_host_test(sim_current_loop)
endif()
