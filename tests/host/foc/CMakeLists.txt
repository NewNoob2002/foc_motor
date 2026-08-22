cmake_minimum_required(VERSION 3.20)
project(foc_host_smoke LANGUAGES C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  message(FATAL_ERROR "Host tests require GCC or Clang")
endif()

enable_testing()
add_executable(foc_host_smoke smoke.c)
target_compile_features(foc_host_smoke PRIVATE c_std_11)
target_compile_options(foc_host_smoke PRIVATE
  -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer)
target_link_options(foc_host_smoke PRIVATE -fsanitize=address,undefined)
add_test(NAME foc_host_smoke COMMAND foc_host_smoke)
set_tests_properties(foc_host_smoke PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
