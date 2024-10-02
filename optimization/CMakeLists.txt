# Code for performing optimization based control.
set(MODULE_NAME "optimization")
add_library(
  ${MODULE_NAME} STATIC
  parameters.hpp integration.hpp single_pendulum_dynamics.hpp
  double_pendulum_dynamics.hpp single_pendulum_energy.hpp)
target_compile_features(${MODULE_NAME} PUBLIC cxx_std_17)
target_link_libraries(${MODULE_NAME} PUBLIC fmt::fmt-header-only eigen
                                            wrenfold::wrenfold-runtime mini_opt)
target_compile_options(${MODULE_NAME} PRIVATE ${WARNING_FLAGS}
                                              ${COMPILER_FLAGS})

target_include_directories(
  ${MODULE_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                        "$<INSTALL_INTERFACE:include>")

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug" AND NOT MSVC)
  target_compile_options(${MODULE_NAME} PRIVATE -O3)
endif()

if(MSVC)
  target_compile_definitions(${MODULE_NAME} PRIVATE -D_USE_MATH_DEFINES)
endif()

# Enable wrenfold-span support
target_compile_definitions(${MODULE_NAME} PUBLIC -DWF_SPAN_EIGEN_SUPPORT)

# Function to add unit tests to this module.
function(add_cpp_test NAME)
  set(options "")
  set(oneValueArgs "")
  set(multiValueArgs SOURCE_FILES)
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}"
                        ${ARGN})
  # add executable for the test:
  add_executable(${NAME} ${ARGS_SOURCE_FILES})
  target_link_libraries(${NAME} ${MODULE_NAME} gtest_main)
  target_compile_options(${NAME} PRIVATE ${WARNING_FLAGS} ${COMPILATION_FLAGS})
  add_test(${NAME} ${NAME})
endfunction()

add_cpp_test(integration_test SOURCE_FILES integration_test.cc)
add_cpp_test(optimization_test SOURCE_FILES optimization_test.cc)
