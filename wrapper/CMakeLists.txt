cmake_minimum_required(VERSION 3.20)
project(pypendulum CXX)
enable_testing()

# Find python (required for nanobind)
find_package(
  Python 3.9
  COMPONENTS Interpreter Development.Module
  REQUIRED)
message(STATUS "Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")

add_subdirectory(dependencies)

if(MSVC)
  set(WARNING_FLAGS /W4 /WX)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror -Wno-sign-compare)
endif()

# The main wrapper module.
set(MODULE_NAME "pypendulum")
nanobind_add_module(
  ${MODULE_NAME}
  source/wrapper.cc
  source/parameters.hpp
  source/integration.hpp
  source/assertions.hpp
  source/dynamics_single.hpp
  source/dynamics_double.hpp)
target_compile_features(${MODULE_NAME} PRIVATE cxx_std_17)
target_link_libraries(${MODULE_NAME} PRIVATE fmt::fmt-header-only eigen
                                             wrenfold::wrenfold-runtime)
target_compile_options(${MODULE_NAME} PRIVATE ${WARNING_FLAGS})
target_compile_definitions(${MODULE_NAME} PRIVATE PY_MODULE_NAME=${MODULE_NAME})

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug" AND NOT MSVC)
  target_compile_options(${MODULE_NAME} PRIVATE -O3)
endif()

# Add unit tests
add_executable(
  integration_test
  source/integration_test.cc source/integration.hpp source/dynamics_single.hpp
  source/dynamics_double.hpp source/assertions.hpp)
target_compile_features(integration_test PRIVATE cxx_std_17)
target_link_libraries(
  integration_test PRIVATE fmt::fmt-header-only eigen gtest_main
                           wrenfold::wrenfold-runtime)
target_compile_options(integration_test PRIVATE ${WARNING_FLAGS})
add_test(integration_test integration_test)
