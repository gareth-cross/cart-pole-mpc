cmake_minimum_required(VERSION 3.20)
project(pendulum CXX)
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
  set(COMPILER_FLAGS /bigobj /EHs /Zc:__cplusplus)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror)
  set(COMPILER_FLAGS "")
endif()

add_subdirectory(optimization)
add_subdirectory(wrapper)
