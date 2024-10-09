cmake_minimum_required(VERSION 3.20)
project(pendulum CXX)

# If true, we adjust targets+flags to compile for WASM.
option(BUILD_WITH_EMSCRIPTEN "Configure for compilation under emscripten." OFF)

# emcc will not accept various arguments, like -mtune=haswell, etc. print these
# so we can see what the user environment has specified.
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")

if(BUILD_WITH_EMSCRIPTEN)
  add_compile_options(-fwasm-exceptions)
  add_link_options(-fwasm-exceptions)
endif()

# We will only build tests + python wrapper if not using emscripten.
if(NOT BUILD_WITH_EMSCRIPTEN)
  enable_testing()

  # Find python (required for nanobind)
  find_package(
    Python 3.9
    COMPONENTS Interpreter Development.Module
    REQUIRED)
  message(STATUS "Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
  message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")
endif()

add_subdirectory(dependencies)

if(MSVC)
  set(WARNING_FLAGS /W4 /WX)
  set(COMPILER_FLAGS /bigobj /EHs /Zc:__cplusplus)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror)
  set(COMPILER_FLAGS "")
endif()

add_subdirectory(optimization)

if(NOT BUILD_WITH_EMSCRIPTEN)
  add_subdirectory(wrapper)
endif()
