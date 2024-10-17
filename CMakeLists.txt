cmake_minimum_required(VERSION 3.20)
project(pendulum CXX)

# If true, we adjust targets+flags to compile for WASM.
option(BUILD_WITH_EMSCRIPTEN "Configure for compilation under emscripten." OFF)
option(EMSCRIPTEN_ENABLE_LTO "Use LTO when building in Release mode." OFF)
option(EMSCRIPTEN_ENABLE_SANITIZE "Use address sanitization." OFF)

# emcc will not accept various arguments, like -mtune=haswell, etc. print these
# so we can see what the user environment has specified.
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")

if(BUILD_WITH_EMSCRIPTEN)
  add_compile_options(-fwasm-exceptions -Wno-deprecated-declarations)
  add_link_options(-fwasm-exceptions)
  # Enable debug symbols and address sanitization in Debug builds.
  if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL
                                          "RelWithDebInfo")
    add_compile_options(-g3)
    add_link_options(-g3)
    if(EMSCRIPTEN_ENABLE_SANITIZE)
      add_compile_options(-fsanitize=address)
      add_link_options(-fsanitize=address)
    endif()
  endif()
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_link_options(--profiling-funcs)
    # Enable LTO in Release builds.
    if(EMSCRIPTEN_ENABLE_LTO)
      add_compile_options(-flto)
      add_link_options(-flto)
    endif()
  endif()
endif()

# We will only build tests + python wrapper if not using emscripten.
if(NOT BUILD_WITH_EMSCRIPTEN)
  enable_testing()

  # Required to build the shared-object wrapper.
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
  set(COMPILER_FLAGS /bigobj /EHs /Zc:__cplusplus -D_USE_MATH_DEFINES)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror)
  set(COMPILER_FLAGS "")
endif()

add_subdirectory(optimization)

if(NOT BUILD_WITH_EMSCRIPTEN)
  add_subdirectory(wrapper)
endif()
