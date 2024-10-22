function(add_libfmt)
  if(NOT TARGET fmt::fmt)
    set(FMT_TEST
        OFF
        CACHE BOOL "" FORCE)
    add_subdirectory(fmt EXCLUDE_FROM_ALL)
  endif()
endfunction()
add_libfmt()

function(add_eigen)
  if(NOT TARGET eigen)
    set(EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
    add_library(eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                           "${EIGEN_INCLUDE_DIRS}")
  endif()
endfunction()
add_eigen()

function(add_gtest)
  if(NOT TARGET gtest)
    set(BUILD_GMOCK
        OFF
        CACHE BOOL "" FORCE)
    set(INSTALL_GTEST
        OFF
        CACHE BOOL "" FORCE)
    add_subdirectory(gtest)
  endif()
endfunction()

if(NOT BUILD_WITH_EMSCRIPTEN)
  add_gtest()
endif()

function(add_nanobind)
  add_subdirectory(nanobind)
endfunction()

if(NOT BUILD_WITH_EMSCRIPTEN)
  add_nanobind()
endif()

# Create an interface library for the wrenfold runtime headers.
function(add_wrenfold_headers)
  add_library(wrenfold-runtime INTERFACE)
  target_include_directories(
    wrenfold-runtime
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/wrenfold-runtime>
              $<INSTALL_INTERFACE:include>)
  add_library(wrenfold::wrenfold-runtime ALIAS wrenfold-runtime)
endfunction()
add_wrenfold_headers()

# Add mini-opt library.
function(add_mini_opt)
  if(BUILD_WITH_EMSCRIPTEN)
    set(MINI_OPT_BUILD_TESTS
        OFF
        CACHE BOOL "" FORCE)
    set(MINI_OPT_SERIALIZATION
        ON
        CACHE BOOL "" FORCE)
  endif()
  add_subdirectory(mini_opt)
endfunction()
add_mini_opt()
