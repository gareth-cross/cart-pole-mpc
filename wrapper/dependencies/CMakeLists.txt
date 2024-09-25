function(add_libfmt)
  set(FMT_TEST
      OFF
      CACHE BOOL "" FORCE) # Turn off libfmt tests
  add_subdirectory(fmt EXCLUDE_FROM_ALL) # Don't need to install.
endfunction()
add_libfmt()

function(add_nanobind)
  add_subdirectory(nanobind)
endfunction()
add_nanobind()

function(add_eigen)
  set(EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
  if(NOT TARGET eigen)
    add_library(eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                           "${EIGEN_INCLUDE_DIRS}")
  endif()
endfunction()
add_eigen()

function(add_gtest)
  set(BUILD_GMOCK
      OFF
      CACHE BOOL "" FORCE)
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE)
  add_subdirectory(gtest)
endfunction()
add_gtest()

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
