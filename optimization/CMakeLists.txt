# Code for performing optimization based control.
set(MODULE_NAME "optimization")
add_library(
  ${MODULE_NAME} STATIC
  integration.hpp
  key.hpp
  optimization.cc
  optimization.hpp
  structs.hpp
  simulator.cc
  simulator.hpp
  single_pendulum_dynamics.hpp)
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

# Add WASM targets:
if(BUILD_WITH_EMSCRIPTEN)
  set(WASM_MODULE_NAME optimization-wasm)
  add_executable(${WASM_MODULE_NAME} wasm.cc)
  target_link_libraries(${WASM_MODULE_NAME} PRIVATE ${MODULE_NAME} eigen)
  target_compile_options(${WASM_MODULE_NAME} PRIVATE ${WARNING_FLAGS}
                                                     ${COMPILER_FLAGS})

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(EXTRA_WASM_LINK_FLAGS "-sASSERTIONS=2 -sSTACK_OVERFLOW_CHECK=2")
  elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(EXTRA_WASM_LINK_FLAGS "-sASSERTIONS=2")
  else()
    set(EXTRA_WASM_LINK_FLAGS "")
  endif()

  # ENVIRONMENT=web, compile for web target. SINGLE_FILE=1, produce a single
  # JavaScript file with base64 encoded WASM. MODULARIZE and EXPORT_NAME,
  # produce an ES6 module.
  set_target_properties(
    ${WASM_MODULE_NAME}
    PROPERTIES
      LINK_FLAGS
      "-sENVIRONMENT=web -sSINGLE_FILE=1 -sMODULARIZE \
       -sEXPORT_NAME=OptimizationWasm -lembind \
       --emit-tsd ${CMAKE_CURRENT_BINARY_DIR}/${WASM_MODULE_NAME}.d.ts \
       -sSTACK_SIZE=1048576 \
       -sINITIAL_MEMORY=64MB \
      ${EXTRA_WASM_LINK_FLAGS}")

  # Copy built JavaScirpt to the viz node project.
  add_custom_command(
    TARGET ${WASM_MODULE_NAME}
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/${WASM_MODULE_NAME}.js"
      "${CMAKE_SOURCE_DIR}/viz/src/${WASM_MODULE_NAME}.js")
  add_custom_command(
    TARGET ${WASM_MODULE_NAME}
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/${WASM_MODULE_NAME}.d.ts"
      "${CMAKE_SOURCE_DIR}/viz/src/${WASM_MODULE_NAME}.d.ts")
endif()

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

if(NOT BUILD_WITH_EMSCRIPTEN)
  add_cpp_test(integration_test SOURCE_FILES integration_test.cc)
  add_cpp_test(optimization_test SOURCE_FILES optimization_test.cc)
endif()
