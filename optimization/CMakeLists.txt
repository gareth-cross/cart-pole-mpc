# Code for performing optimization based control.
set(MODULE_NAME "optimization")
add_library(${MODULE_NAME} STATIC parameters.hpp integration.hpp
                                  dynamics_single.hpp dynamics_double.hpp)
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

# Add unit tests
add_executable(integration_test integration_test.cc)
target_link_libraries(integration_test PRIVATE ${MODULE_NAME} gtest_main)
target_compile_options(integration_test PRIVATE ${WARNING_FLAGS}
                                                ${COMPILER_FLAGS})
add_test(integration_test integration_test)
