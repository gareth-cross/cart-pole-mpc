# The python wrapper module.
set(MODULE_NAME "pypendulum")
nanobind_add_module(${MODULE_NAME} wrapper.cc)
target_link_libraries(${MODULE_NAME} PRIVATE optimization)
target_compile_options(${MODULE_NAME} PRIVATE ${WARNING_FLAGS}
                                              ${COMPILER_FLAGS})
target_compile_definitions(${MODULE_NAME} PRIVATE PY_MODULE_NAME=${MODULE_NAME})
