add_library(brush_sample MODULE
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(brush_sample PRIVATE ${MOVUTL_INCLUDES})
target_link_libraries(brush_sample PRIVATE mucore ${CMAKE_DL_LIBS})

set_target_properties(brush_sample PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
