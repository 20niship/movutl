add_library(export_png MODULE
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(export_png PRIVATE ${MOVUTL_INCLUDES})
target_link_libraries(export_png PRIVATE mucore ${CMAKE_DL_LIBS})

set_target_properties(export_png PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
