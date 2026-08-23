add_library(image_reader MODULE
  image_reader.cpp
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(image_reader PRIVATE ${MOVUTL_INCLUDES})
target_link_libraries(image_reader PRIVATE ${CMAKE_DL_LIBS})

set_target_properties(image_reader PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
