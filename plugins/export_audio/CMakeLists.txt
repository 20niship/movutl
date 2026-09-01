find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG_EA QUIET IMPORTED_TARGET libavformat libavcodec libavutil libswresample)
if(NOT FFMPEG_EA_FOUND)
  message(WARNING "FFmpeg not found: export_audio plugin (.mso) is skipped")
  return()
endif()

add_library(export_audio MODULE
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(export_audio PRIVATE ${MOVUTL_INCLUDES})
target_compile_definitions(export_audio PRIVATE MOVUTL_HAS_FFMPEG)
target_link_libraries(export_audio PRIVATE mucore PkgConfig::FFMPEG_EA ${CMAKE_DL_LIBS})

set_target_properties(export_audio PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
