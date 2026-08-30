find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG_EV QUIET IMPORTED_TARGET libavformat libavcodec libavutil libswscale)
if(NOT FFMPEG_EV_FOUND)
  message(WARNING "FFmpeg not found: export_video plugin (.mso) is skipped")
  return()
endif()

add_library(export_video MODULE
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(export_video PRIVATE ${MOVUTL_INCLUDES})
target_compile_definitions(export_video PRIVATE MOVUTL_HAS_FFMPEG)
target_link_libraries(export_video PRIVATE mucore PkgConfig::FFMPEG_EV ${CMAKE_DL_LIBS})

set_target_properties(export_video PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
