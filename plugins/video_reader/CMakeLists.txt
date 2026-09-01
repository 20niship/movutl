find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG_VR QUIET IMPORTED_TARGET libavformat libavcodec libavutil libswscale libswresample)
if(NOT FFMPEG_VR_FOUND)
  message(WARNING "FFmpeg not found: video_reader plugin (.mso) is skipped")
  return()
endif()

add_library(video_reader MODULE
  video_reader.cpp
  plugin_main.cpp
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
)
target_include_directories(video_reader PRIVATE ${MOVUTL_INCLUDES})
target_compile_definitions(video_reader PRIVATE MOVUTL_HAS_FFMPEG)
target_link_libraries(video_reader PRIVATE PkgConfig::FFMPEG_VR ${CMAKE_DL_LIBS})

set_target_properties(video_reader PROPERTIES
  PREFIX ""
  SUFFIX ".mso"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins
)
