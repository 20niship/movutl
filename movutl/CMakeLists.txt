cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE LEGACY)

find_package(Threads REQUIRED QUIET)
find_package(glfw3 REQUIRED QUIET)
find_package(Freetype REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Vulkan QUIET)
find_package(OpenGL REQUIRED QUIET)
find_package(GLEW REQUIRED QUIET)
find_package(glm REQUIRED QUIET)
find_package(assimp REQUIRED QUIET)

# https://github.com/PointCloudLibrary/pcl/issues/3680
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
   set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PkgConfig REQUIRED)

if(APPLE)
  pkg_check_modules(JSONCPP jsoncpp)
  find_package(JsonCpp  QUIET REQUIRED)
  if(NOT JSONCPP_FOUND)
    set(JsonCpp::JsonCpp jsoncpp)
  endif()

  # Appleでは何もしないとOpenMPが見つからないので、Homebrewのパスを指定する
  # https://zenn.dev/arawii/articles/47a24cf2d0a3b2
  cmake_policy(SET CMP0074 NEW) # OpenMP_ROOTの環境変数を使用してパッケージを探す
  set(OpenMP_ROOT "/opt/homebrew/opt/libomp")
endif()

pkg_check_modules(LuaJIT REQUIRED IMPORTED_TARGET luajit)


# --------------------------------------------------------------
# ------------     Renderer (OpenGL / Vulkan) ------------------
# --------------------------------------------------------------
set(RENDERER "WITH_OPENGL")
add_compile_definitions(${RENDERER})

set(MOVUTL_LIBRARIES 
  glfw
  ${GLFW_LIBRARY_DIRS}
  ${CMAKE_DL_LIBS}
  ${FREETYPE_LIBRARIES}
  OpenGL::GL 
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  # ${assimp_LIBRARIES}
  assimp::assimp
  PkgConfig::LuaJIT
)

set(MOVUTL_INCLUDES 
  ${MOVUTL_INCLUDES}
  ${LUAJIT_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${assimp_INCLUDE_DIR}
  ${glm_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  # ${spdlog_INCLUDE_DIR}
  ${MOVUTL_ROOT_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} GLEW::GLEW)

file(GLOB DATA_FILES ${CMAKE_CURRENT_SOURCE_DIR}/data/*)
file(COPY ${DATA_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB MOVUTL_SOURCE_1 ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) 
file(GLOB MOVUTL_SOURCE_2 ${CMAKE_CURRENT_SOURCE_DIR}/**/*.cpp) 
file(GLOB MOVUTL_SOURCE_3 ${CMAKE_CURRENT_SOURCE_DIR}/*/*/*.cpp) 
file(GLOB MOVUTL_SOURCE_4 ${CMAKE_CURRENT_SOURCE_DIR}/*/*/*/*.cpp) 

# --------------------------------------------------------------
# ------------     Third Party Libraries -----------------------
# --------------------------------------------------------------
set(MOVUTL_INCLUDES
  ${MOVUTL_INCLUDES}
  ${MOVUTL_ROOT_DIR}/ext/stb
  ${MOVUTL_ROOT_DIR}/ext/loguru
  ${MOVUTL_ROOT_DIR}/ext/IconFontCppHeaders
  ${MOVUTL_ROOT_DIR}/ext/imgui
  ${MOVUTL_ROOT_DIR}/ext/imgui/backends
  ${MOVUTL_ROOT_DIR}/ext/filewatch
  ${MOVUTL_ROOT_DIR}/ext/json/include
  ${MOVUTL_ROOT_DIR}/ext/lua-intf/
  ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf
)
file(GLOB MOVUTL_EXT_SRC 
  ${MOVUTL_ROOT_DIR}/ext/loguru/loguru.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/imgui.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/imgui_draw.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/imgui_widgets.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/imgui_tables.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/imgui_demo.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/backends/imgui_impl_glfw.cpp
  ${MOVUTL_ROOT_DIR}/ext/imgui/backends/imgui_impl_opengl3.cpp
  ${MOVUTL_ROOT_DIR}/ext/filewatch/FileWatcher.cpp

  # -- luaintf
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/LuaRef.cpp
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/LuaState.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/LuaFunction.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/LuaRefLink.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/CppBindModule.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/CppBindFunction.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/CppBindClass.cpp 
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/LuaCompat.cpp
  # ${MOVUTL_ROOT_DIR}/ext/lua-intf/LuaIntf/src/QtLuaIntf.cpp
)

include_directories(${MOVUTL_INCLUDES})

set(MOVUTL_SRC ${MOVUTL_SOURCE_1} ${MOVUTL_SOURCE_2} ${MOVUTL_SOURCE_3} ${MOVUTL_SOURCE_4} ${MOVUTL_EXT_SRC})
add_library(mucore ${MOVUTL_SRC})
target_include_directories(mucore PUBLIC ${MOVUTL_INCLUDES})
target_link_libraries(mucore ${MOVUTL_LIBRARIES})
set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} movutl)

set(MOVUTL_INCLUDES ${MOVUTL_INCLUDES} PARENT_SCOPE)
set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} PARENT_SCOPE)

