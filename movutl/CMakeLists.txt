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

find_package(OpenMP QUIET)

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
if(OpenMP_CXX_FOUND)
  set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} OpenMP::OpenMP_CXX)
endif()

# --------------------------------------------------------------
# ------------     Tracy Profiler -------------------------------
# --------------------------------------------------------------
option(MOVUTL_TRACY "Tracyプロファイラを有効にしてビルドする" ON)
if(MOVUTL_TRACY)
  include(FetchContent)
  FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.13.1
    GIT_SHALLOW TRUE
  )
  set(TRACY_ENABLE ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(tracy)
  set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} TracyClient)
  add_compile_definitions(MOVUTL_TRACY_ENABLED)
endif()

# --------------------------------------------------------------
# ------------     miniaudio (再生I/O) --------------------------
# --------------------------------------------------------------
# miniaudio/Maximilianはヘッダ/ソースをmucoreへ直接コンパイルするだけで、両者ともCMakeターゲットとしては消費しない。
# miniaudioのCMakeLists.txt(examples/tests込み)やMaximilian(CMakeLists.txtを持たない)をadd_subdirectoryさせないよう、
# FetchContent_MakeAvailableではなくFetchContent_Populateでソース取得のみ行う
include(FetchContent)
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD) # FetchContent_Populate(単体呼び出し、add_subdirectoryしない用途)を許可する
endif()
FetchContent_Declare(
  miniaudio
  GIT_REPOSITORY https://github.com/mackron/miniaudio.git
  GIT_TAG 9634bedb5b5a2ca38c1ee7108a9358a4e233f14d
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(miniaudio)
if(NOT miniaudio_POPULATED)
  FetchContent_Populate(miniaudio)
endif()
set(MOVUTL_INCLUDES ${MOVUTL_INCLUDES} ${miniaudio_SOURCE_DIR})

# --------------------------------------------------------------
# ------------     Maximilian (音声エフェクトDSP) -----------------
# --------------------------------------------------------------
FetchContent_Declare(
  maximilian
  GIT_REPOSITORY https://github.com/micknoise/Maximilian.git
  GIT_TAG f937afb71aee841607f87a7f8bbd9783581fbd30
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(maximilian)
if(NOT maximilian_POPULATED)
  FetchContent_Populate(maximilian)
endif()
set(MOVUTL_INCLUDES ${MOVUTL_INCLUDES} ${maximilian_SOURCE_DIR}/src)
set(MOVUTL_MAXIMILIAN_SRC ${maximilian_SOURCE_DIR}/src/maximilian.cpp ${maximilian_SOURCE_DIR}/src/libs/maxiReverb.cpp)

# --------------------------------------------------------------
# ------------     uapmd (remidy VST3ホスティング) ----------------
# --------------------------------------------------------------
# atsushieno/uapmd の source/CMakeLists.txt は remidy-gui/uapmd-engine等の
# 未使用モジュールまで一括add_subdirectoryする作りのため、source/CMakeLists.txtは
# 経由せず remidy と uapmd-plugin-hosting だけを直接add_subdirectoryする。
# 両者が親スコープに要求する変数 (choc/concurrentqueue/midicci) を、
# ext/uapmd/source/CMakeLists.txt に書かれているCPMAddPackage呼び出しと同じ内容で用意する。
if(APPLE)
  enable_language(OBJCXX)
endif()
include(${MOVUTL_ROOT_DIR}/ext/uapmd/source/cmake/CPM.cmake)

CPMAddPackage(
  NAME            choc
  GIT_TAG         1857a38
  GIT_REPOSITORY  https://github.com/Tracktion/choc
  DOWNLOAD_ONLY YES
)
CPMAddPackage(
  NAME           concurrentqueue
  GIT_TAG        2f09da73d22a47dc8a89cdd4fc4c3bfae07f4284
  GIT_REPOSITORY https://github.com/cameron314/concurrentqueue.git
  DOWNLOAD_ONLY YES
)
# midicci は静的ライブラリとしてビルドされる (remidyがmidicci_staticにリンクする)
set(_MOVUTL_OLD_BUILD_SHARED_LIBS "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
CPMAddPackage(midicci
  NAME            midicci
  GIT_TAG         0.4.0
  GIT_REPOSITORY  https://github.com/atsushieno/midicci
  OPTIONS         "MIDICCI_SKIP_TOOLS ON" "BUILD_TESTS OFF" "BUILD_EXAMPLES OFF"
  EXCLUDE_FROM_ALL
)
if(DEFINED _MOVUTL_OLD_BUILD_SHARED_LIBS AND NOT "${_MOVUTL_OLD_BUILD_SHARED_LIBS}" STREQUAL "")
  set(BUILD_SHARED_LIBS "${_MOVUTL_OLD_BUILD_SHARED_LIBS}")
else()
  unset(BUILD_SHARED_LIBS)
endif()

# uapmd-plugin-hostingのRemoteScanSessionManager.cppはデスクトップ(Android/Emscripten/iOS以外)では
# UAPMD_PLUGIN_HOSTING_REMOTE_SCAN_SUPPORTEDが常に1になり、無条件でcpplocate/cpplocate.hをincludeする
# (uapmd側のUAPMD_ENABLE_CPPLOCATEオプションでは制御できない)。source/CMakeLists.txtと同じCPMAddPackageで用意する。
set(UAPMD_ENABLE_CPPLOCATE ON)
CPMAddPackage(cpplocate
  NAME           cpplocate
  GIT_TAG        b796990d8df747758dbd6217b6315b7bdbb0c27e
  GIT_REPOSITORY https://github.com/cginternals/cpplocate
  EXCLUDE_FROM_ALL
)
set(cpplocate_LIBRARIES cpplocate::cpplocate)

# remidy/uapmd-plugin-hostingはC++23を要求する(source/CMakeLists.txt準拠)。
# movutl本体のC++標準を変えないよう、add_subdirectory直前だけ変更して元に戻す。
set(_MOVUTL_OLD_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD 23)
add_subdirectory(${MOVUTL_ROOT_DIR}/ext/uapmd/source/remidy ${CMAKE_CURRENT_BINARY_DIR}/uapmd_remidy)
add_subdirectory(${MOVUTL_ROOT_DIR}/ext/uapmd/source/uapmd-plugin-hosting ${CMAKE_CURRENT_BINARY_DIR}/uapmd_plugin_hosting)
set(CMAKE_CXX_STANDARD "${_MOVUTL_OLD_CXX_STANDARD}")

set(MOVUTL_INCLUDES
  ${MOVUTL_INCLUDES}
  ${MOVUTL_ROOT_DIR}/ext/uapmd/source/uapmd-plugin-hosting/include
  ${MOVUTL_ROOT_DIR}/ext/uapmd/source/remidy/include
)
set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} uapmd::uapmd-plugin-hosting remidy::remidy)

file(GLOB DATA_FILES ${CMAKE_CURRENT_SOURCE_DIR}/data/*)
file(COPY ${DATA_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB MOVUTL_SOURCE_1 ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) 
file(GLOB MOVUTL_SOURCE_2 ${CMAKE_CURRENT_SOURCE_DIR}/**/*.cpp) 
file(GLOB MOVUTL_SOURCE_3 ${CMAKE_CURRENT_SOURCE_DIR}/*/*.cpp) 
file(GLOB MOVUTL_SOURCE_4 ${CMAKE_CURRENT_SOURCE_DIR}/*/*/*.cpp) 
file(GLOB MOVUTL_SOURCE_5 ${CMAKE_CURRENT_SOURCE_DIR}/*/*/*/*.cpp) 

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
  ${MOVUTL_ROOT_DIR}/ext/cutil
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

set(MOVUTL_SRC ${MOVUTL_SOURCE_1} ${MOVUTL_SOURCE_2} ${MOVUTL_SOURCE_3} ${MOVUTL_SOURCE_4} ${MOVUTL_EXT_SRC} ${MOVUTL_SOURCE_5} ${MOVUTL_MAXIMILIAN_SRC})
if(APPLE)
  file(GLOB MOVUTL_SRC_MAC ${CMAKE_CURRENT_SOURCE_DIR}/core/*.mm)
  set(MOVUTL_SRC ${MOVUTL_SRC} ${MOVUTL_SRC_MAC})

  set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} "-framework Cocoa")
endif()
add_library(mucore ${MOVUTL_SRC})
# export_png/export_video等の.so/.msoプラグインがmucoreを静的リンクするため、Linux/gccでも共有ライブラリへ組み込めるようPICを強制する
set_target_properties(mucore PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mucore PUBLIC ${MOVUTL_INCLUDES})
target_link_libraries(mucore ${MOVUTL_LIBRARIES})
set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} movutl)

set(MOVUTL_INCLUDES ${MOVUTL_INCLUDES} PARENT_SCOPE)
set(MOVUTL_LIBRARIES ${MOVUTL_LIBRARIES} PARENT_SCOPE)

