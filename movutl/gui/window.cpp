#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// --
#include <movutl/core/logger.hpp>
#include <movutl/core/vector.hpp>
#include <movutl/gui/gui.hpp>
#include <stdio.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>

// ImGui - standalone example application for Glfw + OpenGL 3, using programmable pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
static void error_callback(int error, const char* description) {
  fprintf(stderr, "Error %d: %s\n", error, description);
}

namespace mu {

void GUIManager::init() {
  glfwSetErrorCallback(error_callback);

  if(!glfwInit()) {
    LOG_F(ERROR, "Could not initialize GLFW");
    return;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
#if __APPLE__
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#else
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
#endif
  // 古い機能を削除したプロファイルを使用するか
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  // OpenGLのプロファイルを指定する
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  glfw_window = glfwCreateWindow(1280, 720, "ImGui OpenGL3 example", NULL, NULL);
  glfwMakeContextCurrent(glfw_window);

  glewInit();

  // Setup ImGui binding
  ImGui_ImplOpenGL3_Init();
  ImGui_ImplGlfw_InitForOpenGL(glfw_window, true);

  // Load FontsR
  // (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
  // ImGuiIO& io = ImGui::GetIO();
  // io.Fonts->AddFontDefault();
  // io.Fonts->AddFontFromFileTTF("../../extra_fonts/Cousine-Regular.ttf", 15.0f);
  // io.Fonts->AddFontFromFileTTF("../../extra_fonts/DroidSans.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyClean.ttf", 13.0f);
  // io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyTiny.ttf", 10.0f);
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());

  detail::init_gui_panels();
}

ImVec4 clear_color = ImColor(20, 20, 20);

namespace detail {

void gui_new_frame() {
  auto window = GUIManager::Get()->glfw_window;

  bool should_close = glfwWindowShouldClose(window);
  GUIManager::Get()->should_close = should_close;

  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void gui_render_to_screen() {
  auto window = GUIManager::Get()->glfw_window;
  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui::Render();
  _Vec<int, 2> window_size;
  glfwGetFramebufferSize(window, &window_size[0], &window_size[1]);
  ImGui::SetNextWindowBgAlpha(0.35f);
  glViewport(0, 0, window_size[0], window_size[1]);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(window);
}

} // namespace detail

void update() {
  detail::gui_new_frame();

  detail::update_gui_panels();

  detail::gui_render_to_screen();
}

void GUIManager::terminate() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  glfwTerminate();
}

void terminate() {
  GUIManager::Get()->terminate();
}

bool should_terminate() {
  return GUIManager::Get()->should_close;
}

GUIManager* GUIManager::singleton_ = nullptr;

} // namespace mu
