#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// --
#include <movutl/app/app.hpp>
#include <movutl/core/filesystem.hpp>
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

  LOG_F(2, "ImGui::CheckVersion();");
  IMGUI_CHECKVERSION();
  LOG_F(2, "ImGui::CreateContext with docking enabled");
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // start docking
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // | ImGuiConfigFlags_ViewportsEnable;
  io.ConfigFlags |= ImGuiWindowFlags_NoBackground;
  // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  io.ConfigDockingWithShift = false;
  io.ConfigDockingNoSplit = false;

  glfw_window = glfwCreateWindow(1280, 720, "ImGui OpenGL3 example", NULL, NULL);
  glfwMakeContextCurrent(glfw_window);

  glewInit();

  // Setup ImGui binding
  ImGui_ImplOpenGL3_Init();
  ImGui_ImplGlfw_InitForOpenGL(glfw_window, true);

  const float fontSize = 18.0f;
  auto font_path = fs_get_font_path();
  std::string font_fnames[] = {
    font_path + "/Meiryo.ttf",
    font_path + "/fa-solid-900.ttf",
  };

  auto font = io.Fonts->AddFontDefault();
  font->Scale = 1.0f;
  io.FontDefault = io.Fonts->AddFontFromFileTTF(font_fnames[0].c_str(), fontSize, nullptr, io.Fonts->GetGlyphRangesJapanese());

  ImFontConfig config;
  config.MergeMode = true;
  config.MergeMode = true;
  config.GlyphMinAdvanceX = fontSize;
  static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
  io.Fonts->AddFontFromFileTTF(font_fnames[1].c_str(), fontSize * 0.8, &config, icons_ranges);
  LOG_F(2, "[imgui] Building font %s %f", font_fnames[1].c_str(), fontSize);
  io.Fonts->Build();

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

  const auto c = ImGui::GetStyle().Colors;
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(c[ImGuiCol_WindowBg].x, c[ImGuiCol_WindowBg].y, c[ImGuiCol_WindowBg].z, 0.0f));
  ImGui::PushStyleColor(ImGuiCol_DockingEmptyBg, ImVec4(c[ImGuiCol_DockingEmptyBg].x, c[ImGuiCol_DockingEmptyBg].y, c[ImGuiCol_DockingEmptyBg].z, 0.0f));
  auto app = GUIManager::Get();
  app->dockspace_id = ImGui::DockSpaceOverViewport();
  ImGui::PopStyleColor(2);
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
  { // gui thread
    detail::gui_new_frame();
    detail::update_gui_panels();
    detail::gui_render_to_screen();
  }

  { // renderer thread
    detail::update_renderer_thread();
  }
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
