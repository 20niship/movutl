#include <doctest/doctest.h>
#include <imgui.h>
#include <movutl/binding/imgui_binding.hpp>

// InputText系はウィジェット描画APIのためNewFrame〜EndFrame内でのみ安全に呼べる
TEST_CASE("ImGui::InputText_: 変更フラグと編集後文字列を返す") {
  ImGui::CreateContext();
  ImGuiIO& io    = ImGui::GetIO();
  io.DisplaySize = ImVec2(800, 600);
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

  io.DeltaTime = 1.0f / 60.0f;
  ImGui::NewFrame();
  ImGui::Begin("test_window");

  auto [changed, text] = ImGui::InputText_("label", "hello");
  CHECK_FALSE(changed);
  CHECK(text == "hello");

  auto [changed2, text2] = ImGui::InputTextWithHint_("label2", "hint", "world");
  CHECK_FALSE(changed2);
  CHECK(text2 == "world");

  ImGui::End();
  ImGui::EndFrame();
  ImGui::DestroyContext();
}
