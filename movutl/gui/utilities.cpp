#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/gui/utilities.hpp>

namespace mu {

void add_entities_ui() {
  ImGui::BeginGroup();
  ImGui::Columns(2);
  // text ,image, movie, sound, shape,

  if(ImGui::Button(ICON_FA_FONT " テキスト")) add_new_track("text", EntityType_3DText, 0, 100);
  ImGui::NextColumn();
  if(ImGui::Button(ICON_FA_IMAGE " 画像")) add_new_track("image", EntityType_Image, 0, 100);
  ImGui::NextColumn();
  if(ImGui::Button(ICON_FA_VIDEO " 動画")) add_new_track("movie", EntityType_Movie, 0, 100);
  ImGui::NextColumn();
  if(ImGui::Button(ICON_FA_MUSIC " 音声")) add_new_track("sound", EntityType_Audio, 0, 100);
  ImGui::NextColumn();
  if(ImGui::Button(ICON_FA_DRAW_POLYGON " 図形")) ImGui::OpenPopup("##ADD_SHAPE_POPUP");
  if(ImGui::BeginPopup("##ADD_SHAPE_POPUP")) {
    if(ImGui::Selectable("三角形")) add_new_shape_track("shape", 0, 100, ShapeType_Triangle);
    if(ImGui::Selectable("四角形")) add_new_shape_track("shape", 0, 100, ShapeType_Rect);
    if(ImGui::Selectable("六角形")) add_new_shape_track("shape", 0, 100, ShapeType_Hexagon);
    if(ImGui::Selectable("円")) add_new_shape_track("shape", 0, 100, ShapeType_Circle);
    if(ImGui::Selectable("カスタムパス")) add_new_shape_track("shape", 0, 100, ShapeType_Custom);
    ImGui::EndPopup();
  }
  ImGui::NextColumn();
  if(ImGui::Button(ICON_FA_TV " フレームバッファ")) add_new_track("framebuffer", EntityType_Framebuffer, 0, 100);
  ImGui::NextColumn();
  ImGui::Columns(1);
  ImGui::EndGroup();
}

void UtilityWindow ::Update() {
  ImGui::Begin("ツール");
  add_entities_ui();
  ImGui::End();
}

} // namespace mu
