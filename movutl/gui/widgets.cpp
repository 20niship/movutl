#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <functional>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/entity_gizmo.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/widgets.hpp>

namespace mu {

bool wd_color_edit(const char* name, Vec4b* col) {
  float cf[4]  = {(*col)[0] / 255.0f, (*col)[1] / 255.0f, (*col)[2] / 255.0f, (*col)[3] / 255.0f};
  bool changed = ImGui::ColorEdit4(name, cf);
  if(changed) *col = Vec4b(cf[0] * 255, cf[1] * 255, cf[2] * 255, cf[3] * 255);
  return changed;
}

bool wd_table_begin(const char* id) {
  if(!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_PadOuterX)) return false;
  ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthStretch, 0.4f);
  ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch, 0.6f);
  return true;
}

void wd_table_end() { ImGui::EndTable(); }

void wd_row(const char* label, const char* desc) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  // 幅が足りないラベルは省略記号で切る(隣の値欄へはみ出さない)。全文はツールチップで見せる
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const float w    = std::max(1.0f, ImGui::GetContentRegionAvail().x);
  const float h    = ImGui::GetTextLineHeight();
  ImGui::RenderTextEllipsis(ImGui::GetWindowDrawList(), pos, ImVec2(pos.x + w, pos.y + h), pos.x + w, pos.x + w, label, nullptr, nullptr);
  ImGui::Dummy(ImVec2(w, h));
  if(ImGui::IsItemHovered() && (desc && desc[0] || ImGui::CalcTextSize(label).x > w)) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(label);
    if(desc && desc[0]) ImGui::TextDisabled("%s", desc);
    ImGui::EndTooltip();
  }
  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-FLT_MIN);
}

bool wd_grid9(const char* id, int selected, int* picked, const char* const* tips) {
  bool clicked           = false;
  const float avail      = ImGui::GetContentRegionAvail().x;
  const float sp         = 2.0f;
  const float cell       = std::clamp((avail - sp * 2) / 3.0f, 16.0f, 30.0f);
  const ImVec2 origin    = ImGui::GetCursorScreenPos();
  auto* dl               = ImGui::GetWindowDrawList();
  const ImU32 col_sel    = ImGui::GetColorU32(ImGuiCol_ButtonActive);
  const ImU32 col_hov    = ImGui::GetColorU32(ImGuiCol_ButtonHovered);
  const ImU32 col_bg     = ImGui::GetColorU32(ImGuiCol_FrameBg);
  const ImU32 col_border = ImGui::GetColorU32(ImGuiCol_Border);
  ImGui::PushID(id);
  for(int i = 0; i < 9; i++) {
    const int cx = i % 3, cy = i / 3;
    const ImVec2 mn(origin.x + cx * (cell + sp), origin.y + cy * (cell * 0.75f + sp));
    const ImVec2 mx(mn.x + cell, mn.y + cell * 0.75f);
    ImGui::SetCursorScreenPos(mn);
    ImGui::PushID(i);
    ImGui::InvisibleButton("##c", ImVec2(cell, cell * 0.75f));
    const bool hov = ImGui::IsItemHovered();
    if(ImGui::IsItemClicked()) {
      if(picked) *picked = i;
      clicked = true;
    }
    if(hov && tips && tips[i]) ImGui::SetTooltip("%s", tips[i]);
    ImGui::PopID();
    dl->AddRectFilled(mn, mx, i == selected ? col_sel : (hov ? col_hov : col_bg), 3.0f);
    dl->AddRect(mn, mx, col_border, 3.0f);
    // 枠内の該当位置に点を打つ(左上/上/右上/左/中央/右/左下/下/右下)
    const float pad = 4.0f, d = 3.0f;
    const float px = cx == 0 ? mn.x + pad : cx == 1 ? (mn.x + mx.x) * 0.5f - d * 0.5f : mx.x - pad - d;
    const float py = cy == 0 ? mn.y + pad : cy == 1 ? (mn.y + mx.y) * 0.5f - d * 0.5f : mx.y - pad - d;
    dl->AddRectFilled(ImVec2(px, py), ImVec2(px + d, py + d), i == selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(200, 200, 200, 180));
  }
  ImGui::PopID();
  ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + 3 * (cell * 0.75f + sp)));
  return clicked;
}

namespace {
// infoの各フィールドを編集UIとして描画し、変更があればapply(変更分のProp)で反映する
void edit_props(Entity* e, const cutil::PropInfo* info, const cutil::Prop& p, const std::function<void(const cutil::Prop&)>& apply) {
  if(!info) return;
  if(!wd_table_begin("##props")) return;
  for(const auto& f : info->fields) {
    if(!p.contains(f.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name.c_str(), f.name);
      continue;
    }
    ImGui::PushID(f.name);
    bool changed = false;
    cutil::Prop newp;
    const char* label_       = f.label[0] ? f.label : f.name;
    const char* name_        = "##v"; // ラベルは左の列(wd_row)に出すので、ウィジェット側のラベルは隠す
    wd_row(label_, f.desc);
    const bool is_path_field = std::string(f.name) == "path" || std::string(f.name) == "path_";

    if(f.type == cutil::prop_info_of<bool>()) {
      bool v = p.get<bool>(f.name);
      if(ImGui::Checkbox(name_, &v)) {
        newp.set<bool>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<float>()) {
      float v = p.get<float>(f.name);
      if(std::string(f.name) == "alpha_") { // 内部値は0-1だが表示は%
        float pct = v * 100.0f;
        if(ImGui::DragFloat(name_, &pct, 1.0f, 0.0f, 100.0f, "%.0f")) {
          newp.set<float>(f.name, std::clamp(pct, 0.0f, 100.0f) / 100.0f);
          changed = true;
        }
      } else if(ImGui::DragFloat(name_, &v, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<float>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<int32_t>()) {
      int32_t v = p.get<int32_t>(f.name);
      if(std::string(f.name) == "shape_type_") {
        static const char* kShapeNames[] = {"三角形", "四角形", "六角形", "円", "カスタムパス"};
        int idx                          = std::clamp(v, 0, 4);
        if(ImGui::Combo(name_, &idx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
          newp.set<int32_t>(f.name, idx);
          changed = true;
        }
      } else if(ImGui::InputInt(name_, &v)) {
        newp.set<int32_t>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<uint8_t>()) {
      int v = p.get<uint8_t>(f.name);
      if(ImGui::InputInt(name_, &v)) {
        newp.set<uint8_t>(f.name, static_cast<uint8_t>(std::clamp(v, 0, 255)));
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<std::string>()) {
      std::string s = p.get<std::string>(f.name);
      char buf[256];
      strncpy(buf, s.c_str(), sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';
      if(is_path_field) {
        std::string label = s.empty() ? "ファイルを選択" : std::filesystem::path(s).filename().string();
        std::string btn   = std::string(ICON_FA_FOLDER_OPEN " ") + label;
        if(ImGui::Button(btn.c_str(), ImVec2(-FLT_MIN, 0))) {
          std::string picked = select_file_dialog("ファイルを選択", {});
          if(!picked.empty()) {
            newp.set<std::string>(f.name, picked);
            changed = true;
          }
        }
      } else if(ImGui::InputText(name_, buf, sizeof(buf))) {
        newp.set<std::string>(f.name, std::string(buf));
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec2>()) {
      Vec2 v = p.get<Vec2>(f.name);
      if(ImGui::DragFloat2(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec2>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec3>()) {
      Vec3 v = p.get<Vec3>(f.name);
      if(ImGui::DragFloat3(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec3>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4>()) {
      Vec4 v = p.get<Vec4>(f.name);
      if(ImGui::DragFloat4(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec4>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4b>()) {
      Vec4b v = p.get<Vec4b>(f.name);
      if(wd_color_edit(name_, &v)) {
        newp.set<Vec4b>(f.name, v);
        changed = true;
      }
    }

    if(changed) {
      {
        std::lock_guard<std::mutex> lock(e->mtx);
        apply(newp);
        if(is_path_field) {
          e->reload_asset(); // パス変更時は新しいファイルを読み込み直す
          auto new_path = newp.get<std::string>(f.name);
          if(!new_path.empty()) e->name = std::filesystem::path(new_path).stem().string();
        }
      }
      // Compositionの全フレームではなく、このEntityが映る範囲だけを無効化する(Positionドラッグ等が重くなるのを防ぐ)
      if(auto* comp = e->get_comp()) comp->invalidate_cache_range(e->fstart_, e->fend_);
    }
    ImGui::PopID();
  }
  wd_table_end();
}

// 基点を画像枠上の9点(左上〜右下)へ置くプリセット。keep_visualなら見た目が動かないようposも補正する。現在の基点がいずれかの点と一致すれば選択表示にする
void edit_anchor_presets(Entity* e) {
  static bool keep_visual = true;
  auto* comp              = e->get_comp();
  if(!comp) return;
  static const char* kTips[9] = {"左上", "上", "右上", "左", "中央", "右", "左下", "下", "右下"};
  EntityGizmo g;
  if(!entity_gizmo_of(*e, GizmoPt{(double)comp->size[0], (double)comp->size[1]}, g)) return;
  int selected = -1;
  for(int i = 0; i < 9; i++) {
    const GizmoPt a = gizmo_anchor_preset(i % 3 - 1, i / 3 - 1, g.src_size, g.origin_offset);
    if(std::abs(a.x - g.xform.anchor.x) < 0.5 && std::abs(a.y - g.xform.anchor.y) < 0.5) selected = i;
  }
  if(!wd_table_begin("##anchor_presets")) return;
  wd_row("基点プリセット");
  int picked = -1;
  if(wd_grid9("##anchor_grid", selected, &picked, kTips)) {
    const GizmoPt anchor = gizmo_anchor_preset(picked % 3 - 1, picked / 3 - 1, g.src_size, g.origin_offset);
    {
      std::lock_guard<std::mutex> lock(e->mtx);
      entity_apply_xform(*e, gizmo_set_anchor(g.xform, anchor, keep_visual));
    }
    comp->invalidate_cache_range(e->fstart_, e->fend_);
  }
  wd_row("");
  ImGui::Checkbox("位置を保持", &keep_visual);
  if(ImGui::IsItemHovered()) ImGui::SetTooltip("基点を動かしても見た目の位置が変わらないよう位置を補正する");
  wd_table_end();
}
} // namespace

void wd_entt_transform_editor(Entity* e) {
  MU_ASSERT(e);
  if(!e->has_transform()) return;
  ImGui::PushID(e);
  edit_props(e, e->getTransformPropsInfo(), e->getTransformProps(), [&](const cutil::Prop& np) { e->setTransformProps(np); });
  edit_anchor_presets(e);
  ImGui::PopID();
}

void wd_entt_props_editor(Entity* e) {
  MU_ASSERT(e);
  ImGui::PushID(e);
  edit_props(e, e->getPropsInfo(), e->getProps(), [&](const cutil::Prop& np) { e->setProps(np); });
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) { MU_ASSERT(e); }

void wd_image_inspector(Entity* e) { MU_ASSERT(e); }

} // namespace mu
