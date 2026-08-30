#include <cstring>
#include <imgui.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/gui/export_window.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

namespace {

bool g_open               = false;
int g_plugin_index        = -1;
bool g_current_frame_only = false;
char g_path_buf[512]      = "";
cutil::Prop g_props;

struct ExportRun {
  OutputPluginTable* plugin = nullptr;
  Composition* comp         = nullptr;
  void* handle              = nullptr;
  int fstart = 0, fend = 0, cur = 0;
  Ref<Image> frame_buf;
};
ExportRun g_run;
bool is_running() { return g_run.plugin != nullptr; }

// PropInfoのbool/int32/float/stringのみ扱う縮小版エディタ(Entity/mtxに依存しないためwd_entt_props_editorとは別実装)
void draw_props_editor(const cutil::PropInfo* info, cutil::Prop* props) {
  if(info == nullptr) return;
  for(const auto& f : info->fields) {
    if(!props->contains(f.name)) continue;
    if(f.type == cutil::prop_info_of<bool>()) {
      bool v = props->get<bool>(f.name);
      if(ImGui::Checkbox(f.name, &v)) props->set<bool>(f.name, v);
    } else if(f.type == cutil::prop_info_of<int32_t>()) {
      int32_t v = props->get<int32_t>(f.name);
      if(ImGui::DragInt(f.name, &v, 1.0f, (int)f.min_value, (int)f.max_value)) props->set<int32_t>(f.name, v);
    } else if(f.type == cutil::prop_info_of<float>()) {
      float v = props->get<float>(f.name);
      if(ImGui::DragFloat(f.name, &v, f.drag_speed, f.min_value, f.max_value)) props->set<float>(f.name, v);
    } else if(f.type == cutil::prop_info_of<std::string>()) {
      std::string s = props->get<std::string>(f.name);
      char buf[256];
      std::strncpy(buf, s.c_str(), sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';
      if(ImGui::InputText(f.name, buf, sizeof(buf))) props->set<std::string>(f.name, std::string(buf));
    }
  }
}

void start_export() {
  auto* comp = Composition::GetActiveComp();
  if(comp == nullptr || g_plugin_index < 0) return;
  auto& plugins = detail::AppMain::Get()->output_plugins;
  if(g_plugin_index >= (int)plugins.size()) return;

  g_run.plugin = &plugins[g_plugin_index];
  g_run.comp   = comp;
  g_run.fstart = g_current_frame_only ? comp->get_frame() : comp->fstart;
  g_run.fend   = g_current_frame_only ? comp->get_frame() : comp->fend;
  g_run.cur    = g_run.fstart;
  g_run.handle = g_run.plugin->fn_open(g_path_buf, (int)comp->size[0], (int)comp->size[1], comp->framerate, g_props);
  if(g_run.handle == nullptr) g_run.plugin = nullptr; // fn_open失敗時はrunningにしない
}

// 1フレーム分だけ進める(UIをブロックしないためUpdate()毎に1枚ずつ処理する)
void step_export() {
  CPURenderer renderer;
  if(renderer.render_frame(g_run.comp, g_run.cur, g_run.frame_buf)) g_run.plugin->fn_write_frame(g_run.handle, g_run.frame_buf.get(), g_run.cur - g_run.fstart);
  g_run.cur++;
  if(g_run.cur > g_run.fend) {
    g_run.plugin->fn_close(g_run.handle);
    g_run = ExportRun();
  }
}

} // namespace

void open_export_window() {
  g_open        = true;
  auto& plugins = detail::AppMain::Get()->output_plugins;
  if(g_plugin_index < 0 && !plugins.empty()) {
    g_plugin_index = 0;
    g_props        = plugins[0].defaults;
  }
}

void ExportWindow::Update() {
  if(!g_open) return;
  if(ImGui::Begin("エクスポート", &g_open)) {
    if(is_running()) {
      const int total = g_run.fend - g_run.fstart + 1;
      const int done  = g_run.cur - g_run.fstart;
      ImGui::ProgressBar(total > 0 ? (float)done / (float)total : 1.0f);
      ImGui::Text("%d / %d フレーム", done, total);
      step_export();
    } else {
      auto& plugins = detail::AppMain::Get()->output_plugins;
      if(plugins.empty()) {
        ImGui::TextUnformatted("出力プラグインが登録されていません");
      } else {
        if(ImGui::BeginCombo("出力形式", g_plugin_index >= 0 ? plugins[g_plugin_index].name : "")) {
          for(int i = 0; i < (int)plugins.size(); i++) {
            bool selected = i == g_plugin_index;
            if(ImGui::Selectable(plugins[i].name, selected)) {
              g_plugin_index = i;
              g_props        = plugins[i].defaults;
            }
          }
          ImGui::EndCombo();
        }

        if(g_plugin_index >= 0) {
          auto& plg = plugins[g_plugin_index];
          ImGui::InputText("出力先", g_path_buf, sizeof(g_path_buf));
          ImGui::SameLine();
          if(ImGui::Button("...")) {
            std::vector<std::string> exts;
            for(auto* e : plg.extensions)
              if(e && e[0]) exts.push_back(e);
            std::string picked = select_save_file_dialog("エクスポート先を選択", g_path_buf, exts);
            if(!picked.empty()) std::strncpy(g_path_buf, picked.c_str(), sizeof(g_path_buf) - 1);
          }

          ImGui::Checkbox("現在フレームのみ", &g_current_frame_only);
          draw_props_editor(&plg.props, &g_props);

          if(ImGui::Button("エクスポート") && g_path_buf[0] != '\0') start_export();
        }
      }
    }
  }
  ImGui::End();
}

} // namespace mu
