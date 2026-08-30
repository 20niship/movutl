#include <cstring>
#include <imgui.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/app/prop_editor.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/gui/export_window.hpp>
#include <movutl/render2d/renderer.hpp>
#include <thread>

namespace mu {

namespace {

int g_plugin_index        = -1;
bool g_current_frame_only = false;
char g_path_buf[512]      = "";
cutil::Prop g_props;

// バックグラウンドスレッドで実行(gui.cpp/mainmenu.cppがis_exporting()中UIを無効化するためplugin/comp/path/propsは実行中不変)
void export_thread_func(OutputPluginTable* plugin, Composition* comp, std::string path, cutil::Prop props, int fstart, int fend) {
  auto& prog         = get_export_progress();
  prog.total_frames  = fend - fstart + 1;
  prog.current_frame = 0;
  void* handle       = plugin->fn_open(path.c_str(), (int)comp->size[0], (int)comp->size[1], comp->framerate, props);
  if(handle != nullptr) {
    CPURenderer renderer;
    Ref<Image> frame_buf;
    for(int f = fstart; f <= fend; f++) {
      if(prog.cancel_requested.load()) break;
      if(renderer.render_frame(comp, f, frame_buf)) plugin->fn_write_frame(handle, frame_buf.get(), f - fstart);
      prog.current_frame = f - fstart + 1;
    }
    plugin->fn_close(handle);
  }
  prog.running = false;
}

void start_export() {
  auto* comp = Composition::GetActiveComp();
  if(comp == nullptr || g_plugin_index < 0) return;
  auto& plugins = detail::AppMain::Get()->output_plugins;
  if(g_plugin_index >= (int)plugins.size()) return;

  const int fstart = g_current_frame_only ? comp->get_frame() : comp->fstart;
  const int fend   = g_current_frame_only ? comp->get_frame() : comp->fend;

  auto& prog            = get_export_progress();
  prog.cancel_requested = false;
  prog.running          = true;
  std::thread(export_thread_func, &plugins[g_plugin_index], comp, std::string(g_path_buf), g_props, fstart, fend).detach();
}

} // namespace

void open_export_window(int plugin_index) {
  auto& plugins = detail::AppMain::Get()->output_plugins;
  if(plugin_index < 0 || plugin_index >= (int)plugins.size()) return;
  g_plugin_index = plugin_index;
  g_props        = plugins[plugin_index].defaults;
}

bool ExportWindow::always_enabled_during_export() const { return true; }

void ExportWindow::Update() {
  const bool running = is_exporting();
  if(!running && g_plugin_index < 0) return;

  if(ImGui::Begin("エクスポート")) {
    if(running) {
      auto& prog      = get_export_progress();
      const int total = prog.total_frames.load();
      const int done  = prog.current_frame.load();
      ImGui::ProgressBar(total > 0 ? (float)done / (float)total : 1.0f);
      ImGui::Text("%d / %d フレーム", done, total);
      ImGui::TextUnformatted("Escキーでキャンセルできます");
      if(ImGui::Button("キャンセル")) request_cancel_export();
    } else {
      auto& plugins = detail::AppMain::Get()->output_plugins;
      if(g_plugin_index < 0 || g_plugin_index >= (int)plugins.size()) {
        ImGui::TextUnformatted("出力プラグインが選択されていません");
      } else {
        auto& plg = plugins[g_plugin_index];
        ImGui::Text("出力形式: %s", plg.name);

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
  ImGui::End();
}

} // namespace mu
