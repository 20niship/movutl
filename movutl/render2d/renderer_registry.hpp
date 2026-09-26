#pragma once

#include <functional>
#include <memory>
#include <movutl/render2d/renderer.hpp>
#include <string>
#include <vector>

namespace mu {

using RendererFactory = std::function<std::unique_ptr<Renderer>()>;

// 名前→ファクトリの登録(同名は上書き)。"cpu"/"vulkan"は組み込みで常に登録済み
void register_renderer(const std::string& name, RendererFactory factory);
std::vector<std::string> renderer_names();

// 未登録名はログを出して"cpu"にフォールバックする(必ず非nullを返す)
std::unique_ptr<Renderer> create_renderer(const std::string& name);

// Config::Get()->renderer(movutl_cnf.json)を毎回読むので、設定変更は次フレームから反映される(再起動不要)
const std::string& active_renderer_name();
std::unique_ptr<Renderer> create_active_renderer();

} // namespace mu
