#pragma once

#include <functional>
#include <memory>
#include <movutl/render2d/renderer.hpp>
#include <string>
#include <vector>

namespace mu {

using RendererFactory = std::function<std::unique_ptr<Renderer>()>;

constexpr const char* kDefaultRendererName = "cpu";

// 名前→ファクトリの登録(同名は上書き)。"cpu"は組み込みで常に登録済み
void register_renderer(const std::string& name, RendererFactory factory);
std::vector<std::string> renderer_names();

// 未登録名はログを出して"cpu"にフォールバックする(必ず非nullを返す)
std::unique_ptr<Renderer> create_renderer(const std::string& name);

// 優先順位: cli(--renderer=) > env(MOVUTL_RENDERER) > Config(movutl_cnf.json) > "cpu"。空文字は未指定扱い
std::string resolve_renderer_name(const std::string& cli, const std::string& env, const std::string& config);

// 起動時に一度だけ決定し、以降はこの名前を使う(設定変更は次回起動から反映)
void set_renderer_cli_override(const std::string& name);
void init_active_renderer(); // Config::Load後に呼ぶ
const std::string& active_renderer_name();
std::unique_ptr<Renderer> create_active_renderer();

} // namespace mu
