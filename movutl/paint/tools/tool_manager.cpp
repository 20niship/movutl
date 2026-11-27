#include <movutl/paint/tools/brush_tool.hpp>
#include <movutl/paint/tools/eyedrop_tool.hpp>
#include <movutl/paint/tools/tool_manager.hpp>
#include <utility>

namespace mu {

ToolManager::ToolManager() {
  auto brush = [&](const char* id, const char* name, const char* key) { tools_.push_back(std::make_unique<BrushTool>(id, name, key)); };
  brush("pen", "ペン", "p");
  brush("pencil", "鉛筆", "p");
  brush("brush", "筆", "b");
  brush("airbrush", "エアブラシ", "b");
  brush("deco", "デコレーション", "b");
  brush("eraser", "消しゴム", "e");
  tools_.push_back(std::make_unique<EyedropTool>("eyedropper", "スポイト", "i"));
  selected_ = tools_.front().get();
}

Tool* ToolManager::find(const std::string& id) const {
  for(const auto& t : tools_)
    if(t->id == id) return t.get();
  return nullptr;
}

bool ToolManager::select(const std::string& id) {
  Tool* t = find(id);
  if(!t) return false;
  selected_     = t;
  last_[t->key] = t;
  return true;
}

bool ToolManager::select_by_key(const std::string& key) {
  std::vector<Tool*> group;
  for(const auto& t : tools_)
    if(t->key == key) group.push_back(t.get());
  if(group.empty()) return false;
  for(size_t i = 0; i < group.size(); i++)
    if(group[i] == selected_) return select(group[(i + 1) % group.size()]->id);
  auto it = last_.find(key);
  return select((it == last_.end() ? group.front() : it->second)->id);
}

bool ToolManager::begin_temporary(const std::string& id) {
  Tool* t = find(id);
  if(!t) return false;
  temp_ = t;
  return true;
}

bool ToolManager::press(const ToolContext& c, const PenSample& p) {
  if(op_ || !c.doc) return false;
  ctx_ = c;
  if(!active().press(*ctx_, p)) return false;
  op_ = &active();
  return true;
}

void ToolManager::drag(const PenSample& p) {
  if(op_) op_->drag(*ctx_, p);
}

void ToolManager::tick(float dt) {
  if(op_) op_->tick(*ctx_, dt);
}

std::unique_ptr<PaintEdit> ToolManager::release() {
  if(!op_) return nullptr;
  return std::exchange(op_, nullptr)->release(*ctx_);
}

void ToolManager::cancel() {
  if(op_) std::exchange(op_, nullptr)->cancel(*ctx_);
}

} // namespace mu
