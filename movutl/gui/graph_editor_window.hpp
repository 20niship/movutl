#pragma once
#include <cstdint>
#include <movutl/gui/gui.hpp>

namespace mu {

// ImGuiのドラッグドロップペイロードはmemcpy前提のためstd::stringではなく固定長char配列で持つ
struct GraphDragPayload {
  uint64_t entity_guid = 0;
  int filter_index     = -1; // -1: Entity本体のanim_props_, それ以外: filters_[filter_index].props
  char prop_name[64]   = {};
};
inline constexpr const char* kGraphDragDropId = "MOVUTL_ANIM_CHANNEL";

class GraphEditorWindow final : public UIPanel {
public:
  void Update() override;
};

} // namespace mu
