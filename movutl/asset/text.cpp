#include <movutl/asset/project.hpp>
#include <movutl/asset/text.hpp>

namespace mu {
bool TextEntt::render(Composition* cmp) {
  MU_FAIL("nOT IMPLEMENTED");
  return false;
}

Ref<TextEntt> TextEntt::Create(const char* text, const char* font) {
  auto ent = Ref<TextEntt>(new TextEntt());
  ent->text = text;
  ent->font = font;
  return ent;
}

} // namespace mu
