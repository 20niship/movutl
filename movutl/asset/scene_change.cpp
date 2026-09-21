#include <algorithm>
#include <cmath>
#include <movutl/asset/project.hpp>
#include <movutl/asset/scene_change.hpp>

namespace mu {

Ref<SceneChangeEntt> SceneChangeEntt::Create(const char* name) {
  auto s  = cutil::make_ref<SceneChangeEntt>();
  s->name = name;
  Project::Get()->entities.push_back(s);
  s->guid_ = Project::Get()->entities.size();
  return s;
}

float SceneChangeProgress(int frame, int fstart, int fend) {
  if(fend <= fstart) return 1.0f;
  return std::clamp((float)(frame - fstart) / (float)(fend - fstart), 0.0f, 1.0f);
}

float SceneChangeWeight(int type, float p, float nx, float ny, bool invert, float blur) {
  if(type == SceneChangeType_Fade) return p;
  float t;
  if(type == SceneChangeType_WipeLR)
    t = nx;
  else if(type == SceneChangeType_WipeUD)
    t = ny;
  else
    t = std::min(1.0f, std::hypot(nx - 0.5f, ny - 0.5f) / 0.70710678f); // 中心0、隅1
  if(invert) t = 1.0f - t;
  blur = std::max(blur, 0.0f);
  // 境界位置 p*(1+blur) が t を通過した所が遷移後。p=0で全面A、p=1で全面B(blur込み)
  if(blur < 1e-6f) return t < p ? 1.0f : 0.0f;
  return std::clamp((p * (1.0f + blur) - t) / blur, 0.0f, 1.0f);
}

bool SceneChangeFromExoName(const std::string& name, int& type, bool& invert) {
  auto has = [&](const char* s) { return name.find(s) != std::string::npos; };
  invert   = false;
  // ponytail: AviUtl標準のシーンチェンジ名の一部のみ(フェード系とワイプ系)。それ以外(スクリプト/画像ワイプ等)は未対応としてfalse
  if(has("フェード"))
    type = SceneChangeType_Fade;
  else if(has("円"))
    type = SceneChangeType_Circle;
  else if(has("上から下") || has("下から上"))
    type = SceneChangeType_WipeUD, invert = has("下から上");
  else if(has("左から右") || has("右から左"))
    type = SceneChangeType_WipeLR, invert = has("右から左");
  else if(has("ワイプ"))
    type = SceneChangeType_WipeLR;
  else
    return false;
  return true;
}

} // namespace mu
