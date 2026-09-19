#include <cstdlib>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/command/exo/exo_effects.hpp>
#include <movutl/command/exo/exo_report.hpp>

namespace mu {

namespace {
const char* const kExoBlendNames[] = {"通常", "加算", "減算", "乗算", "スクリーン", "オーバーレイ", "比較(明)", "比較(暗)", "輝度", "色差", "陰影", "明暗", "差分"};
}

BlendType exo_blend_type(int v, bool* supported) {
  if(supported) *supported = true;
  switch(v) {
    case 0: return Blend_Alpha;
    case 1: return Blend_Add;
    case 2: return Blend_Sub;
    case 3: return Blend_Mul;
    case 4: return Blend_Screen;
    case 5: return Blend_Overlay;
    case 6: return Blend_Lighten;
    case 7: return Blend_Darken;
    default:
      if(supported) *supported = false;
      return Blend_Alpha;
  }
}

void apply_exo_blend(Entity& e, const ExoSection* draw) {
  if(!draw) return;
  auto it = draw->find("blend");
  if(it == draw->end() || it->second.empty()) return;
  int v          = atoi(it->second.c_str());
  bool supported = true;
  e.blend_       = exo_blend_type(v, &supported);
  if(!supported) {
    std::string name = (v >= 0 && v < (int)(sizeof(kExoBlendNames) / sizeof(*kExoBlendNames))) ? kExoBlendNames[v] : std::to_string(v);
    exo_import_report().add("合成モード「" + name + "」は未対応のため通常で代用しました");
  }
}

void apply_exo_object_flags(Entity& e, const ExoSection& obj) {
  auto flag = [&](const char* key, int def) {
    auto it = obj.find(key);
    return it == obj.end() || it->second.empty() ? def : atoi(it->second.c_str());
  };
  e.camera_ctrl_ = flag("camera", 0) != 0;
  e.clipping_up_ = flag("clipping", 0) != 0;
  if(flag("overlay", 1) == 0) exo_import_report().add("overlay=0(現在のレイヤーを同時に表示しない)は未対応です");
}

void apply_exo_header(Composition& comp, const ExoSection& exedit) {
  {
    std::lock_guard<std::mutex> lock(comp.mtx);
    for(auto& l : comp.layers)
      if(!l.entts.empty()) return;
  }
  auto num = [&](const char* key) {
    auto it = exedit.find(key);
    return it == exedit.end() ? 0.0 : atof(it->second.c_str());
  };
  int w = (int)num("width"), h = (int)num("height");
  if(w > 0 && h > 0) comp.resize(w, h);
  // AviUtlのフレームレートは rate/scale (例: 30000/1001)
  double rate = num("rate"), scale = num("scale");
  if(rate > 0) comp.framerate = (float)(rate / (scale > 0 ? scale : 1.0));
  int ar = (int)num("audio_rate"), ach = (int)num("audio_ch");
  if((ar > 0 && ar != comp.audio_sample_rate) || (ach > 0 && ach != comp.audio_channels)) {
    if(ar > 0) comp.audio_sample_rate = ar;
    if(ach > 0) comp.audio_channels = ach;
    comp.audio_buf = cutil::make_ref<AudioRingBuffer>(comp.audio_sample_rate, comp.audio_channels); // ponytail: 再生中の差し替えは考慮しない(取り込み直後は停止中が前提)
  }
  comp.invalidate_cache_all();
}

ExoTrack parse_exo_track(const std::string& v) {
  ExoTrack t;
  std::vector<std::string> parts;
  size_t pos = 0;
  while(true) {
    auto c = v.find(',', pos);
    parts.push_back(v.substr(pos, c == std::string::npos ? c : c - pos));
    if(c == std::string::npos) break;
    pos = c + 1;
  }
  t.start = t.end = (float)atof(parts[0].c_str());
  if(parts.size() >= 3) {
    t.end  = (float)atof(parts[1].c_str());
    t.mode = atoi(parts[2].c_str());
  }
  return t;
}

AniInterpType exo_track_interp(int mode, bool* exact) {
  if(exact) *exact = true;
  switch(mode) {
    case 1: return LINEAR;
    case 3: return LINEAR; // 瞬間移動は呼び出し側で終点直前まで開始値を保持するキーフレームにする
    case 7:
      if(exact) *exact = false;
      return EaseInOut;
    default:
      if(exact) *exact = false;
      return LINEAR;
  }
}

namespace {

struct FxParam {
  const char* exo;  // exoのキー名
  const char* prop; // movutlフィルタのプロパティ名
};
struct FxMap {
  const char* exo_name;
  const char* filter;
  std::vector<FxParam> params;
};

// ponytail: 対応するmovutlフィルタとプロパティが確認できたもののみ。増やすときはここへ1行足す
const std::vector<FxMap>& fx_table() {
  static const std::vector<FxMap> t = {
    {"ぼかし", "ぼかし", {{"範囲", "range"}}},
    {"方向ぼかし", "方向ぼかし", {{"範囲", "range"}, {"角度", "angle"}}},
    {"放射ぼかし", "放射ぼかし", {{"範囲", "range"}}},
    {"グロー", "グロー", {{"強さ", "intensity"}, {"拡散", "range"}}},
    {"モザイク", "モザイク", {{"サイズ", "block_size"}}},
    {"シャープ", "シャープ", {{"強さ", "strength"}}},
    {"縁取り", "縁取り", {{"サイズ", "width"}, {"color", "color"}}},
    {"単色化", "単色化", {{"強さ", "strength"}, {"color", "color"}}},
  };
  return t;
}

bool is_handled_elsewhere(const std::string& name) { return name == "標準描画" || name == "拡張描画" || name == "標準再生"; }

std::string get_str(const ExoSection& s, const char* key) {
  auto it = s.find(key);
  return it == s.end() ? std::string() : it->second;
}

// float型プロパティへトラックバー値を反映する(開始/終了で値が変わるならEntityの表示区間に2点のキーフレームを打つ)
void set_float_track(PAniClip<float>& clip, const ExoTrack& t, int fstart, int fend, std::string* inexact_note) {
  clip.keys.clear();
  if(!t.animated() || fend <= fstart) {
    clip.keys.push_back(AnimKeyframe<float>(t.start));
    clip.keys.back().frame_ = (uint32_t)std::max(fstart, 0);
    return;
  }
  bool exact = true;
  auto type  = exo_track_interp(t.mode, &exact);
  if(!exact && inexact_note) *inexact_note = "移動方式" + std::to_string(t.mode);
  clip.add_keyframe((uint32_t)std::max(fstart, 0), t.start, type);
  if(t.mode == 3 && fend - 1 > fstart) clip.add_keyframe((uint32_t)(fend - 1), t.start, LINEAR); // 瞬間移動: 終点フレームまで開始値を保持
  clip.add_keyframe((uint32_t)fend, t.end, LINEAR);
}

void apply_one_effect(Entity& e, const FxMap& m, const ExoSection& fx) {
  FilterPluginTable* plg = nullptr;
  for(auto& f : detail::AppMain::Get()->filters)
    if(std::string(f.name.c_str()) == m.filter) plg = &f;
  if(!plg) {
    exo_import_report().add(std::string("エフェクト「") + m.exo_name + "」に対応するフィルタ「" + m.filter + "」が見つかりません");
    return;
  }
  FilterParam fp;
  fp.plg_ = plg;
  fp.props.add_props(plg->defaults);
  fp.enabled = get_str(fx, "_disable") != "1";

  std::string inexact;
  for(auto& prm : m.params) {
    auto v = get_str(fx, prm.exo);
    if(v.empty()) continue;
    for(auto& var : fp.props.props) {
      if(auto* c = std::get_if<PAniClip<float>>(&var); c && c->keyname == prm.prop) {
        set_float_track(*c, parse_exo_track(v), e.fstart_, e.fend_, &inexact);
      } else if(auto* cc = std::get_if<PAniClip<Vec4b>>(&var); cc && cc->keyname == prm.prop && v.size() == 6) {
        auto rgb = (uint32_t)strtoul(v.c_str(), nullptr, 16);
        cc->keys.assign(1, AnimKeyframe<Vec4b>(Vec4b((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, 255)));
      }
    }
  }
  if(!inexact.empty()) exo_import_report().add(std::string("エフェクト「") + m.exo_name + "」の" + inexact + "は近似の補間で代用しました");

  // 対応表に無いパラメータ(_name/_disable等の内部キーを除く)を記録する
  std::string dropped;
  for(auto& [k, v] : fx) {
    if(k.empty() || k[0] == '_') continue;
    bool mapped = false;
    for(auto& prm : m.params) mapped |= (k == prm.exo);
    if(!mapped) dropped += (dropped.empty() ? "" : ",") + k;
  }
  if(!dropped.empty()) exo_import_report().add(std::string("エフェクト「") + m.exo_name + "」の未対応パラメータ: " + dropped);
  e.filters_.push_back(fp);
}

} // namespace

void apply_exo_effects(Entity& e, const std::vector<const ExoSection*>& effects) {
  for(size_t i = 1; i < effects.size(); ++i) {
    auto name = get_str(*effects[i], "_name");
    if(is_handled_elsewhere(name)) continue;
    const FxMap* found = nullptr;
    for(auto& m : fx_table())
      if(name == m.exo_name) found = &m;
    if(found)
      apply_one_effect(e, *found, *effects[i]);
    else
      exo_import_report().add("未対応のエフェクト「" + name + "」をスキップしました");
  }
}

} // namespace mu
