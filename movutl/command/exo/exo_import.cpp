#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/command/exo/exo_import.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/timeline.hpp>
#include <sstream>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace mu {
namespace {

std::string cp932_to_utf8(const std::string& src) {
  if(src.empty()) return {};
#ifdef _WIN32
  int wn = MultiByteToWideChar(932, 0, src.data(), (int)src.size(), nullptr, 0);
  if(wn <= 0) return src;
  std::wstring w(wn, L'\0');
  MultiByteToWideChar(932, 0, src.data(), (int)src.size(), w.data(), wn);
  int un = WideCharToMultiByte(CP_UTF8, 0, w.data(), wn, nullptr, 0, nullptr, nullptr);
  std::string out(un, '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), wn, out.data(), un, nullptr, nullptr);
  return out;
#else
  iconv_t cd = iconv_open("UTF-8", "CP932");
  if(cd == (iconv_t)-1) return src;
  std::string out(src.size() * 4, '\0');
  char* in       = const_cast<char*>(src.data());
  size_t in_left = src.size();
  char* op       = out.data();
  size_t o_left  = out.size();
  // 不正バイトは読み飛ばして変換を続ける
  while(in_left > 0) {
    if(iconv(cd, &in, &in_left, &op, &o_left) == (size_t)-1) {
      if(errno == EILSEQ || errno == EINVAL) {
        ++in;
        --in_left;
        continue;
      }
      break;
    }
  }
  iconv_close(cd);
  out.resize(op - out.data());
  return out;
#endif
}

static void append_utf8(std::string& out, uint32_t cp) {
  if(cp < 0x80) {
    out += (char)cp;
  } else if(cp < 0x800) {
    out += (char)(0xC0 | (cp >> 6));
    out += (char)(0x80 | (cp & 0x3F));
  } else if(cp < 0x10000) {
    out += (char)(0xE0 | (cp >> 12));
    out += (char)(0x80 | ((cp >> 6) & 0x3F));
    out += (char)(0x80 | (cp & 0x3F));
  } else {
    out += (char)(0xF0 | (cp >> 18));
    out += (char)(0x80 | ((cp >> 12) & 0x3F));
    out += (char)(0x80 | ((cp >> 6) & 0x3F));
    out += (char)(0x80 | (cp & 0x3F));
  }
}

// exoのtext=は UTF-16LE のバイト列を16進文字列にしたもの。0x0000で終端(以降は0埋め)
std::string utf16le_hex_to_utf8(const std::string& hex) {
  auto hv = [](char c) -> int {
    if(c >= '0' && c <= '9') return c - '0';
    if(c >= 'a' && c <= 'f') return c - 'a' + 10;
    if(c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  std::vector<uint16_t> units;
  for(size_t i = 0; i + 3 < hex.size(); i += 4) {
    int b0 = hv(hex[i]) * 16 + hv(hex[i + 1]);
    int b1 = hv(hex[i + 2]) * 16 + hv(hex[i + 3]);
    if(b0 < 0 || b1 < 0) break;
    uint16_t u = (uint16_t)(b0 | (b1 << 8));
    if(u == 0) break;
    units.push_back(u);
  }
  std::string out;
  for(size_t i = 0; i < units.size(); ++i) {
    uint32_t u = units[i];
    if(u >= 0xD800 && u < 0xDC00 && i + 1 < units.size() && units[i + 1] >= 0xDC00 && units[i + 1] < 0xE000) {
      u = 0x10000 + ((u - 0xD800) << 10) + (units[++i] - 0xDC00);
    }
    append_utf8(out, u);
  }
  return out;
}

using Section = std::map<std::string, std::string>;

std::map<std::string, Section> parse_ini(const std::string& text) {
  std::map<std::string, Section> out;
  std::istringstream ss(text);
  std::string line, cur;
  while(std::getline(ss, line)) {
    if(!line.empty() && line.back() == '\r') line.pop_back();
    if(line.empty()) continue;
    if(line.front() == '[' && line.back() == ']') {
      cur = line.substr(1, line.size() - 2);
      out[cur];
      continue;
    }
    auto eq = line.find('=');
    if(eq == std::string::npos || cur.empty()) continue;
    out[cur][line.substr(0, eq)] = line.substr(eq + 1);
  }
  return out;
}

std::string get(const Section& s, const char* key, const std::string& def = "") {
  auto it = s.find(key);
  return it == s.end() ? def : it->second;
}

// トラックバー値は"0.0,100.0,1"のような 開始値,終了値,移動方式 形式のことがある。先頭値のみ使う
float getf(const Section& s, const char* key, float def = 0.f) {
  auto v = get(s, key);
  if(v.empty()) return def;
  return (float)atof(v.c_str());
}

int geti(const Section& s, const char* key, int def = 0) {
  auto v = get(s, key);
  return v.empty() ? def : atoi(v.c_str());
}

Vec4b parse_color(const std::string& hex, Vec4b def) {
  if(hex.size() != 6) return def;
  auto v = (uint32_t)strtoul(hex.c_str(), nullptr, 16);
  return Vec4b((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF, 255);
}

// exoの透明度(0-100, 100で完全透明) -> 不透明度(0-1)
float parse_alpha(const Section& s) { return std::clamp((100.f - getf(s, "透明度", 0.f)) / 100.f, 0.f, 1.f); }

// exoの標準描画(X/Y/Z/拡大率/回転/透明度)をEntity共通の変換へ反映する。exoの座標系(中心原点・Y下向き・%・度)はEntityの規約と同じ
void apply_standard_draw(Entity& e, const Section* draw) {
  if(!draw) return;
  e.pos_      = Vec3(getf(*draw, "X"), getf(*draw, "Y"), getf(*draw, "Z"));
  float scale = getf(*draw, "拡大率", 100.f);
  e.scale_    = Vec2(scale, scale);
  e.rotation_ = getf(*draw, "回転");
  e.alpha_    = parse_alpha(*draw);
}

// exo内のfile=を実在するパスへ解決する。
//  - 相対パスはexoのあるディレクトリ基準
//  - 絶対パス(Windowsのドライブ付き含む)が存在しない場合は、先頭の階層を1つずつ削ったパスをexoのディレクトリ基準で探す
//    (exoをPCから移動した場合でも、素材がexoの近くに置かれていれば見つかる)
// 見つからなければ(\を/へ変えた)元のパスを返す
std::string resolve_media_path(const std::string& raw, const std::filesystem::path& base) {
  namespace fs  = std::filesystem;
  std::string p = raw;
  std::replace(p.begin(), p.end(), '\\', '/');
  if(p.empty()) return p;
  std::error_code ec;
  bool drive = p.size() >= 2 && p[1] == ':';
  bool abs   = drive || fs::path(p).is_absolute();
  if(abs) {
    if(fs::exists(p, ec)) return p;
  } else if(fs::exists(base / p, ec)) {
    return (base / p).lexically_normal().string();
  }
  std::vector<std::string> comps;
  std::stringstream ss(drive ? p.substr(2) : p);
  for(std::string c; std::getline(ss, c, '/');)
    if(!c.empty() && c != "." && c != "..") comps.push_back(c);
  for(size_t i = 0; i < comps.size(); ++i) {
    fs::path cand = base;
    for(size_t j = i; j < comps.size(); ++j) cand /= comps[j];
    if(fs::exists(cand, ec)) return cand.lexically_normal().string();
  }
  LOG_F(WARNING, "import_exo_file: media not found: %s", raw.c_str());
  return p;
}

std::string stem_of(const std::string& p) { return std::filesystem::path(p).stem().string(); }

void set_range(const Ref<Entity>& e, int start, int end) {
  e->fstart_ = start - 1; // exoは1始まり
  e->fend_   = end - 1;   // 終了フレームも含む
}

// 追加したEntity数を返す。ファイルが開けない場合は-1
int import_exo_file(const char* path) {
  std::ifstream ifs(path, std::ios::binary);
  if(!ifs) {
    LOG_F(ERROR, "import_exo_file: cannot open %s", path);
    return -1;
  }
  std::string raw((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  const auto base_dir = std::filesystem::absolute(std::filesystem::path(path)).parent_path();
  auto ini            = parse_ini(cp932_to_utf8(raw));

  Composition* comp = Composition::GetActiveComp();
  if(!comp) {
    Project::New();
    comp = Composition::GetActiveComp();
  }
  MU_ASSERT(comp);

  // [N](Nは整数)を番号順に処理する。[N.M]はそのエフェクト
  std::vector<int> ids;
  for(auto& [k, v] : ini)
    if(!k.empty() && std::all_of(k.begin(), k.end(), ::isdigit)) ids.push_back(atoi(k.c_str()));
  std::sort(ids.begin(), ids.end());

  std::vector<std::pair<Ref<Entity>, int>> pending; // (Entity, exo上のレイヤーindex)
  for(int n : ids) {
    auto it            = ini.find(std::to_string(n));
    const Section& obj = it->second;
    int start          = geti(obj, "start", 1);
    int end            = geti(obj, "end", start);
    int layer          = std::max(geti(obj, "layer", 1), 1) - 1;

    // [N.M]をエフェクトとして_name -> Sectionで引けるようにする(先頭が描画元/メディア)
    std::vector<const Section*> effects;
    for(int m = 0;; ++m) {
      auto e = ini.find(std::to_string(n) + "." + std::to_string(m));
      if(e == ini.end()) break;
      effects.push_back(&e->second);
    }
    if(effects.empty()) continue;
    auto find_fx = [&](const char* name) -> const Section* {
      for(auto* fx : effects)
        if(get(*fx, "_name") == name) return fx;
      return nullptr;
    };
    const Section& src = *effects[0];
    auto kind          = get(src, "_name");
    auto draw          = find_fx("標準描画");
    auto play          = find_fx("標準再生");

    Ref<Entity> ent;
    if(kind == "動画ファイル") {
      auto file = resolve_media_path(get(src, "file"), base_dir);
      auto mov  = Movie::Create(stem_of(file).c_str(), file.c_str());
      apply_standard_draw(*mov, draw);
      mov->start_frame_ = geti(src, "再生位置", 0);
      mov->speed        = getf(src, "再生速度", 100.f);
      mov->loop_        = geti(src, "ループ再生") != 0;
      ent               = mov;
    } else if(kind == "音声ファイル") {
      auto file      = resolve_media_path(get(src, "file"), base_dir);
      auto a         = AudioEntt::Create(stem_of(file).c_str(), file.c_str());
      a->offset_sec_ = getf(src, "再生位置");
      a->speed       = getf(src, "再生速度", 100.f);
      a->loop_       = geti(src, "ループ再生") != 0;
      if(play) a->volume_ = getf(*play, "音量", 100.f);
      ent = a;
    } else if(kind == "画像ファイル") {
      auto file  = resolve_media_path(get(src, "file"), base_dir);
      auto img   = Image::Create(stem_of(file).c_str(), file.c_str());
      img->guid_ = Project::Get()->entities.size();
      if(draw) {
        img->pos      = Vec3(getf(*draw, "X"), getf(*draw, "Y"), getf(*draw, "Z"));
        float sc      = getf(*draw, "拡大率", 100.f) / 100.f;
        img->scale    = Vec2(sc, sc);
        img->rotation = getf(*draw, "回転") * 3.14159265f / 180.f;
        img->alpha    = parse_alpha(*draw);
      }
      ent = img;
    } else if(kind == "図形") {
      // exoのtype: 0=背景 1=円 2=四角形 3=三角形 4=五角形 5=六角形(それ以外は四角形扱い)
      static const ShapeType types[] = {ShapeType_Rect, ShapeType_Circle, ShapeType_Rect, ShapeType_Triangle, ShapeType_Hexagon, ShapeType_Hexagon};
      int t                          = geti(src, "type", 2);
      auto shp                       = ShapeEntt::Create("shape", types[(t >= 0 && t < 6) ? t : 2]);
      float size                     = getf(src, "サイズ", 100.f);
      shp->size_                     = Vec2(size, size);
      shp->color_                    = parse_color(get(src, "color"), shp->color_);
      if(draw) {
        shp->pos_   = Vec3(getf(*draw, "X"), getf(*draw, "Y"), getf(*draw, "Z"));
        shp->size_  = shp->size_ * (getf(*draw, "拡大率", 100.f) / 100.f);
        shp->rot_   = getf(*draw, "回転") * 3.14159265f / 180.f;
        shp->alpha_ = (uint8_t)std::lround(parse_alpha(*draw) * 255);
      }
      Project::Get()->entities.push_back(shp);
      shp->guid_ = Project::Get()->entities.size();
      ent        = shp;
    } else if(kind == "テキスト") {
      auto t           = TextEntt::Create(utf16le_hex_to_utf8(get(src, "text")).c_str(), get(src, "font").c_str());
      t->color_        = parse_color(get(src, "color"), t->color_);
      t->border_color_ = parse_color(get(src, "color2"), t->border_color_);
      if(draw) {
        t->pos_     = Vec3(getf(*draw, "X"), getf(*draw, "Y"), getf(*draw, "Z"));
        t->scale_x_ = t->scale_y_ = getf(*draw, "拡大率", 100.f) / 100.f;
        t->rot_                   = getf(*draw, "回転") * 3.14159265f / 180.f;
        t->alpha_                 = (uint8_t)std::lround(parse_alpha(*draw) * 255);
      }
      // TextEntt::CreateはProject::entitiesへ登録もguid採番もしないため自前で行う
      Project::Get()->entities.push_back(t);
      t->guid_ = Project::Get()->entities.size();
      ent      = t;
    } else {
      LOG_F(WARNING, "import_exo_file: [%d] unsupported object '%s', skipped", n, kind.c_str());
      continue;
    }
    set_range(ent, start, end);
    pending.push_back({ent, layer});
  }

  // 既存のどのEntityともフレーム範囲が重ならない位置まで、exoのレイヤー全体を下(layer index増加方向)へずらす
  auto free_at = [&](int off) {
    std::lock_guard<std::mutex> lock(comp->mtx);
    for(auto& [e, l] : pending) {
      int li = l + off;
      if(li >= (int)comp->layers.size()) continue;
      for(auto& o : comp->layers[li].entts)
        if(o && o->fstart_ <= e->fend_ && e->fstart_ <= o->fend_) return false;
    }
    return true;
  };
  int layer_offset = 0;
  while(!free_at(layer_offset)) ++layer_offset;
  int count = 0;
  for(auto& [e, l] : pending) {
    int li = l + layer_offset;
    {
      std::lock_guard<std::mutex> lock(comp->mtx);
      if(li >= (int)comp->layers.size()) comp->layers.resize(li + 1);
    }
    comp->insert_entity(e, li); // insert_entityはlayers[li]が存在する前提
    ++count;
  }

  // Compositionの表示範囲を全Entityのfstart_の最小値/fend_の最大値から再計算する(exoの[exedit] lengthは使わない)
  if(count > 0) {
    int fmin = INT_MAX, fmax = INT_MIN;
    {
      std::lock_guard<std::mutex> lock(comp->mtx);
      for(auto& l : comp->layers)
        for(auto& e : l.entts) {
          if(!e) continue;
          fmin = std::min(fmin, e->fstart_);
          fmax = std::max(fmax, e->fend_);
        }
    }
    if(fmin <= fmax) {
      comp->fstart = fmin;
      comp->fend   = fmax;
    }
    comp->invalidate_cache_all();
  }
  LOG_F(INFO, "import_exo_file: %s -> %d objects (range %d-%d)", path, count, comp->fstart, comp->fend);
  return count;
}

struct ExoImportCommand final : mCommand {
  CommandStatus on_start() override {
    std::string path = arg;
    if(path.empty()) path = select_file_dialog("EXOを読み込む", {"exo"});
    if(path.empty()) return CommandStatus::Failed;
    if(import_exo_file(path.c_str()) < 0) return CommandStatus::Failed;
    RequestTimelineFit(); // タイムラインの表示範囲を取り込んだEntity全体に合わせる
    return CommandStatus::Finished;
  }
};
} // namespace

void register_exo_command() { register_command<ExoImportCommand>({"import_exo", "EXOを読み込む", "AviUtlのexoを読み込み全トラックをアクティブCompositionへ追加する", "ctrl+shift+i", {"exo"}}); }

} // namespace mu
