#include <algorithm>
#include <cmath>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/filter.hpp>
#include <movutl/plugin/plugin.hpp>
#include <set>
//
#include <movutl/asset/audio.hpp>
#include <movutl/asset/camera.hpp>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/midi.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>

namespace mu {
struct InputPluginTable;

Ref<Entity> Entity::CreateEntity(const char* name, EntityType type) {
  Ref<Entity> e = nullptr;
  switch(type) {
    case EntityType_Movie: e = cutil::make_ref<Movie>(); break;
    case EntityType_Image: e = cutil::make_ref<Image>(); break;
    case EntityType_3DText: e = cutil::make_ref<TextEntt>(); break;
    case EntityType_Audio: e = cutil::make_ref<AudioEntt>(); break;
    case EntityType_Framebuffer: e = cutil::make_ref<FramebufferEntt>(); break;
    case EntityType_Group: e = cutil::make_ref<GroupEntt>(); break;
    case EntityType_Polygon: e = cutil::make_ref<ShapeEntt>(); break;
    case EntityType_Camera: e = cutil::make_ref<Camera3D>(); break;
    // 1個のフラグで複数のLuaスクリプトを表すため、実体はsetProps()内でscript_name経由でdef_を再解決する(ここではdef_未設定のまま生成するだけでよい)
    case EntityType_Custom: e = cutil::make_ref<CustomObjectEntt>(); break;
    case EntityType_Scene: e = cutil::make_ref<CompoRefEntt>(); break;
    case EntityType_SceneAudio: e = cutil::make_ref<CompoAudioEntt>(); break;
    case EntityType_Midi: e = cutil::make_ref<MidiEntt>(); break;
    default: break;
  }
  if(!e) {
    LOG_F(ERROR, "[Entity::Create] Unknown type %d", type);
    return nullptr;
  }
  e->name = name;
  Project::Get()->entities.push_back(e);
  e->guid_ = Project::Get()->entities.size();
  return e;
}

Ref<Entity> Entity::Find(const char* name) {
  for(auto& e : Project::Get()->entities) {
    if(e->name == name) return e;
  }
  return nullptr;
}

cutil::Prop Entity::getSaveProps() const {
  cutil::Prop p;
  p.set<int32_t>("type", (int32_t)getType());
  p.set<std::string>("name", name.c_str());
  p.set<int32_t>("guid", (int32_t)guid_);
  p.set_child("props", getProps());
  p.set_child("trk", getTrackProps());
  p.set_child("xform", getTransformProps());
  if(getPropsInfo() || has_transform()) {
    ensure_anim_props();
    p.set_child("anim_props", anim_props_.save());
  }

  // filters_はgetTrackPropsInfo()の自動生成対象外(std::vector<FilterParam>)のため個別にシリアライズする
  cutil::Prop filters_p;
  filters_p.set<int32_t>("count", (int32_t)filters_.size());
  for(size_t i = 0; i < filters_.size(); i++) {
    const auto& f = filters_[i];
    cutil::Prop fp;
    fp.set<int32_t>("plugin_guid", (int32_t)(f.plg_ ? f.plg_->guid : 0));
    fp.set<bool>("enabled", f.enabled);
    fp.set_child("anim_params", f.props.save());
    filters_p.set_child(("filter_" + std::to_string(i)).c_str(), fp);
  }
  p.set_child("filters", filters_p);
  return p;
}

Ref<Entity> Entity::fromSaveProps(const cutil::Prop& p) {
  auto type = (EntityType)cutil::get_or<int32_t>(p, "type", 0);
  auto name = cutil::get_or<std::string>(p, "name", "");
  auto e    = Entity::CreateEntity(name.c_str(), type);
  if(!e) return nullptr;
  e->guid_ = (uint64_t)cutil::get_or<int32_t>(p, "guid", (int32_t)e->guid_);
  if(p.contains("props")) e->setProps(p.get_child("props"));
  if(p.contains("trk")) e->setTrackProps(p.get_child("trk"));
  if(p.contains("xform")) e->setTransformProps(p.get_child("xform"));
  if(e->getPropsInfo() || e->has_transform()) {
    e->ensure_anim_props(); // setProps()適用後の値を各プロパティの初期キーフレームにする
    if(p.contains("anim_props")) e->anim_props_.load_keys(p.get_child("anim_props"));
  }

  if(p.contains("filters")) {
    const auto& filters_p = p.get_child("filters");
    int32_t count         = cutil::get_or<int32_t>(filters_p, "count", 0);
    auto* main            = detail::AppMain::Get();
    for(int32_t i = 0; i < count; i++) {
      const auto& fp = filters_p.get_child(("filter_" + std::to_string(i)).c_str());
      int32_t guid   = cutil::get_or<int32_t>(fp, "plugin_guid", 0);
      FilterParam f;
      for(auto& plg : main->filters) {
        if((int32_t)plg.guid == guid) {
          f.plg_ = &plg;
          break;
        }
      }
      if(!f.plg_) {
        LOG_F(WARNING, "Entity::fromSaveProps: filter plugin (guid=%d) not found, skipping", guid);
        continue;
      }
      f.enabled = cutil::get_or<bool>(fp, "enabled", true);
      f.props.add_props(f.plg_->defaults);
      if(fp.contains("anim_params")) f.props.load_keys(fp.get_child("anim_params"));
      e->filters_.push_back(f);
    }
  }

  e->reload_asset(); // pathはsetProps()でコピーされるだけなので、ここで独立した読み込みプラグインのインスタンスを持たせる
  return e;
}

Composition* Entity::get_comp() const {
  auto pj = Project::Get();
  for(int i = 0; i < pj->compos_.size(); i++) {
    for(auto& layer : pj->compos_[i]->layers) {
      for(auto& e : layer.entts) {
        if(e.get() == this) return pj->compos_[i].get();
      }
    }
  }
  return nullptr;
}

Entity::~Entity() {
  if(in_plg_ && in_handle_ && in_plg_->fn_close) in_plg_->fn_close(in_handle_);
}

std::string EntityInfo::str() const {
  char buf[256];
  sprintf(buf, "EntityInfo: Flag%d %dx%d %d frames %.3f fps", (int)flag, width, height, nframes, framerate);
  return std::string(buf);
}

namespace {
thread_local const GroupXform* tls_parent_xform = nullptr;
}

GroupXform GroupXform::compose(const GroupXform& child) const {
  const double rad = rotation * M_PI / 180.0;
  const double c = std::cos(rad), sn = std::sin(rad);
  const double s  = scale / 100.0;
  const double px = child.pos[0] * s, py = child.pos[1] * s;
  GroupXform out;
  out.pos      = Vec3((float)(pos[0] + px * c - py * sn), (float)(pos[1] + px * sn + py * c), pos[2] + child.pos[2]);
  out.scale    = (float)(child.scale * s);
  out.rotation = rotation + child.rotation;
  out.alpha    = alpha * child.alpha;
  return out;
}

GroupXformScope::GroupXformScope(const GroupXform* parent) : prev_(tls_parent_xform) { tls_parent_xform = parent; }
GroupXformScope::~GroupXformScope() { tls_parent_xform = prev_; }

GroupXform Entity::world_xform() const {
  GroupXform local{pos_, scale_, rotation_, alpha_};
  return tls_parent_xform ? tls_parent_xform->compose(local) : local;
}

bool Entity::composite(const Image& src, Image* target, const Vec2& origin_offset) const {
  MOVUTL_ZONE_SCOPED_N("Entity::composite");
  MU_ASSERT(target);
  Placement pl;
  const GroupXform w = world_xform(); // 親グループ変換込みの実効変換
  pl.x = w.pos[0], pl.y = w.pos[1];
  pl.anchor_x = anchor_[0] + origin_offset[0], pl.anchor_y = anchor_[1] + origin_offset[1];
  pl.scale_x = pl.scale_y = w.scale / 100.0;
  pl.aspect               = aspect_;
  pl.rot_x = rot_x_, pl.rot_y = rot_y_, pl.rot_z = w.rotation;
  pl.alpha = w.alpha;
  pl.blend = blend_;
  return src.place(target, pl);
}

void Entity::ensure_anim_props() const {
  if(anim_props_.size() > 0 || (!getPropsInfo() && !has_transform())) return;
  if(getPropsInfo()) anim_props_.add_props(getProps());
  if(has_transform()) anim_props_.add_props(getTransformProps()); // 位置/拡大率/回転/不透明度もキーフレームでアニメーションできる
  // 文字列(text/path等)はアニメーション不要。残すとapply_animated_props()が古い値で上書きして入力がリセットされる
  if(const auto* info = getPropsInfo()) {
    for(const auto& f : info->fields)
      if(f.type == cutil::prop_info_of<std::string>()) anim_props_.erase(f.name);
  }
}

void Entity::apply_animated_props(int frame) {
  if(!getPropsInfo() && !has_transform()) return;
  ensure_anim_props();
  const auto p = anim_props_.get(rel_frame(frame));
  if(getPropsInfo()) setProps(p);
  if(has_transform()) setTransformProps(p);
}

void Entity::on_len_change_done(int old_start) {
  const int shift = std::max(fstart_, 0) - std::max(old_start, 0);
  const int len   = std::max(fend_ - fstart_, 0);
  auto apply      = [&](AnimProps& a) {
    a.shift_frames(shift);
    a.trim_end((uint32_t)len);
  };
  if(getPropsInfo() || has_transform()) {
    ensure_anim_props();
    apply(anim_props_);
  }
  for(auto& f : filters_) apply(f.props);
  if(auto* comp = get_comp()) comp->invalidate_cache_range(std::min(old_start, fstart_), std::max(fend_, old_start));
}

std::vector<uint32_t> Entity::collect_animated_frames() const {
  ensure_anim_props();
  std::set<uint32_t> frames;
  const uint32_t off = (uint32_t)std::max(fstart_, 0);
  for(int i = 0; i < (int)anim_props_.props.size(); i++) {
    if(!anim_props_.has_animation(i)) continue; // 単一キー(=アニメーションしていない初期値)は中間点として扱わない
    for(uint32_t f : anim_props_.keyframe_frames(i)) frames.insert(f + off);
  }
  for(auto& filt : filters_)
    for(int i = 0; i < (int)filt.props.props.size(); i++) {
      if(!filt.props.has_animation(i)) continue;
      for(uint32_t f : filt.props.keyframe_frames(i)) frames.insert(f + off);
    }
  return std::vector<uint32_t>(frames.begin(), frames.end());
}

bool Entity::move_keyframes_at(uint32_t old_abs, uint32_t new_abs) {
  if(old_abs == new_abs) return false;
  const uint32_t old_frame = rel_frame((int)old_abs), new_frame = rel_frame((int)new_abs);
  if(old_frame == new_frame) return false;
  ensure_anim_props();
  bool any = false;
  for(int i = 0; i < (int)anim_props_.props.size(); i++)
    if(anim_props_.has_key_at(i, old_frame)) any |= anim_props_.move_keyframe(i, old_frame, new_frame);
  for(auto& filt : filters_)
    for(int i = 0; i < (int)filt.props.props.size(); i++)
      if(filt.props.has_key_at(i, old_frame)) any |= filt.props.move_keyframe(i, old_frame, new_frame);
  return any;
}

bool Entity::erase_keyframes_at(uint32_t abs_frame) {
  const uint32_t frame = rel_frame((int)abs_frame);
  ensure_anim_props();
  bool any = false;
  for(int i = 0; i < (int)anim_props_.props.size(); i++) any |= anim_props_.erase_keyframe(i, frame);
  for(auto& filt : filters_)
    for(int i = 0; i < (int)filt.props.props.size(); i++) any |= filt.props.erase_keyframe(i, frame);
  return any;
}

bool Entity::render_filters(Composition* cmp, Image* img, int frame) {
  MU_ASSERT(cmp != nullptr);
  MOVUTL_ZONE_SCOPED_N("Entity::render_filters");
  for(int i = 0; i < filters_.size(); i++) {
    auto& f = filters_[i];
    if(!f.enabled) continue;
    MU_ASSERT(f.plg_ != nullptr);
    if(!f.plg_->fn_proc) {
      LOG_F(ERROR, "Plugin %s has no render function", f.plg_->name.c_str());
      continue;
    }
    MOVUTL_ZONE_SCOPED;
    MOVUTL_ZONE_NAME(f.plg_->name.c_str(), f.plg_->name.size());
    void* fp = f.plg_;
    FilterInData in;
    in.img   = img;
    in.compo = cmp;
    in.entt  = this;
    in.frame = frame;
    if(!f.plg_->fn_proc(fp, &in, f.props.get(rel_frame(frame)))) {
      LOG_F(ERROR, "Plugin %s render failed", f.plg_->name.c_str());
      return false;
    }
  }
  return true;
}

} // namespace mu
