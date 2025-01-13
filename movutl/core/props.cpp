#include <movutl/core/props.hpp>

namespace mu {

void PropsInfo::add_float_prop(                             //
  const char* name, const char* category, const char* desc, //
  float def, float min, float max,                          //
  float step, bool is_angle, bool is_radian) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.min = min;
  prop.max = max;
  prop.step = step;
  prop.value_.float_ = def;
  prop.value_.float_.is_angle = is_angle;
  prop.value_.float_.is_radian = is_radian;
  prop.type = PropT_Float;
  props.push_back(prop);
}

void PropsInfo::add_int_prop(const char* name, const char* category, const char* desc, int def, int min, int max, int step) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.min = min;
  prop.max = max;
  prop.step = step;
  prop.value_.int_ = def;
  prop.type = PropT_Int;
  props.push_back(prop);
}

void PropsInfo::add_bool_prop(const char* name, const char* category, const char* desc, bool def) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.value_.bool_ = def;
  prop.type = PropT_Bool;
  props.push_back(prop);
}

void PropsInfo::add_string_prop(const char* name, const char* category, const char* desc, const char* def) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  strcpy(prop.value_.str_, def);
  prop.type = PropT_String;
  props.push_back(prop);
}

void PropsInfo::add_vec2_prop(const char* name, const char* cat, const char* desc, Vec2 def, float min, float max, float step) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = cat;
  prop.desc = desc;
  prop.min = min;
  prop.max = max;
  prop.step = step;
  prop.value_.vec2_ = def;
  prop.type = PropT_Vec2;
  props.push_back(prop);
}

void PropsInfo::add_vec3_prop(const char* name, const char* category, const char* desc, Vec3 def, float min, float max, float step) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.min = min;
  prop.max = max;
  prop.step = step;
  prop.value_.vec3_ = def;
  prop.type = PropT_Vec3;
  props.push_back(prop);
}

void PropsInfo::add_color_prop(const char* name, const char* category, const char* desc, Vec4b def) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.value_.color_ = def;
  prop.type = PropT_Color;
  props.push_back(prop);
}

void PropsInfo::add_selection_prop(const char* name, const char* category, const char* desc, const char* items[], uint8_t count, uint8_t def) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  prop.value_.selection_.count = count;
  prop.value_.selection_.default_ = def;
  for(uint8_t i = 0; i < count; i++) strcpy(prop.value_.selection_.items[i], items[i]);
  prop.type = PropT_Selection;
  props.push_back(prop);
}

void PropsInfo::add_path_prop(const char* name, const char* category, const char* desc, const char* def) {
  PropInfoBase prop;
  prop.name = name;
  prop.categ = category;
  prop.desc = desc;
  strcpy(prop.value_.str_, def);
  prop.type = PropT_Path;
  props.push_back(prop);
}

void PropsInfo::set_last_prop_dispname(const char* dispname) {
  if(props.size() == 0) return;
  props.back().dispname = dispname;
}
void PropsInfo::set_last_prop_category(const char* category) {
  if(props.size() == 0) return;
  props.back().categ = category;
}
void PropsInfo::set_last_prop_desc(const char* desc) {
  if(props.size() == 0) return;
  props.back().desc = desc;
}

void PropsInfo::set_last_prop_readonly(bool readonly) {
  if(props.size() == 0) return;
  props.back().readonly = readonly;
}

} // namespace mu
