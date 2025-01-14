#include <fstream>

#include <nlohmann/json.hpp>

#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/props.hpp>
#include <movutl/core/props/custom_values.hpp>


namespace mu {
using json = nlohmann::json;

static Props::Value parse_json_data_impl(const nlohmann::json& v) {
  Props::Value d;

  if(v.is_number_float())
    d = v.get<float>();
  else if(v.is_number_integer())
    d = v.get<int>();
  else if(v.is_number_unsigned())
    d = (int)v.get<unsigned int>();
  else if(v.is_boolean())
    d = v.get<bool>();
  /*else if(v.is_null()) {*/
  /*  LOG_F(2, "json object is null");*/
  else if(v.is_string()) {
    std::string str = v.get<std::string>();
    if(is_vector<2>(str))
      d = get_vec<2>(str);
    else if(is_vector<3>(str))
      d = get_vec<3>(str);
    else if(get_color_rgba_(str))
      d = get_color_rgba_(str).value();
    else
      d = str;
  } else if(v.is_array()) {
    Props p;
    for(auto& e : v) p.push_back(parse_json_data_impl(e));
    d = p;
  } else if(v.is_object()) {
    Props p;
    for(auto& [k, x] : v.items()) p[k] = parse_json_data_impl(x);
    d = p;
  }
  return d;
}

inline Props parse_json_data_impl_wrapper(const std::string& str, const std::string& filename) {
  json js;
  try {
    js = json::parse(str, nullptr, true, true);
  } catch(json::parse_error& ex) {
    auto input_str = str.size() > 100 ? str.substr(0, 100) + "......" : str;
    auto input = filename.empty() ? input_str : filename;
    LOG_F(ERROR, "[json::parse] Failed to parse \"%s\" :%s last token=%d", input.c_str(), ex.what(), (int)ex.byte);
  } catch(std::exception& ex) {
    auto input = filename.empty() ? str : filename;
    LOG_F(ERROR, "[std::exception] Failed to parse \"%s\" :%s", input.c_str(), ex.what());
  }

  auto d = parse_json_data_impl(js);
  if(!std::holds_alternative<Props>(d)) {
    auto input_str = str.size() > 100 ? str.substr(0, 100) + "......" : str;
    LOG_F(ERROR, "Failed to parse json: %s", input_str.c_str());
    return {};
  }
  return std::get<Props>(d);
}

Props Props::LoadJson(const std::string& json) {
  return parse_json_data_impl_wrapper(json, "");
}

Props Props::LoadJsonFile(const std::string& json) {
  std::ifstream fs(json);
  std::string str((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
  return parse_json_data_impl_wrapper(str, json);
}
std::string Props::str() const {
  return "Props(" + std::to_string(values.size()) + ")";
}

std::string Props::summary() const {
  return this->dump_json();
}

struct ToJsonVisitor {
  nlohmann::json* js;
  ToJsonVisitor(nlohmann::json* js) : js(js) {}
  void operator()(const int& value) const { *js = value; }
  void operator()(const float& value) const { *js = value; }
  void operator()(const std::string& value) const { *js = value; }
  void operator()(const bool& value) const { *js = value; }
  void operator()(const Vec2& value) const { *js = dumpjson<2>(value); }
  void operator()(const Vec3& value) const { *js = dumpjson<3>(value); }
  void operator()(const Vec4& value) const { *js = dumpjson<4>(value); }
  void operator()(const Vec4b& value) const { *js = dumpjson<4>(value); }
  void operator()(const Entity* value) const { *js = (value == nullptr) ? nullptr : value->name.c_str(); }
  void operator()(const Props& value) const {
    if(value.is_array()) {
      for(auto& [k, v] : value.values) {
        nlohmann::json x;
        std::visit(ToJsonVisitor(&x), v);
        js->push_back(x);
      }
    } else {
      nlohmann::json j;
      for(const auto& [k, v] : value.values) {
        nlohmann::json x;
        std::visit(ToJsonVisitor(&x), v);
        j[k] = x;
      }
      *js = j;
    }
  }
};

nlohmann::json to_json(const Props& d) {
  nlohmann::json j;
  for(const auto& [k, v] : d.values) {
    nlohmann::json x;
    std::visit(ToJsonVisitor(&x), v);
    j[k] = x;
  }
  return j;
}

std::string Props::dump_json(int indent) const {
  auto j = to_json(*this);
  std::string str = j.dump(indent);
  return str;
}

bool Props::dump_json_file(const std::string& filename, int indent) const {
  auto str = this->dump_json(indent);
  FILE* fp = fopen(filename.c_str(), "w");
  fprintf(fp, "%s", str.c_str());
  fclose(fp);
  return true;
}
} // namespace mu
