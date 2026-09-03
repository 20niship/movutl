#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/aviutl_script/aviutl_filter_bridge.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>

using namespace mu;
using namespace mu::detail;

TEST_CASE("aviutl_script_parser: track0/check0とスクリプト本体を分離できる") {
  std::string text = "--track0:しきい値,0,255,128,1\n"
                     "--check0:反転,1\n"
                     "@テスト効果\n"
                     "obj.putpixeldata(obj.getpixeldata())\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 1);
  CHECK(defs[0].name == "テスト効果");
  REQUIRE(defs[0].tracks.size() == 1);
  CHECK(defs[0].tracks[0].name == "しきい値");
  CHECK(defs[0].tracks[0].min_value == 0.0f);
  CHECK(defs[0].tracks[0].max_value == 255.0f);
  CHECK(defs[0].tracks[0].default_value == 128.0f);
  REQUIRE(defs[0].checks.size() == 1);
  CHECK(defs[0].checks[0].name == "反転");
  CHECK(defs[0].checks[0].default_value == true);
  CHECK(defs[0].lua_body.find("obj.putpixeldata") != std::string::npos);
  CHECK(defs[0].lua_body.find('@') == std::string::npos);
}

TEST_CASE("aviutl_script_parser: 複数の@ブロックを個別に分離できる") {
  std::string text = "@効果A\n"
                     "obj.putpixeldata(obj.getpixeldata())\n"
                     "@効果B\n"
                     "obj.putpixeldata(obj.getpixeldata())\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 2);
  CHECK(defs[0].name == "効果A");
  CHECK(defs[1].name == "効果B");
}

namespace {
FilterPluginTable* find_filter(const char* name) {
  for(auto& f : AppMain::Get()->filters)
    if(std::string(f.name.c_str()) == name) return &f;
  return nullptr;
}
} // namespace

TEST_CASE("register_aviutl_filter: 2値化スクリプトをフィルタとして登録・実行できる") {
  std::string text = "--track0:しきい値,0,255,128,1\n"
                     "@AviUtlテスト2値化\n"
                     "local unpack = table.unpack or unpack\n"
                     "local w = obj.getinfo(\"image_w\")\n"
                     "local h = obj.getinfo(\"image_h\")\n"
                     "local px = obj.getpixeldata()\n"
                     "local buf = {}\n"
                     "for i = 1, w*h do\n"
                     "  local o = (i-1)*4\n"
                     "  local r,g,b,a = string.byte(px, o+1, o+4)\n"
                     "  local v = (r+g+b)/3 >= track0 and 255 or 0\n"
                     "  buf[o+1], buf[o+2], buf[o+3], buf[o+4] = v, v, v, a\n"
                     "end\n"
                     "obj.putpixeldata(string.char(unpack(buf)))\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 1);
  REQUIRE(register_aviutl_filter(defs[0]));

  FilterPluginTable* plg = find_filter("AviUtlテスト2値化");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_proc != nullptr);
  REQUIRE(plg->props.fields.size() == 1);
  CHECK(std::string(plg->props.fields[0].name) == "しきい値");

  Image img(2, 2);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255); // 平均200

  FilterInData fin;
  fin.img = &img;

  cutil::Prop p = plg->defaults;
  p.set<float>("しきい値", 128.0f);
  CHECK(plg->fn_proc(plg, &fin, p));
  CHECK(img[0][0] == 255); // 200 >= 128 なので白

  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255); // 1回目の実行結果で上書きされているので再初期化
  p.set<float>("しきい値", 250.0f);
  CHECK(plg->fn_proc(plg, &fin, p));
  CHECK(img[0][0] == 0); // 200 < 250 なので黒
}
