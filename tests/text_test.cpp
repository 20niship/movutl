#include <doctest/doctest.h>
#include <fstream>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>

using namespace mu;

/// GUIを操作せずにテキスト追加〜描画までを検証するテスト

TEST_CASE("get_available_fonts: システムフォント一覧の取得") {
  const auto fonts = get_available_fonts();
  CHECK(fonts.size() > 0);
  for(const auto& f : fonts) {
    CHECK(fs_exists(f));
    const auto ext = fs_extension(f);
    CHECK((ext == "ttf" || ext == "otf" || ext == "ttc"));
  }
}

TEST_CASE("TextEntt::Create") {
  SUBCASE("デフォルトフォントが選択される") {
    auto t = TextEntt::Create("こんにちは世界");
    REQUIRE(t != nullptr);
    CHECK(t->text == "こんにちは世界");
    /// システムまたはバンドルのフォントが存在する環境では有効なパスが選ばれる
    if(!get_available_fonts().empty() || fs_exists(fs_get_font_path())) {
      CHECK(fs_exists(t->font));
    }
  }

  SUBCASE("存在しないフォント指定でもクラッシュせずフォールバックする") {
    auto t = TextEntt::Create("hello", "/nonexistent/font.ttf");
    REQUIRE(t != nullptr);
    CHECK_FALSE(t->font.empty());
    CHECK(fs_exists(t->font));
  }

  SUBCASE("有効なフォント指定はそのまま使われる") {
    const std::string meiryo = fs_get_font_path() + "/Meiryo.ttf";
    auto t                   = TextEntt::Create("hello", meiryo.c_str());
    REQUIRE(t != nullptr);
    CHECK(t->font == meiryo);
  }
}

TEST_CASE("FontRenderManager::renderText") {
  SUBCASE("無効なフォントパスでもクラッシュせず false を返す") {
    auto img = cutil::make_ref<Image>();
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", "/nonexistent/font.ttf"));
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", "not_a_font.txt"));
  }

  SUBCASE("フォントとして不正な内容のファイルでもクラッシュせず false を返す") {
    const std::string dummy = "/tmp/opencode/movutl_test_not_a_font.ttf";
    {
      std::ofstream ofs(dummy);
      ofs << "this is not a font";
    }
    auto img = cutil::make_ref<Image>();
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", dummy.c_str()));
  }

  SUBCASE("空文字列・nullチェック") {
    auto img = cutil::make_ref<Image>();
    CHECK_FALSE(detail::FontRenderManager::renderText(nullptr, "test", ""));
  }

  SUBCASE("バンドルフォントで日本語テキストを描画できる") {
    const std::string meiryo = fs_get_font_path() + "/Meiryo.ttf";
    if(!fs_exists(meiryo)) return; // フォントがない環境はスキップ
    auto img = cutil::make_ref<Image>();
    CHECK(detail::FontRenderManager::renderText(img.get(), "こんにちは 世界", meiryo.c_str()));
    CHECK_FALSE(img->empty());
    CHECK(img->width > 0);
    CHECK(img->height > 0);
  }
}

namespace {
std::string bundled_font() { return fs_get_font_path() + "/Meiryo.ttf"; }
Ref<Image> render_style(const char* text, const detail::TextStyle& st) {
  auto img = cutil::make_ref<Image>();
  detail::FontRenderManager::renderText(img.get(), text, bundled_font().c_str(), st);
  return img;
}
} // namespace

TEST_CASE("FontRenderManager::renderText: スタイルがサイズに反映される") {
  if(!fs_exists(bundled_font())) return;
  detail::TextStyle base;
  base.size  = 20;
  auto plain = render_style("ABCD", base);
  REQUIRE(plain->width > 0);

  detail::TextStyle big = base;
  big.size              = 40;
  CHECK(render_style("ABCD", big)->height > plain->height * 3 / 2);

  detail::TextStyle wide = base;
  wide.spacing_x         = 10;
  CHECK((int)render_style("ABCD", wide)->width == (int)plain->width + 30); // 4文字の字間は3か所

  detail::TextStyle bold = base;
  bold.bold              = true;
  CHECK(render_style("ABCD", bold)->width > plain->width);

  detail::TextStyle mono = base;
  mono.monospace         = true;
  CHECK((int)render_style("ABCD", mono)->width == 4 * (base.size / 2)); // 半角は size/2

  auto two = render_style("AB\nCD", base);
  CHECK(two->height > plain->height);
  detail::TextStyle gap = base;
  gap.spacing_y         = 7;
  CHECK((int)render_style("AB\nCD", gap)->height == (int)two->height + 7);
}

TEST_CASE("TextEntt: 揃えに応じて基点の位置が文字ブロックの左端・右端・中央になる") {
  if(!fs_exists(bundled_font())) return;
  auto comp = cutil::make_ref<Composition>("t", 200, 200, 30);
  auto t    = TextEntt::Create("ABCD", bundled_font().c_str());
  Image target(200, 200);
  t->align_ = TextAlign_CenterMiddle;
  REQUIRE(t->render(comp.get(), &target, 0));
  const float w = (float)t->img_->width, h = (float)t->img_->height;
  CHECK(t->align_origin_offset()[0] == doctest::Approx(0.f));
  CHECK(t->align_origin_offset()[1] == doctest::Approx(0.f));

  t->align_ = TextAlign_LeftTop;
  t->render(comp.get(), &target, 0);
  CHECK(t->align_origin_offset()[0] == doctest::Approx(-w / 2));
  CHECK(t->align_origin_offset()[1] == doctest::Approx(-h / 2));

  t->align_ = TextAlign_RightBottom;
  t->render(comp.get(), &target, 0);
  CHECK(t->align_origin_offset()[0] == doctest::Approx(w / 2));
  CHECK(t->align_origin_offset()[1] == doctest::Approx(h / 2));
}

TEST_CASE("TextEntt: 文字装飾の余白は揃えの基準(文字ブロック)に含まれない") {
  if(!fs_exists(bundled_font())) return;
  auto comp = cutil::make_ref<Composition>("t", 200, 200, 30);
  auto t    = TextEntt::Create("ABCD", bundled_font().c_str());
  Image target(200, 200);
  t->render(comp.get(), &target, 0);
  const int plain_w = t->img_->width;
  t->deco_          = TextDeco_Outline;
  t->align_         = TextAlign_LeftMiddle;
  t->render(comp.get(), &target, 0);
  CHECK((int)t->img_->width > plain_w);
  const float block_w = (float)t->img_->width - 2 * std::max(1, t->font_size_ / 12);
  CHECK(t->align_origin_offset()[0] == doctest::Approx(-block_w / 2));
}
