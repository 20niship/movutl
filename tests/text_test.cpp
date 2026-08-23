#include <doctest/doctest.h>
#include <fstream>
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
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", 16, 0, 0, "/nonexistent/font.ttf"));
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", 16, 0, 0, "not_a_font.txt"));
  }

  SUBCASE("フォントとして不正な内容のファイルでもクラッシュせず false を返す") {
    const std::string dummy = "/tmp/opencode/movutl_test_not_a_font.ttf";
    {
      std::ofstream ofs(dummy);
      ofs << "this is not a font";
    }
    auto img = cutil::make_ref<Image>();
    CHECK_FALSE(detail::FontRenderManager::renderText(img.get(), "test", 16, 0, 0, dummy.c_str()));
  }

  SUBCASE("空文字列・nullチェック") {
    auto img = cutil::make_ref<Image>();
    CHECK_FALSE(detail::FontRenderManager::renderText(nullptr, "test", 16, 0, 0, ""));
  }

  SUBCASE("バンドルフォントで日本語テキストを描画できる") {
    const std::string meiryo = fs_get_font_path() + "/Meiryo.ttf";
    if(!fs_exists(meiryo)) return; // フォントがない環境はスキップ
    auto img = cutil::make_ref<Image>();
    CHECK(detail::FontRenderManager::renderText(img.get(), "こんにちは 世界", 16, 0, 0, meiryo.c_str()));
    CHECK_FALSE(img->empty());
    CHECK(img->width > 0);
    CHECK(img->height > 0);
  }
}
