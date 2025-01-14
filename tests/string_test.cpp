#include <doctest/doctest.h>
#include <movutl/core/string.hpp>

using namespace mu;
using CString = CStr;

TEST_CASE("CString 基本機能のテスト") {
  SUBCASE("デフォルトコンストラクタ") {
    CString str;
    CHECK(str.size() == 0);
    CHECK(str.c_str() == nullptr);
  }

  SUBCASE("コンストラクタ (const char*)") {
    CString str("Hello");
    CHECK(str.size() == 5);
    CHECK(strcmp(str.c_str(), "Hello") == 0);
  }

  SUBCASE("コピーコンストラクタ") {
    CString str1("Hello");
    CString str2(str1);
    CHECK(str2.size() == 5);
    CHECK(strcmp(str2.c_str(), "Hello") == 0);
    CHECK(str1 == str2);
  }

  SUBCASE("ムーブコンストラクタ") {
    CString str1("Hello");
    CString str2(std::move(str1));
    CHECK(str2.size() == 5);
    CHECK(strcmp(str2.c_str(), "Hello") == 0);
    CHECK(str1.c_str() == nullptr); // str1 はムーブ後に nullptr
  }

  SUBCASE("コピー代入演算子") {
    CString str1("Hello");
    CString str2("World");
    str2 = str1;
    CHECK(str2.size() == 5);
    CHECK(strcmp(str2.c_str(), "Hello") == 0);
    CHECK(str1 == str2);
  }

  SUBCASE("ムーブ代入演算子") {
    CString str1("Hello");
    CString str2("World");
    str2 = std::move(str1);
    CHECK(str2.size() == 5);
    CHECK(strcmp(str2.c_str(), "Hello") == 0);
    CHECK(str1.c_str() == nullptr); // str1 はムーブ後に nullptr
  }
}

TEST_CASE("CString 操作のテスト") {
  SUBCASE("length() / size()") {
    CString str("Hello");
    CHECK(str.length() == 5);
    CHECK(str.size() == 5);
  }

  SUBCASE("添字アクセス") {
    CString str("Hello");
    CHECK(str[0] == 'H');
    CHECK(str[4] == 'o');
    CHECK_THROWS_AS(str[5], std::out_of_range); // 範囲外アクセス
  }

  SUBCASE("substr") {
    CString str("Hello, World!");
    CHECK(str.substr(0, 5) == CString("Hello"));
    CHECK(str.substr(7, 5) == CString("World"));
    CHECK_THROWS_AS(str.substr(20, 5), std::out_of_range); // 範囲外アクセス
  }

  SUBCASE("find") {
    CString str("Hello, World!");
    CHECK(str.find("Hello") == 0);
    CHECK(str.find("World") == 7);
    CHECK(str.find("!") == 12);
    CHECK(str.find("NotFound") == std::string::npos);
  }

  SUBCASE("contains") {
    CString str("Hello, World!");
    CHECK(str.contains("Hello") == true);
    CHECK(str.contains("World") == true);
    CHECK(str.contains("NotFound") == false);
  }

  SUBCASE("split") {
    CString str("a,b,c");
    auto parts = str.split(',');
    CHECK(parts.size() == 3);
    CHECK(parts[0] == CString("a"));
    CHECK(parts[1] == CString("b"));
    CHECK(parts[2] == CString("c"));

    CString str2("single");
    auto parts2 = str2.split(',');
    CHECK(parts2.size() == 1);
    CHECK(parts2[0] == CString("single"));
  }

  SUBCASE("連結演算子 +") {
    CString str1("Hello");
    CString str2(", World!");
    CString result = str1 + str2;
    CHECK(result == CString("Hello, World!"));
    CHECK(str1 == CString("Hello")); // 元の文字列は変更されない
    CHECK(str2 == CString(", World!"));
  }

  SUBCASE("連結演算子 +=") {
    CString str1("Hello");
    CString str2(", World!");
    str1 += str2;
    CHECK(str1 == CString("Hello, World!"));
  }

  SUBCASE("同一判定") {
    CString str1("Test");
    CString str2("Test");
    CString str3("Different");
    CHECK(str1 == str2);
    CHECK(str1 != str3);
  }
}

TEST_CASE("CString エッジケースのテスト") {
  SUBCASE("空文字列の操作") {
    CString str1;
    CString str2("");
    CHECK(str1.size() == 0);
    CHECK(str2.size() == 0);
    CHECK(str1 == str2);
  }

  SUBCASE("連結で空文字列") {
    CString str1("Hello");
    CString str2("");
    CString result = str1 + str2;
    CHECK(result == CString("Hello"));
    CHECK(result.size() == 5);
  }

  SUBCASE("substr で範囲外") {
    CString str("Test");
    CHECK_THROWS_AS(str.substr(10, 5), std::out_of_range);
  }

  SUBCASE("find で見つからない文字列") {
    CString str("Example");
    CHECK(str.find("NotFound") == std::string::npos);
  }
}
