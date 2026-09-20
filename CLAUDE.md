# movutl 開発ルール

## 動作検証をする際は:

- ビルドしたい時はcmakeコマンドを叩かずに, `just build`を使用せよ。
- movutl_mainを実行したい時は, `just run`を使用せよ。`just run ./examples/xxx.lua`と引数を追加することで.luaを読み込んだりプロジェクトファイルを開くことができる。


## 実装が終わったら(コミット・push・PR作成の前に必ず実行)
```
just autogen
```
- `just autogen`: pygenでLua/C++バインディングとプロパティ生成物(`movutl/generated/`、`lancher/runtime/`)を再生成し、続けてclang-formatで全ファイルを整形する(整形まで含む)。ヘッダの `MPROPERTY` / `MUFUNC_AUTOGEN` / `app.hpp` 等の公開関数を変えたときは必須。
- 生成物は整形後の状態がコミット対象(CIが `pygen` → 整形 → `git diff --exit-code` で一致を確認する)。
- 実行後に `git status` で生成物・整形の差分を確認し、意図した変更と一緒にコミットする。
- 最終確認は `just check`(フォーマット確認・ビルド・テスト)。

## UIを変更したとき
GUIを実際に起動してスクリーンショットで確認する(`.claude/skills/ui-screenshot/SKILL.md` 参照)。
