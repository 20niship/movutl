# movutl 開発ルール

## 実装が終わったら(コミット・push・PR作成の前に必ず実行)
```
just autogen && just format
```
- `just autogen`: pygenでLua/C++バインディングとプロパティ生成物(`movutl/generated/`、`lancher/runtime/`)を再生成する。ヘッダの `MPROPERTY` / `MUFUNC_AUTOGEN` / `app.hpp` 等の公開関数を変えたときは必須。
- `just format`: clang-formatで全ファイルを整形する。生成物も整形後の状態がコミット対象(CIが `pygen` → 整形 → `git diff --exit-code` で一致を確認する)。
- 実行後に `git status` で生成物・整形の差分を確認し、意図した変更と一緒にコミットする。
- 最終確認は `just check`(フォーマット確認・ビルド・テスト)。

## UIを変更したとき
GUIを実際に起動してスクリーンショットで確認する(`.claude/skills/ui-screenshot/SKILL.md` 参照)。
