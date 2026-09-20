# 外部のAviUtl Scriptの導入

-  https://bowlroll.net/file/3777 さつき氏
- https://tim3.web.fc2.com/sidx.htm から大量にダウンロード
- 

## 一括ダウンロード

tim3 のスクリプト置き場の zip を全て取得し、`plugins/scripts/aviutl/<zip名>/` に解凍する。

```
python3 scripts/download_aviutl_scripts.py
```

- 標準ライブラリのみ(追加インストール不要)、Python 3.9+
- 取得に失敗した zip があると終了コード 1
- bowlroll の script_20160828.zip はJSダウンロードのため対象外(手動で配置)
