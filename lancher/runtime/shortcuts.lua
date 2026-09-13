movutl.register_command("save_project_cmd", "保存", "プロジェクトを保存する", "ctrl+s", {
  on_start = function(self)
    if movutl.has_project_path() then
      movutl.save_project()
    else
      local path = movutl.select_save_file_dialog("プロジェクトを保存", "project.json", {"json"})
      if path == "" then return "failed" end
      movutl.save_project_as(path)
    end
  end,
})

movutl.register_command("save_project_as_cmd", "名前を付けて保存", "プロジェクトを別名で保存する", "ctrl+shift+s", {
  on_start = function(self)
    local path = movutl.select_save_file_dialog("名前を付けて保存", "project.json", {"json"})
    if path == "" then return "failed" end
    movutl.save_project_as(path)
  end,
})

movutl.register_command("open_project_cmd", "開く", "プロジェクトファイルを開く", "ctrl+o", {
  on_start = function(self)
    local path = movutl.select_file_dialog("プロジェクトを開く", {"json"})
    if path == "" then return "failed" end
    movutl.open_project(path)
  end,
})

movutl.register_command("import_media_cmd", "ファイルを読み込む", "ファイルを選択し内容に応じたEntityを追加する", "ctrl+i", {
  on_start = function(self)
    local path = movutl.select_file_dialog("ファイルを読み込む", {})
    if path == "" then return "failed" end
    if movutl.import_media_file(path) == nil then return "failed" end
  end,
})
