local icons = require("icons_fontawesome6")

local filter_text = ""

-- 「ツール」ウィンドウ本体とShift+Aポップアップの両方から呼ばれるオブジェクト追加ボタン群
local function add_entities_ui()
  imgui.BeginGroup()
  imgui.Columns(2, "", true)

  if imgui.Button(icons.ICON_FA_FONT .. " テキスト", imgui.ImVec2(0, 0)) then movutl.add_new_track("text", movutl.EntityType_3DText, 0, 100) end
  imgui.NextColumn()
  if imgui.Button(icons.ICON_FA_IMAGE .. " 画像", imgui.ImVec2(0, 0)) then movutl.add_new_track("image", movutl.EntityType_Image, 0, 100) end
  imgui.NextColumn()
  if imgui.Button(icons.ICON_FA_VIDEO .. " 動画", imgui.ImVec2(0, 0)) then movutl.add_new_track("movie", movutl.EntityType_Movie, 0, 100) end
  imgui.NextColumn()
  if imgui.Button(icons.ICON_FA_MUSIC .. " 音声", imgui.ImVec2(0, 0)) then movutl.add_new_track("sound", movutl.EntityType_Audio, 0, 100) end
  imgui.NextColumn()
  if imgui.Button(icons.ICON_FA_KEYBOARD .. " MIDI", imgui.ImVec2(0, 0)) then movutl.add_new_track("midi", movutl.EntityType_Midi, 0, 100) end
  imgui.NextColumn()

  if imgui.Button(icons.ICON_FA_DRAW_POLYGON .. " 図形", imgui.ImVec2(0, 0)) then imgui.OpenPopup("##ADD_SHAPE_POPUP", 0) end
  if imgui.BeginPopup("##ADD_SHAPE_POPUP", 0) then
    if imgui.Selectable("三角形", false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_shape_track("shape", 0, 100, movutl.ShapeType_Triangle) end
    if imgui.Selectable("四角形", false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_shape_track("shape", 0, 100, movutl.ShapeType_Rect) end
    if imgui.Selectable("六角形", false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_shape_track("shape", 0, 100, movutl.ShapeType_Hexagon) end
    if imgui.Selectable("円", false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_shape_track("shape", 0, 100, movutl.ShapeType_Circle) end
    if imgui.Selectable("カスタムパス", false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_shape_track("shape", 0, 100, movutl.ShapeType_Custom) end
    imgui.EndPopup()
  end
  imgui.NextColumn()

  if imgui.Button(icons.ICON_FA_TV .. " フレームバッファ", imgui.ImVec2(0, 0)) then movutl.add_new_track("framebuffer", movutl.EntityType_Framebuffer, 0, 100) end
  imgui.NextColumn()

  if imgui.Button(icons.ICON_FA_MAGNIFYING_GLASS .. " カスタムオブジェクト", imgui.ImVec2(0, 0)) then imgui.OpenPopup("##ADD_CUSTOM_OBJECT_POPUP", 0) end
  if imgui.BeginPopup("##ADD_CUSTOM_OBJECT_POPUP", 0) then
    local _, new_text = imgui.InputText("##custom_obj_filter", filter_text, 0)
    filter_text = new_text
    for _, name in ipairs(movutl.list_custom_objects()) do
      if filter_text == "" or string.find(name, filter_text, 1, true) then
        if imgui.Selectable(name, false, 0, imgui.ImVec2(0, 0)) then movutl.add_new_custom_object_track(name, 0, 100) end
      end
    end
    imgui.EndPopup()
  end
  imgui.NextColumn()

  if imgui.Button(icons.ICON_FA_CLONE .. " コンポ参照", imgui.ImVec2(0, 0)) then movutl.add_new_track("compo ref", movutl.EntityType_Scene, 0, 100) end
  imgui.NextColumn()
  if imgui.Button(icons.ICON_FA_CLONE .. " コンポ音声参照", imgui.ImVec2(0, 0)) then movutl.add_new_track("compo audio ref", movutl.EntityType_SceneAudio, 0, 100) end
  imgui.NextColumn()

  imgui.Columns(1, "", true)
  imgui.EndGroup()
end

movutl.register_window("ツール", {
  update = function(self)
    add_entities_ui()
    if imgui.BeginPopup("##add_object_shortcut_menu", 0) then
      add_entities_ui()
      imgui.EndPopup()
    end
  end,
})

movutl.register_command("add_object_menu", "オブジェクト追加メニュー", "オブジェクト追加メニューを表示する", "shift+a", {
  on_start = function(self) imgui.OpenPopup("##add_object_shortcut_menu", 0) end,
})
