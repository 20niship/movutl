local icons = require("icons_fontawesome6")

local filter_text = ""

local BTN_MIN_W = 96 -- ボタン最小幅(px)。パネル幅に応じて列数を決め、余りを均等に割り振る
local BTN_SPACING = 8

-- 「ツール」ウィンドウ本体とShift+Aポップアップの両方から呼ばれるオブジェクト追加ボタン群。何か1つでも確定したらtrueを返す
local function add_entities_ui()
  local confirmed = false
  local avail = imgui.GetContentRegionAvail().x
  local cols = math.max(1, math.floor((avail + BTN_SPACING) / (BTN_MIN_W + BTN_SPACING)))
  local btn_w = (avail - (cols - 1) * BTN_SPACING) / cols
  local n = 0

  -- グループ見出し。次のボタンは新しい行から始める
  local function group(title)
    if n > 0 then imgui.Spacing() end
    imgui.SeparatorText(title)
    n = 0
  end

  -- 幅を揃えたボタンを描画し、押されたらtrueを返す。ホバー時に正式名称をツールチップで出す
  local function tile(icon, label, tip)
    if n % cols ~= 0 then imgui.SameLine(0, BTN_SPACING) end
    n = n + 1
    local clicked = imgui.Button(icon .. " " .. label, imgui.ImVec2(btn_w, 0))
    if imgui.IsItemHovered(0) and imgui.BeginTooltip() then
      imgui.Text(tip or label)
      imgui.EndTooltip()
    end
    return clicked
  end

  local function add(icon, label, tip, name, type)
    if tile(icon, label, tip) then
      movutl.add_new_track(name, type, 0, 100)
      confirmed = true
    end
  end

  group("メディア")
  add(icons.ICON_FA_VIDEO, "動画", "動画を追加", "movie", movutl.EntityType.EntityType_Movie)
  add(icons.ICON_FA_IMAGE, "画像", "画像を追加", "image", movutl.EntityType.EntityType_Image)
  add(icons.ICON_FA_MUSIC, "音声", "音声を追加", "sound", movutl.EntityType.EntityType_Audio)
  if movutl.MidiEntt then -- `just build --daw`でビルドした時のみMIDIが有効
    add(icons.ICON_FA_KEYBOARD, "MIDI", "MIDIを追加", "midi", movutl.EntityType.EntityType_Midi)
  end

  group("テキスト・図形")
  add(icons.ICON_FA_FONT, "テキスト", "テキストを追加", "text", movutl.EntityType.EntityType_3DText)
  if tile(icons.ICON_FA_DRAW_POLYGON, "図形", "図形を追加(種類を選択)") then imgui.OpenPopup("##ADD_SHAPE_POPUP", 0) end
  if imgui.BeginPopup("##ADD_SHAPE_POPUP", 0) then
    local shapes = {
      { "三角形", movutl.ShapeType.ShapeType_Triangle },
      { "四角形", movutl.ShapeType.ShapeType_Rect },
      { "六角形", movutl.ShapeType.ShapeType_Hexagon },
      { "円", movutl.ShapeType.ShapeType_Circle },
      { "カスタムパス", movutl.ShapeType.ShapeType_Custom },
    }
    for _, sh in ipairs(shapes) do
      if imgui.Selectable(sh[1], false, 0, imgui.ImVec2(0, 0)) then
        movutl.add_new_shape_track("shape", 0, 100, sh[2])
        confirmed = true
      end
    end
    imgui.EndPopup()
  end

  group("制御")
  add(icons.ICON_FA_LAYER_GROUP, "グループ", "グループ制御を追加", "group", movutl.EntityType.EntityType_Group)
  add(icons.ICON_FA_VIDEO, "カメラ", "カメラ制御を追加", "camera", movutl.EntityType.EntityType_Camera)
  add(icons.ICON_FA_SHUFFLE, "シーンチェンジ", "シーンチェンジを追加", "scene change", movutl.EntityType.EntityType_SceneChange)
  add(icons.ICON_FA_TV, "バッファ", "フレームバッファを追加", "framebuffer", movutl.EntityType.EntityType_Framebuffer)
  if tile(icons.ICON_FA_MAGNIFYING_GLASS, "カスタム", "カスタムオブジェクトを追加(検索して選択)") then imgui.OpenPopup("##ADD_CUSTOM_OBJECT_POPUP", 0) end
  if imgui.BeginPopup("##ADD_CUSTOM_OBJECT_POPUP", 0) then
    local _, new_text = imgui.InputText("##custom_obj_filter", filter_text, 0)
    filter_text = new_text
    for i, name in ipairs(movutl.list_custom_objects()) do
      if filter_text == "" or string.find(name, filter_text, 1, true) then
        -- 同名オブジェクトでIDが衝突しないよう連番を付ける(PushIDはLuaから1引数で呼べない)
        if imgui.Selectable(name .. "##" .. i, false, 0, imgui.ImVec2(0, 0)) then
          movutl.add_new_custom_object_track(name, 0, 100)
          confirmed = true
        end
      end
    end
    imgui.EndPopup()
  end

  group("参照")
  add(icons.ICON_FA_CLONE, "コンポ", "コンポ参照を追加", "compo ref", movutl.EntityType.EntityType_Scene)
  add(icons.ICON_FA_VOLUME_HIGH, "コンポ音声", "コンポ音声参照を追加", "compo audio ref", movutl.EntityType.EntityType_SceneAudio)

  return confirmed
end

movutl.register_window("ツール", {
  update = function(self) add_entities_ui() end,
})

-- 確定時はCloseCurrentPopupで明示的に閉じる。Esc/外側クリックで閉じた場合もBeginPopupがfalseになるためどちらもnilで終了する
movutl.register_command("add_object_menu", "オブジェクト追加メニュー", "オブジェクト追加メニューを表示する", "shift+a", {
  on_start = function(self)
    imgui.OpenPopup("##add_object_shortcut_menu", 0)
    return "running"
  end,
  tick = function(self)
    imgui.SetNextWindowSize(imgui.ImVec2(300, 200), 0)
    if not imgui.BeginPopup("##add_object_shortcut_menu", 0) then return nil end
    local confirmed = add_entities_ui()
    if confirmed then imgui.CloseCurrentPopup() end
    imgui.EndPopup()
    if confirmed then return nil end
    return "running"
  end,
})
