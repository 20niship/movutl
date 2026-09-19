-- 12個のオブジェクト(図形/画像/テキスト)を左端に縦並びで置き、frame 0→100で右端へ移動させる。
-- オブジェクトごとにイージングを変えて比較する。X位置に加え、色(図形色/文字色/枠線色)や画像の透明度も変化させる。
local W, H, FPS = 1920, 1080, 30
local FSTART, FEND = 0, 100
local X_LEFT, X_RIGHT = 120, W - 120
local ROW_H = H / 13

movutl.Project.New(W, H, FPS)

local E = movutl.AniInterpType
-- {ラベル, イージング}: 直線を基準に代表的なイージングを12種
local eases = {
  {"Linear", E.LINEAR},
  {"InQuad", E.EaseInQuad},
  {"OutQuad", E.EaseOutQuad},
  {"InOutCubic", E.EaseInOutCubic},
  {"InSine", E.EaseInSine},
  {"OutExpo", E.EaseOutExpo},
  {"InBack", E.EaseInBack},
  {"OutBack", E.EaseOutBack},
  {"InOutElastic", E.EaseInOutElastic},
  {"OutBounce", E.EaseOutBounce},
  {"InOutCirc", E.EaseInOutCirc},
  {"OutQuint", E.EaseOutQuint},
}

math.randomseed(12345) -- 色を再現可能にする
local function rand_color()
  return {math.random(40, 255), math.random(40, 255), math.random(40, 255), 255}
end

local images = {"../assets/images/texture.jpg", "../assets/images/folder.jpg", "../assets/images/blender_png.png"}

-- kind: 順に 円・画像・テキストを巡回させる
for i, spec in ipairs(eases) do
  local label, ease = spec[1], spec[2]
  local y = ROW_H * i
  local kind = (i - 1) % 3
  local e
  if kind == 0 then
    e = movutl.add_new_shape_track("circle_" .. label, FSTART, FEND, movutl.ShapeType.ShapeType_Circle)
    e.size_ = movutl.Vec2(70, 70)
    e.border_width_ = 4
    movutl.add_keyframe(e, "color_", 0, {255, 0, 0, 255}) -- 赤→ランダム色
    movutl.add_keyframe(e, "color_", FEND, rand_color())
    movutl.add_keyframe(e, "border_color_", 0, {0, 0, 0, 255})
    movutl.add_keyframe(e, "border_color_", FEND, rand_color())
    movutl.set_keyframe_ease(e, "color_", 0, ease)
    movutl.set_keyframe_ease(e, "border_color_", 0, ease)
  elseif kind == 1 then
    e = movutl.add_new_image_track("image_" .. label, images[math.floor(i / 3) % #images + 1], FSTART, FEND)
    e.scale = movutl.Vec2(0.15, 0.15)
    movutl.add_keyframe(e, "alpha", 0, 0.2)
    movutl.add_keyframe(e, "alpha", FEND, 1.0)
    movutl.set_keyframe_ease(e, "alpha", 0, ease)
  else
    e = movutl.add_new_text_track("text_" .. label, FSTART, FEND)
    e.text = label
    movutl.add_keyframe(e, "color_", 0, {0, 128, 255, 255}) -- 青→ランダム色
    movutl.add_keyframe(e, "color_", FEND, rand_color())
    movutl.set_keyframe_ease(e, "color_", 0, ease)
  end

  local pos_prop = (kind == 1) and "pos" or "pos_"
  movutl.add_keyframe(e, pos_prop, 0, {X_LEFT, y, 0})
  movutl.add_keyframe(e, pos_prop, FEND, {X_RIGHT, y, 0})
  movutl.set_keyframe_ease(e, pos_prop, 0, ease)
end

print("easing_test: 12 objects created (circle/image/text x4), frame 0 -> " .. FEND)
movutl.play()
