-- 6x4グリッドに図形を並べ、実装済み映像フィルタ42種を(重ねがけも使い)セルごとに検証しPNG書き出しする
local W, H = 1920, 1080
local COLS, ROWS = 6, 4
local CELL_W, CELL_H = W / COLS, H / ROWS

movutl.Project.New(W, H, 30)

local shape_types = {
  movutl.ShapeType.ShapeType_Triangle,
  movutl.ShapeType.ShapeType_Rect,
  movutl.ShapeType.ShapeType_Hexagon,
  movutl.ShapeType.ShapeType_Circle,
}
local colors = {
  movutl.Vec4b(220, 80, 80, 255),
  movutl.Vec4b(80, 200, 120, 255),
  movutl.Vec4b(80, 140, 220, 255),
  movutl.Vec4b(230, 200, 60, 255),
}

-- params={フィルタ名,パラメータ名,値}: デフォルトのままだと効果が見えないフィルタだけ明示的に値を与える
local cell_specs = {
  {filters = {"色調補正"}, label = "color_correction+rot", rot = 30, params = {{"色調補正", "hue", 90}}},
  {filters = {"単色化"}, label = "single_color+scale", scale = 1.5},
  {filters = {"色ずらし", "フィルムグレイン"}, label = "color_shift+grain", params = {{"色ずらし", "shift_x", 15}}},
  {filters = {"グラデーション", "斜めクリッピング"}, label = "gradient+diag_clip"},
  {filters = {"拡張色調補正", "カラーバランス"}, label = "extend_color+balance", params = {{"拡張色調補正", "r", 100}, {"カラーバランス", "highlight_b", 150}}},
  {filters = {"ぼかし", "リサイズ"}, label = "blur+resize", params = {{"ぼかし", "range", 15}}},
  {filters = {"方向ぼかし", "走査線"}, label = "dir_blur+scanline", params = {{"方向ぼかし", "range", 20}}},
  {filters = {"放射ぼかし"}, label = "radial_blur+scale", scale = 1.2},
  {filters = {"クロマキー"}, label = "chroma_key", params = {{"クロマキー", "threshold", 260}}},
  {filters = {"ルミナンスキー", "反転"}, label = "luminance_key+invert"},
  {filters = {"発光"}, label = "glow", params = {{"発光", "threshold", 50}, {"発光", "intensity", 300}}},
  {filters = {"グロー", "エンボス"}, label = "bloom+emboss"},
  {filters = {"縁取り"}, label = "outline+rot", rot = 15, params = {{"縁取り", "width", 12}}},
  {filters = {"クリッピング&リサイズ"}, label = "clipping"},
  {filters = {"モノクロ", "ポスタリゼーション"}, label = "grayscale+posterize", params = {{"ポスタリゼーション", "levels", 3}}},
  {filters = {"セピア", "ハーフトーン"}, label = "sepia+halftone"},
  {filters = {"ノイズ除去", "シャープ"}, label = "denoise+sharpen", params = {{"シャープ", "strength", 400}}},
  {filters = {"エッジ抽出", "2値化"}, label = "edge+binarize"},
  {filters = {"モザイク", "レンズ歪み"}, label = "mosaic+lens_dist", params = {{"レンズ歪み", "strength", 80}}},
  {filters = {"波紋"}, label = "ripple", params = {{"波紋", "amplitude", 40}}},
  {filters = {"揺らぎ"}, label = "wave_dist", params = {{"揺らぎ", "amplitude", 25}}},
  {filters = {"万華鏡"}, label = "kaleidoscope", params = {{"万華鏡", "segments", 8}}},
  {filters = {"4色グラデーション", "円形クリッピング"}, label = "4color_grad+circle_clip"},
  {filters = {"放射グラデーション", "カラーLUT", "ソフトフォーカス", "インターレースシフト", "VHSノイズ", "ビネット"}, label = "radial_grad+etc"},
}

for idx, spec in ipairs(cell_specs) do
  local i = idx - 1
  local col, row = i % COLS, math.floor(i / COLS)
  local cx, cy = col * CELL_W, row * CELL_H

  local shp = movutl.add_new_shape_track(string.format("cell_%02d", i), 0, 10, shape_types[i % 4 + 1])
  shp.pos_ = movutl.Vec3(cx + (CELL_W - 180) / 2, cy + (CELL_H - 200) / 2, 0)
  shp.size_ = movutl.Vec2(180, 130)
  shp.color_ = colors[i % 4 + 1]
  if spec.rot then shp.rot_ = spec.rot * math.pi / 180 end
  if spec.scale then shp.size_ = movutl.Vec2(180 * spec.scale, 130 * spec.scale) end
  for _, fname in ipairs(spec.filters) do
    if not movutl.add_filter_to_shape(shp, fname) then
      print("WARN: filter not found: " .. fname)
    end
  end
  if spec.params then
    for _, prm in ipairs(spec.params) do
      movutl.set_shape_filter_param(shp, prm[1], prm[2], prm[3])
    end
  end

  local txt = movutl.add_new_text_track(string.format("label_%02d", i), 0, 10)
  txt.text = spec.label
  txt.scale_x_ = 0.35
  txt.scale_y_ = 0.35
  txt.pos_ = movutl.Vec3(cx + 8, cy + 10, 0)
  txt.color_ = movutl.Vec4b(255, 255, 255, 255)
end

print("filter_grid_test: " .. #cell_specs .. " cells created (" .. COLS .. "x" .. ROWS .. ")")

local out_path = "../filter_grid_output.png"
if movutl.export_current_frame_png(out_path) then
  print("wrote: " .. out_path)
else
  print("ERROR: failed to export png")
end

os.exit(0)
