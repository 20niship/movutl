-- AviUtl互換アニメーション効果スクリプト(plugins/scripts/*.anm、起動時に自動登録される)を画像に適用してPNG書き出しする動作確認用スクリプト
local W, H = 64, 64
movutl.Project.New(W, H, 30)

local img = movutl.add_new_image_track("photo", "../assets/textures/warning.jpeg", 0, 10)
img.scale = movutl.Vec2(W / img.width, H / img.height)
img.pos = movutl.Vec3(W / 2 - img.width / 2, H / 2 - img.height / 2, 0)

local applied = movutl.add_filter_to_image(img, "AviUtlグレースケール")
print("add_filter_to_image:", applied)

local out_path = "../aviutl_script_output.png"
if movutl.export_current_frame_png(out_path) then
  print("wrote: " .. out_path)
else
  print("ERROR: failed to export png")
end
os.exit(0)
