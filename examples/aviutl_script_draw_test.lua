-- obj.draw/obj.drawpoly(座標変換系)のAviUtl互換スクリプトを画像に適用して動作確認する
local W, H = 300, 150
movutl.Project.New(W, H, 30)

print("load perspective:", movutl.load_aviutl_effect_script("../examples/aviutl_script_perspective.anm"))
print("load shake:", movutl.load_aviutl_effect_script("../examples/aviutl_script_shake.anm"))

local img1 = movutl.add_new_image_track("tile1", "../assets/textures/tile.jpg", 0, 10)
img1.scale = movutl.Vec2(100 / img1.width, 100 / img1.height)
img1.pos = movutl.Vec3(60 - img1.width / 2, 75 - img1.height / 2, 0)
print("add perspective filter:", movutl.add_filter_to_image(img1, "あおり変形"))
movutl.set_image_filter_param(img1, "あおり変形", "上辺の幅", 30)
movutl.set_image_filter_param(img1, "あおり変形", "下辺の幅", 0)

local img2 = movutl.add_new_image_track("tile2", "../assets/textures/tile.jpg", 0, 10)
img2.scale = movutl.Vec2(100 / img2.width, 100 / img2.height)
img2.pos = movutl.Vec3(200 - img2.width / 2, 75 - img2.height / 2, 0)
print("add shake filter:", movutl.add_filter_to_image(img2, "シェイク"))

local out_path = "../aviutl_script_draw_output.png"
if movutl.export_current_frame_png(out_path) then
  print("wrote: " .. out_path)
else
  print("ERROR: failed to export png")
end
os.exit(0)
