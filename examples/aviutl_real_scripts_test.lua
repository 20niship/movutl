-- 実際に配布されているAviUtl拡張編集Luaスクリプト(plugins/scripts/aviutl_real_*.anm、起動時に自動登録される。softfocus=CaffemochaY氏MIT Licenseを無改変使用、auto_circle=定番パターンを参考にした自作)をmovutlで動かす検証
movutl.Project.New(240, 240, 30)

local photo = movutl.add_new_image_track("photo", "../assets/textures/warning.jpeg", 0, 10)
photo.pos = movutl.Vec3(10 - photo.width / 2, 60 - photo.height / 2, 0)
print("add softfocus filter:", movutl.add_filter_to_image(photo, "aviutl_real_softfocus"))

local dot = movutl.add_new_image_track("dot", "../assets/textures/warning.jpeg", 0, 30)
dot.pos = movutl.Vec3(170 - dot.width / 2, 120 - dot.height / 2, 0)
print("add auto_circle filter:", movutl.add_filter_to_image(dot, "円運動"))
movutl.set_image_filter_param(dot, "円運動", "半径", 40)
movutl.set_image_filter_param(dot, "円運動", "速度", 30)

local comp = movutl.Composition.GetActiveComp()
comp:set_frame(3) -- deg=90度(円軌道の右端)で書き出す

local out_path = "../aviutl_real_scripts_output.png"
if movutl.export_current_frame_png(out_path) then
  print("wrote: " .. out_path)
else
  print("ERROR: failed to export png")
end
os.exit(0)
