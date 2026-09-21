-- パフォーマンス計測: 1920x1080に画像/図形/テキストを計~54トラック重ね、位置・拡大率・回転・色調補正等のフィルタを付けて描画時間を測る
-- GUI: just run ./examples/perf_bench.lua(Tracy接続時はzone別の内訳も見られる) / headless計測: build/run_lua_headless ../examples/perf_bench.lua
local W, H, FEND = 1920, 1080, 60
movutl.Project.New(W, H, 30)

local images = {
  "../assets/textures/tile.jpg", "../assets/textures/concrete.png",
  "../assets/textures/concrete1.png", "../assets/textures/wood.png",
}
local shape_types = {
  movutl.ShapeType.ShapeType_Triangle, movutl.ShapeType.ShapeType_Rect,
  movutl.ShapeType.ShapeType_Hexagon, movutl.ShapeType.ShapeType_Circle,
}
local function pos(i) -- 画面全体に散らす(中心原点)
  return movutl.Vec3(((i * 197) % 1600) - 800, ((i * 113) % 900) - 450, 0)
end

-- 動画 x6 (Downloads内の1920x1080素材。3x2に50%で並べる。最下層のlayer 0-5)
local dl = os.getenv("HOME") .. "/Downloads/"
local movies = {
  "Untitled.mp4", "melchior-pv3-107.mp4", "VコンテLo_melchior_0014.mp4",
  "taisei_melchior_pv_ver07 (1).mp4", "taisei_melchior_pv_ver07.mp4", "souko_infrastructure/test_comp0001-0069.mkv",
}
for i, m in ipairs(movies) do
  local v = movutl.add_new_video_track("movie_" .. i, dl .. m, 0, i - 1)
  if v then
    v.scale_ = 50
    v.pos_ = movutl.Vec3(((i - 1) % 3 - 1) * 640, (math.floor((i - 1) / 3) - 0.5) * 360, 0)
  else
    print("WARN: video load failed: " .. m)
  end
end

-- 画像 x16: 位置/拡大率/回転 + 色調補正 + ぼかし(4個に1個)
for i = 1, 16 do
  local e = movutl.add_new_image_track("img_" .. i, images[(i - 1) % #images + 1], 0, FEND)
  e.scale_ = 40 + (i % 5) * 20
  e.pos_ = pos(i)
  e.rotation_ = i * 17 % 360
  movutl.add_filter_to_image(e, "色調補正")
  movutl.set_image_filter_param(e, "色調補正", "hue", i * 20 % 360)
  if i % 4 == 0 then
    movutl.add_filter_to_image(e, "ぼかし")
    movutl.set_image_filter_param(e, "ぼかし", "range", 8)
  end
end

-- 図形 x24: 位置/サイズ/回転 + 色調補正 or 縁取り
for i = 1, 24 do
  local e = movutl.add_new_shape_track("shape_" .. i, 0, FEND, shape_types[i % 4 + 1])
  e.pos_ = pos(i + 20)
  e.size_ = movutl.Vec2(150 + (i % 5) * 60, 150 + (i % 3) * 60)
  e.color_ = movutl.Vec4b(60 + i * 7 % 195, 200 - i * 5 % 150, 80 + i * 11 % 175, 220)
  e.rotation_ = i * 23 % 360
  movutl.add_filter_to_shape(e, i % 2 == 0 and "色調補正" or "縁取り")
end

-- テキスト x8
for i = 1, 8 do
  local t = movutl.add_new_text_track("text_" .. i, 0, FEND)
  t.text = "PERF TEST " .. i
  t.font_size_ = 80
  t.pos_ = pos(i + 50)
  t.rotation_ = i * 9
end

local comp = movutl.Composition.GetActiveComp()

-- headless(run_lua_headless。GUI用のregister_frame_hookが無い): 単一スレッドで各フレームをキャッシュ無しの同期描画にかかる時間を測って終了(before/after比較用)
if not movutl.register_frame_hook then
  local N = 10
  local function bench(from)
    local t0 = os.clock() -- 単一スレッド描画なので実時間とほぼ一致
    for f = from, from + N - 1 do
      comp:set_frame(f)
      comp:invalidate_cache_all()
      comp:render_current_frame_main_thread()
    end
    return (os.clock() - t0) / N * 1000
  end
  bench(0) -- ウォームアップ(動画のオープン/初回デコード等)
  local ms = bench(20)
  print(string.format("perf_bench: 54 tracks %dx%d: %.1f ms/frame single-thread (%.1f fps)", W, H, ms, 1000 / ms))
  os.exit(0)
end

-- GUI(movutl_main): perf_test.luaと同様に再生してプレビューを表示し続ける。1周するごとに再生ヘッドの速さを出力する
-- (再生ヘッドは実時間で進むので、描画キャッシュが間に合っているかとは別。間に合っているかはタイムラインのキャッシュ表示で見る)
local target = comp.framerate
local t0, f0, last = nil, nil, nil
local WARMUP = 5 -- 最初の数frameは除く
movutl.goto_frame(0)
movutl.play()
movutl.register_frame_hook(function()
  local f = comp:get_frame()
  if f == last then return end
  if last and f < last and t0 then -- ループで巻き戻った=1周した
    local sec = imgui.GetTime() - t0
    print(string.format("perf_bench: 54 tracks %dx%d target %.0ffps: playhead %.1f fps (%.0f%% of realtime)", W, H, target, (last - f0) / sec, (last - f0) / sec / target * 100))
    t0 = nil
  end
  last = f
  if f == WARMUP and not t0 then t0, f0 = imgui.GetTime(), f end
end)
