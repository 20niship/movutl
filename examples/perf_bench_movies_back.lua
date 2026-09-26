-- 動画を後方レイヤーに配置した速度計測(先読みタスクへのヘッドスタート検証用)。build/run_lua_headless --renderer=cpu ../examples/perf_bench_movies_back.lua
local W, H, FEND = 1920, 1080, 60
movutl.Project.New(W, H, 30)

local shape_types = {
  movutl.ShapeType.ShapeType_Triangle, movutl.ShapeType.ShapeType_Rect,
  movutl.ShapeType.ShapeType_Hexagon, movutl.ShapeType.ShapeType_Circle,
}
local function pos(i)
  return movutl.Vec3(((i * 197) % 1600) - 800, ((i * 113) % 900) - 450, 0)
end

for i = 1, 30 do
  local e = movutl.add_new_shape_track("shape_" .. i, 0, FEND, shape_types[i % 4 + 1])
  e.pos_ = pos(i)
  e.size_ = movutl.Vec2(150, 150)
  e.color_ = movutl.Vec4b(60 + i * 7 % 195, 200 - i * 5 % 150, 80 + i * 11 % 175, 220)
  e.rotation_ = i * 23 % 360
end

local dl = os.getenv("HOME") .. "/Downloads/"
local movies = {
  "Untitled.mp4", "melchior-pv3-107.mp4", "VコンテLo_melchior_0014.mp4",
  "taisei_melchior_pv_ver07 (1).mp4", "taisei_melchior_pv_ver07.mp4", "souko_infrastructure/test_comp0001-0069.mkv",
}
for i, m in ipairs(movies) do
  local v = movutl.add_new_video_track("movie_" .. i, dl .. m, 0, 30 + i - 1)
  if v then
    v.scale_ = 50
    v.pos_ = movutl.Vec3(((i - 1) % 3 - 1) * 640, (math.floor((i - 1) / 3) - 0.5) * 360, 0)
  else
    print("WARN: video load failed: " .. m)
  end
end

local comp = movutl.Composition.GetActiveComp()
local N = 10
local function bench(from)
  local t0 = os.clock()
  for f = from, from + N - 1 do
    comp:set_frame(f)
    comp:invalidate_cache_all()
    comp:render_current_frame_main_thread()
  end
  return (os.clock() - t0) / N * 1000
end
bench(0)
local ms = bench(20)
print(string.format("perf_bench_movies_back: 30shapes+6movies %dx%d: %.1f ms/frame single-thread (%.1f fps)", W, H, ms, 1000 / ms))
os.exit(0)
