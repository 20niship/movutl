-- 合成のみ(動画デコード無し)の速度計測。build/run_lua_headless --renderer=cpu|vulkan ../examples/perf_bench_shapes.lua
local W, H, FEND = 1920, 1080, 60
movutl.Project.New(W, H, 30)

local shape_types = {
  movutl.ShapeType.ShapeType_Triangle, movutl.ShapeType.ShapeType_Rect,
  movutl.ShapeType.ShapeType_Hexagon, movutl.ShapeType.ShapeType_Circle,
}
local function pos(i)
  return movutl.Vec3(((i * 197) % 1600) - 800, ((i * 113) % 900) - 450, 0)
end

local N = 100
for i = 1, N do
  local e = movutl.add_new_shape_track("shape_" .. i, 0, FEND, shape_types[i % 4 + 1])
  e.pos_ = pos(i)
  e.size_ = movutl.Vec2(150 + (i % 5) * 60, 150 + (i % 3) * 60)
  e.color_ = movutl.Vec4b(60 + i * 7 % 195, 200 - i * 5 % 150, 80 + i * 11 % 175, 220)
  e.rotation_ = i * 23 % 360
end

local comp = movutl.Composition.GetActiveComp()

local WARMUP, MEASURE = 10, 30
local function bench(from, count)
  local t0 = os.clock()
  for f = from, from + count - 1 do
    comp:set_frame(f)
    comp:invalidate_cache_all()
    comp:render_current_frame_main_thread()
  end
  return (os.clock() - t0) / count * 1000
end
bench(0, WARMUP)
local ms = bench(WARMUP, MEASURE)
print(string.format("perf_bench_shapes: %d shapes %dx%d: %.2f ms/frame single-thread (%.1f fps)", N, W, H, ms, 1000 / ms))
os.exit(0)
