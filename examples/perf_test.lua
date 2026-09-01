local W, H, FPS = 1920, 1080, 30
local FSTART, FEND = 0, 150 -- 5秒分

movutl.Project.New(W, H, FPS)

local movie_paths = {
  "../assets/movies/big_buck_bunny_360_10s.mp4",
  "../assets/movies/sample-5s.mp4",
}

-- 動画トラック x10 (layer 0-9)
for i = 1, 1 do
  local path = movie_paths[(i - 1) % #movie_paths + 1]
  movutl.add_new_video_track("movie_" .. i, path, FSTART, i - 1)
end

-- テキストトラック x5 (layer 10-14 に自動配置)
for i = 1, 5 do
  movutl.add_new_track("text_" .. i, movutl.EntityType.EntityType_3DText, FSTART, FEND)
end

-- 図形トラック x5 (layer 15-19 に自動配置)
local shape_types = {
  movutl.ShapeType.ShapeType_Triangle,
  movutl.ShapeType.ShapeType_Rect,
  movutl.ShapeType.ShapeType_Hexagon,
  movutl.ShapeType.ShapeType_Circle,
  movutl.ShapeType.ShapeType_Custom,
}
for i = 1, 5 do
  local shp = movutl.add_new_shape_track("shape_" .. i, FSTART, FEND, shape_types[i])
  shp.pos_   = movutl.Vec3(100 + (i - 1) * 200, 800, 0)
  shp.size_  = movutl.Vec2(150, 150)
  shp.color_ = movutl.Vec4b(255, 255, 255, 255)
  if shp.shape_type_ == movutl.ShapeType.ShapeType_Custom then
    shp.custom_path = "0,0;150,0;220,80;110,150;-50,80"
  end
end

-- 音声トラック x3 (layer 11-13、開始位置を30フレームずつずらして重ねる)
local audio_path = "../assets/audio/file_example_WAV_1MG.wav"
movutl.add_new_audio_track("audio_1", audio_path, FSTART, 11)
movutl.add_new_audio_track("audio_2", audio_path, FSTART + 30, 12)
movutl.add_new_audio_track("audio_3", audio_path, FSTART + 60, 13)

print("perf_test: 14 layers created (movie x1, text x5, shape x5, audio x3)")

movutl.play()
