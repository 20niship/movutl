-- UIのスクリーンショットを撮って終了する。build/ から実行する:
--   MU_CAPTURE_OUT=/tmp/ui MU_CAPTURE_EXO=path/to/x.exo ./movutl_main ../examples/ui_capture.lua
-- MU_CAPTURE_OUT: 出力ディレクトリ(既定 /tmp/movutl_ui)。MU_CAPTURE_EXO: 読み込むexo(省略時は何も読まない)
-- MU_CAPTURE_FRAME: 再生ヘッドを移動するフレーム(省略可)
local out_dir = os.getenv("MU_CAPTURE_OUT") or "/tmp/movutl_ui"
local exo = os.getenv("MU_CAPTURE_EXO")
os.execute("mkdir -p '" .. out_dir .. "'")

if exo and exo ~= "" then movutl.open_file(exo) end
local f = tonumber(os.getenv("MU_CAPTURE_FRAME") or "")
if f then movutl.goto_frame(f) end

-- 各ステップ: 待機フレーム数(描画が安定するまで)と、その時点で行う操作、撮影ファイル名
-- 撮影はGL_FRONTを読むため、操作の数フレーム後に行う
local steps = {
  { wait = 30, name = "01_initial" },
  { wait = 10, name = "02_select", act = function()
      -- MU_CAPTURE_SELECT: 選択するEntityの通し番号(レイヤー順、0始まり。既定0)
      local i = tonumber(os.getenv("MU_CAPTURE_SELECT") or "0")
      print("ui_capture: select_entt_by_index(" .. i .. ") = " .. tostring(movutl.select_entt_by_index(i)))
      -- MU_CAPTURE_FILTER: 選択中Entityに追加するフィルタ名(エフェクト/キーフレームUIの確認用)
      local fx = os.getenv("MU_CAPTURE_FILTER")
      if fx and fx ~= "" then
        print("ui_capture: add_filter(" .. fx .. ") = " .. tostring(movutl.add_filter_to_selected_entt(fx)))
      end
    end },
  { wait = 10, name = "03_after_select" },
}

local idx, waited = 1, 0
-- 毎フレーム呼ばれるフック(ウィンドウは作られない)
local function tick()
  local s = steps[idx]
  if not s then
    print("ui_capture: done -> " .. out_dir)
    os.exit(0)
  end
  if waited == 0 and s.act then local ok, err = pcall(s.act); if not ok then print("ui_capture: act error: " .. tostring(err)) end end
  waited = waited + 1
  if waited >= s.wait then
    local path = out_dir .. "/" .. s.name .. ".png"
    print("ui_capture: " .. path .. " " .. tostring(movutl.export_screen_png(path)))
    idx, waited = idx + 1, 0
  end
end
movutl.register_frame_hook(tick)
