#pragma once

#ifdef __linux__

#include <cstdint>
#include <map>
#include <movutl/input/pen_input.hpp>
#include <optional>
#include <string>

struct _XDisplay;

namespace mu {

// Xサーバ無しで動く変換部分(テスト用に分離)
namespace xi2 {

// ペンデバイス1つ分の状態
struct PenDevice {
  bool eraser         = false;
  int pressure_axis   = -1; // "Abs Pressure"のvaluator番号
  double pressure_min = 0, pressure_max = 1;
  float pressure = 0; // 直近の筆圧(0〜1)。イベントには変化した軸しか入らないので覚えておく
  bool down      = false;
  PenEvent last{}; // 直近の入力(抜かれた時にUpを作る)
};

// XIDeviceEventのうち使う部分
struct DeviceInput {
  PenPhase phase;                      // ButtonPress=Down, Motion=Move, ButtonRelease=Up
  int button;                          // Down/Upのボタン番号
  double x, y;                         // ウィンドウ座標(サブピクセル)
  double time;                         // 秒
  const unsigned char* mask = nullptr; // valuatorのビットマスク
  int mask_len              = 0;
  const double* values      = nullptr; // maskで立っている軸の値だけが順に並ぶ
};

bool is_eraser_name(std::string name);
float normalize_pressure(double v, double min, double max);
std::optional<double> read_valuator(const unsigned char* mask, int mask_len, const double* values, int axis);
// ペン先(ボタン1)の接触中だけイベントを返す。ホバーとサイドボタンはnullopt
std::optional<PenEvent> translate(PenDevice& d, const DeviceInput& in);

} // namespace xi2

// Linux(X11/XWayland)のペンタブ入力。自前のX接続でペンのslaveデバイスだけを選択するので、GLFWのcoreイベントには影響しない。
// 座標はwindowのクライアント座標。GUIスレッドから使う
class XInput2PenInput : public PenInput {
public:
  XInput2PenInput()                                  = default;
  XInput2PenInput(const XInput2PenInput&)            = delete;
  XInput2PenInput& operator=(const XInput2PenInput&) = delete;
  ~XInput2PenInput() override { close(); }

  // X11のWindow idを渡す。X/XInput2が使えない、windowが無効、libXi無しビルドならfalse。
  // ペンが未接続でもtrue(libinputは最初にペンを近づけた時にデバイスを作るため、抜き差しを監視し続ける)
  bool open(unsigned long window);
  void close();
  bool has_pen() const { return !devices_.empty(); } // 今ペンデバイスがあるか
  // ペンが接地中か、直前までペンの入力があった。trueの間はマウス入力を捨てる(同じ操作がマウスとしても届くため)。poll()の後に見る
  bool pen_active() const;
  std::vector<PenEvent> poll() override;

private:
  void rescan(std::vector<PenEvent>& out); // ペンデバイスを列挙し直してイベントを選択する

  _XDisplay* display_   = nullptr;
  unsigned long window_ = 0;
  int opcode_           = 0;
  double last_input_    = -1e9;           // ペン入力を最後に受けた時刻(steady_clock、秒)
  std::map<int, xi2::PenDevice> devices_; // deviceid →
};

} // namespace mu

#endif
