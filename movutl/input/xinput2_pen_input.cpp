#ifdef __linux__

#include <algorithm>
#include <cctype>
#include <chrono>
#include <movutl/input/xinput2_pen_input.hpp>

// Xlibのマクロ(None, Status等)が他へ漏れないよう最後にincludeする
#ifdef MOVUTL_XINPUT2
#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>
#endif

namespace mu {

namespace xi2 {

bool is_eraser_name(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return (char)std::tolower(c); });
  return name.find("eraser") != std::string::npos; // wacom/libinput/XWaylandのどれも名前に含む
}

float normalize_pressure(double v, double min, double max) { return max > min ? (float)std::clamp((v - min) / (max - min), 0.0, 1.0) : 1.0f; }

std::optional<double> read_valuator(const unsigned char* mask, int mask_len, const double* values, int axis) {
  if(axis < 0 || axis >= mask_len * 8) return std::nullopt;
  for(int i = 0; i <= axis; i++) {
    if(!(mask[i / 8] & (1 << (i % 8)))) continue;
    if(i == axis) return *values;
    values++;
  }
  return std::nullopt;
}

std::optional<PenEvent> translate(PenDevice& d, const DeviceInput& in) {
  // 離した時の筆圧(ほぼ0)は使わない。手ブレ補正が終点へ追いつく線が消えないように直前の値で終える
  if(in.phase != PenPhase::Up)
    if(auto v = read_valuator(in.mask, in.mask_len, in.values, d.pressure_axis)) d.pressure = normalize_pressure(*v, d.pressure_min, d.pressure_max);
  d.last = {{(float)in.x, (float)in.y, d.pressure}, in.phase, d.eraser, in.time};
  if(in.phase != PenPhase::Move && in.button != 1) return std::nullopt;
  if(in.phase != PenPhase::Down && !d.down) return std::nullopt;
  d.down = in.phase != PenPhase::Up;
  return d.last;
}

} // namespace xi2

namespace {
double now_sec() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
constexpr double kPenHold = 0.3; // ペン入力が途切れてからマウスを受け付けるまで(秒)
} // namespace

bool XInput2PenInput::pen_active() const {
  for(const auto& [_, d] : devices_)
    if(d.down) return true;
  return now_sec() - last_input_ < kPenHold;
}

#ifdef MOVUTL_XINPUT2

namespace {
// Xのエラーは既定でプロセスを終了させるので、無効なwindow idや列挙直後に抜かれたデバイスに備えて一時的に無視する
struct XErrorTrap {
  Display* display;
  XErrorHandler old;
  explicit XErrorTrap(Display* d) : display(d), old(XSetErrorHandler([](Display*, XErrorEvent*) { return 0; })) {}
  ~XErrorTrap() {
    XSync(display, False);
    XSetErrorHandler(old);
  }
};
} // namespace

bool XInput2PenInput::open(unsigned long window) {
  close();
  display_ = XOpenDisplay(nullptr);
  if(!display_) return false;
  int event = 0, error = 0, major = 2, minor = 0;
  bool ok = XQueryExtension(display_, "XInputExtension", &opcode_, &event, &error) && XIQueryVersion(display_, &major, &minor) == Success;
  if(ok) {
    XErrorTrap trap(display_);
    XWindowAttributes attr;
    ok = XGetWindowAttributes(display_, window, &attr) != 0;
  }
  if(!ok) {
    close();
    return false;
  }
  window_ = window;

  unsigned char bits[XIMaskLen(XI_HierarchyChanged)] = {};
  XISetMask(bits, XI_HierarchyChanged);
  XIEventMask mask{XIAllDevices, sizeof(bits), bits};
  XISelectEvents(display_, DefaultRootWindow(display_), &mask, 1);

  std::vector<PenEvent> none;
  rescan(none);
  return true;
}

void XInput2PenInput::close() {
  if(display_) XCloseDisplay(display_);
  display_    = nullptr;
  last_input_ = -1e9;
  devices_.clear();
}

void XInput2PenInput::rescan(std::vector<PenEvent>& out) {
  XErrorTrap trap(display_);
  const Atom pressure = XInternAtom(display_, "Abs Pressure", False);
  std::map<int, xi2::PenDevice> found;
  int n              = 0;
  XIDeviceInfo* info = XIQueryDevice(display_, XIAllDevices, &n);
  for(int i = 0; info && i < n; i++) {
    const XIDeviceInfo& dev = info[i];
    if(dev.use != XISlavePointer || !dev.enabled) continue;
    for(int j = 0; j < dev.num_classes; j++) {
      if(dev.classes[j]->type != XIValuatorClass) continue;
      const auto* v = reinterpret_cast<const XIValuatorClassInfo*>(dev.classes[j]);
      if(v->label != pressure || v->max <= v->min) continue;
      const auto old   = devices_.find(dev.deviceid);
      xi2::PenDevice d = old != devices_.end() ? old->second : xi2::PenDevice{};
      if(old == devices_.end()) d.pressure = xi2::normalize_pressure(v->value, v->min, v->max);
      d.eraser            = xi2::is_eraser_name(dev.name);
      d.pressure_axis     = v->number;
      d.pressure_min      = v->min;
      d.pressure_max      = v->max;
      found[dev.deviceid] = d;

      // slaveデバイスを指定して選択する。XIAllDevices等でmasterを選択するとGLFWへcoreイベントが届かなくなる
      unsigned char bits[XIMaskLen(XI_Motion)] = {};
      XISetMask(bits, XI_ButtonPress); // 暗黙のgrabが付くので、ウィンドウ外へ出てもUpまで届く
      XISetMask(bits, XI_ButtonRelease);
      XISetMask(bits, XI_Motion);
      XIEventMask mask{dev.deviceid, sizeof(bits), bits};
      XISelectEvents(display_, window_, &mask, 1);
    }
  }
  if(info) XIFreeDeviceInfo(info);
  // 接地中に抜かれたペンはUpで閉じる
  for(auto& [id, d] : devices_) {
    if(!d.down || found.count(id)) continue;
    d.last.phase = PenPhase::Up;
    out.push_back(d.last);
  }
  devices_ = std::move(found);
}

std::vector<PenEvent> XInput2PenInput::poll() {
  std::vector<PenEvent> out;
  while(display_ && XPending(display_) > 0) {
    XEvent e;
    XNextEvent(display_, &e);
    XGenericEventCookie* c = &e.xcookie;
    if(c->type != GenericEvent || c->extension != opcode_ || !XGetEventData(display_, c)) continue;
    const int type = c->evtype;
    if(type == XI_Motion || type == XI_ButtonPress || type == XI_ButtonRelease) {
      const auto* ev = static_cast<const XIDeviceEvent*>(c->data);
      const auto it  = devices_.find(ev->deviceid);
      if(it != devices_.end()) {
        const PenPhase phase = type == XI_ButtonPress ? PenPhase::Down : type == XI_ButtonRelease ? PenPhase::Up : PenPhase::Move;
        last_input_          = now_sec();
        if(auto p = xi2::translate(it->second, {phase, ev->detail, ev->event_x, ev->event_y, ev->time / 1000.0, ev->valuators.mask, ev->valuators.mask_len, ev->valuators.values})) out.push_back(*p);
      }
    }
    XFreeEventData(display_, c);
    if(type == XI_HierarchyChanged) rescan(out);
  }
  return out;
}

#else // libX11/libXiが無い環境

bool XInput2PenInput::open(unsigned long) { return false; }
void XInput2PenInput::close() {}
void XInput2PenInput::rescan(std::vector<PenEvent>&) {}
std::vector<PenEvent> XInput2PenInput::poll() { return {}; }

#endif

} // namespace mu

#endif
