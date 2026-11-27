// glfw3native.hはXlibのマクロ(None, Status等)を持ち込むので、このファイルに隔離する
#include <movutl/gui/paint/canvas_input.hpp>

#if defined(__linux__) && __has_include(<X11/extensions/Xrandr.h>)
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3native.h>
#endif

namespace mu {

unsigned long x11_window_id(GLFWwindow* window) {
#ifdef GLFW_EXPOSE_NATIVE_X11
  return window ? glfwGetX11Window(window) : 0;
#else
  (void)window;
  return 0;
#endif
}

} // namespace mu
