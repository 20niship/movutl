#include <csignal>
#include <movutl/core/assert.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/platform.hpp>

// #define _GNU_SOURCE
#include <csignal>

#if defined(WIN32) || defined(_WIN32)
#include <Windows.h>
// 順番関係ある windows.hより先に入れるとエラーになる
#include <DbgHelp.h>
#include <cstdlib>
#pragma comment(lib, "imagehlp.lib")
#else
#include <cxxabi.h>
#include <dlfcn.h>
#include <execinfo.h>
#endif

#include <cstdlib>

inline const std::string cout_yellow = "\033[33m";
inline const std::string cout_clear = "\033[0m";

namespace mu::detail {

void _mu_assert_fail(const char* file, int line, const char* msg1) {
  LOG_F(ERROR, "_mu_assert_fail: %s:%d: %s", file, line, msg1);
  auto backtrace = get_backtrace();
  for(auto& bt : backtrace) {
    printf("  %s\n", bt.c_str());
  }
#ifdef _WIN32
  std::exit(1);
#else
  std::abort();
#endif
}

void enable_signal_handlers() {
  std::signal(SIGINT, [](int) { MU_FAIL("Interrupted by user"); });
  signal(SIGSEGV, [](int) { MU_FAIL("Segmentation fault"); });
  signal(SIGFPE, [](int) { MU_FAIL("Floating point exception"); });
}

struct BacktraceState {
  std::string file;
  std::string function;
  std::string line;
  std::string offset;

  [[nodiscard]] std::string toString() const { return " [ " + cout_yellow + function + cout_clear + " ] " + file + " " + line + " " + offset; }
};

static BacktraceState demangle(const std::string& symbol) {
  BacktraceState res;
  res.function = symbol;

#if defined(WIN32) || defined(_WIN32)
  SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
  if(!SymInitialize(GetCurrentProcess(), NULL, TRUE)) {
    std::cout << "SymInitialize returned error: " << GetLastError() << '\n';
    return res;
  }

  const char* decorated_name = typeid(symbol).name();
  char undecorated_name[1024];
  if(!UnDecorateSymbolName(decorated_name, undecorated_name, sizeof(undecorated_name) / sizeof(*undecorated_name), UNDNAME_COMPLETE)) {
    std::cout << "UnDecorateSymbolName returned error: " << GetLastError() << '\n';
    return res;
  }

  res.function = undecorated_name;
  return res;
#else
  auto start = symbol.find('(');
  auto end = symbol.find('+');
  auto end2 = symbol.find(')');
  if(start == std::string::npos || end == std::string::npos || start >= end) {
    return res;
  }
  if(end2 == std::string::npos) end2 = symbol.size() - 1;

  res.file = symbol.substr(0, start);
  auto name = symbol.substr(start + 1, end - start - 1);
  res.offset = symbol.substr(end2 + 1);

  int status;
  auto demangled = abi::__cxa_demangle(name.c_str(), 0, 0, &status);
  res.function = (status == 0) ? demangled : name;
  return res;
#endif
}

std::vector<std::string> get_backtrace() {
  std::vector<std::string> result;
  constexpr int trace_size = 100;
#if defined(WIN32) || defined(_WIN32)
  void* traces[trace_size] = {};
  HANDLE process = GetCurrentProcess();
  SymInitialize(process, NULL, TRUE);
  uint16_t trace_size2 = CaptureStackBackTrace(0, trace_size, traces, NULL);
  constexpr size_t MaxNameSize = 255;
  constexpr size_t SymbolInfoSize = sizeof(SYMBOL_INFO) + ((MaxNameSize + 1) * sizeof(char));
  SYMBOL_INFO* symbol = (SYMBOL_INFO*)calloc(SymbolInfoSize, 1);
  if(symbol == nullptr) {
    std::cerr << "melchior malloc error : backtrace() malloc of symbols SymbolInfoSize = " << SymbolInfoSize << std::endl;
    return {};
  }
  symbol->MaxNameLen = MaxNameSize;
  symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
  for(uint16_t i = 0; i < trace_size2; i++) {
    SymFromAddr(process, (DWORD64)(traces[i]), 0, symbol);
    /* トレースアドレスをトレースリストに追加 */
    // stack_trace.traces.push_back(traces[i]);
    /* シンボル名をシンボルリストに追加 */
    result.push_back(std::string(symbol->Name));
  }
  free(symbol);
#else
  void* callstack[trace_size];
  int frames = ::backtrace(callstack, trace_size);
  char** symbols = ::backtrace_symbols(callstack, frames);
  for(int i = 0; i < frames; i++) {
    char* symbol = symbols[i];
    auto demangled = detail::demangle(symbol);
    result.push_back(demangled.toString());
  }
  ::free(symbols);
#endif
  return result;
}

} // namespace mu::detail
