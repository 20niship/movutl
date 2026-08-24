#include <movutl/core/defines.hpp>
#include <movutl/core/filesystem.hpp>

MOVUTL_WARNING_PUSH
MOVUTL_DISABLE_ALL_WARNINGS
_Pragma("clang diagnostic ignored \"-Wavailability\"")
_Pragma("clang diagnostic ignored \"-Wmissing-method-return-type\"")

//
#include <AppKit/AppKit.h>
#include <sstream>
#include <string>
#include <vector>

    MOVUTL_WARNING_POP

    namespace mu {

  std::string select_file_dialog(const std::string &title, const std::vector<std::string> &extensions) {
    @autoreleasepool {
      NSOpenPanel *panel = [NSOpenPanel openPanel];
      [panel setAllowsMultipleSelection:NO];
      [panel setCanChooseDirectories:NO];
      [panel setCanChooseFiles:YES];

      panel.title = [NSString stringWithUTF8String:title.c_str()];
      if (!extensions.empty()) {
        NSMutableArray<NSString *> *types = [NSMutableArray array];
        for (const auto &ext : extensions) {
          if (ext.empty()) continue;
          [types addObject:[NSString stringWithUTF8String:ext.c_str()]];
        }
        if (types.count > 0) [panel setAllowedFileTypes:types];
      }

      if ([panel runModal] == NSModalResponseOK) {
        NSURL *url = [[panel URLs] firstObject];
        return std::string([[url path] UTF8String]);
      }
    }

    return "";
  }

#if 0
std::vector<std::string> get_available_fonts() {
  std::vector<std::string> fonts;
  @autoreleasepool {
    CFArrayRef fontFamilies = CTFontManagerCopyAvailableFontFamilyNames();
    CFIndex count = CFArrayGetCount(fontFamilies);
    for (CFIndex i = 0; i < count; i++) {
      CFStringRef fontName =
          (CFStringRef)CFArrayGetValueAtIndex(fontFamilies, i);
      char buffer[256];
      if (CFStringGetCString(fontName, buffer, sizeof(buffer),
                             kCFStringEncodingUTF8)) {
        fonts.push_back(std::string(buffer));
      }
    }
    CFRelease(fontFamilies);
  }
  return fonts;
}
#endif

} // namespace mu
