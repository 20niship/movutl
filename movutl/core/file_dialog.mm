#include <movutl/core/filesystem.hpp>
//
#include <Cocoa/Cocoa.h>
#include <sstream>
#include <string>
#include <vector>

namespace mu {

std::string select_file_dialog(const std::vector<std::string> &extensions) {
  @autoreleasepool {
    NSOpenPanel *panel = [NSOpenPanel openPanel];
    [panel setAllowsMultipleSelection:NO];
    [panel setCanChooseDirectories:NO];
    [panel setCanChooseFiles:YES];

    NSMutableArray<NSString *> *types = [NSMutableArray array];
    for (const auto &ext : extensions) {
      [types addObject:[NSString stringWithUTF8String:ext.c_str()]];
    }
    [panel setAllowedFileTypes:types];

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
