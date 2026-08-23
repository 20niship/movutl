#include <cstring>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {
extern InputPluginTable plg_image_reader;
} // namespace mu::detail

namespace {

void plugin_init(mu::ABIContext* abi) { abi->register_input_plugin(&mu::detail::plg_image_reader); }
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "STB Image Reader", sizeof(table->name) - 1);
  std::strncpy(table->description, "stb_imageを用いた画像入力プラグイン", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
