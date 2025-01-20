if false then
  imgui = require("imgui")
end

package.path = package.path .. ";../lancher/runtime/?.lua"

local init_styles = require("ui_styles")
init_styles()
