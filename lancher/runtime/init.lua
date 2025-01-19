print(1 + 1)

if false then
  imgui = require("imgui")
end

-- require module
print(movutl)
print(imgui)

local io = imgui.GetIO()
print(io.DisplaySize.x, io.DisplaySize.y)

print("package.path", package.path)
package.path = package.path .. ";../lancher/runtime/?.lua"

local init_styles = require("ui_styles")
init_styles()

