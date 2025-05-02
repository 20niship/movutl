-- custom lua styles

local inspect = require('lib/inspect')

if false then
  imgui = require("imgui")
  movutl = require("movutl")
end

local spectrum = require("imgui_spectrum")

-- @param name: string
-- @param style: table
local function impl_register_style(name, style)
  local new_style = imgui.new_style()
  for i, v in ipairs(style.colors) do
    if v[1] == nil or v[2] == nil then
      -- print("invalid color scheme" .. name .. " " .. inspect(v))
      goto continue
    end
    new_style = imgui.set_imgui_style_color(new_style, v[1], v[2])
    ::continue::
  end
  style.func(new_style)
  movutl.register_imgui_style(name, new_style)
end

local style_dark2 = {
  colors = {
    { imgui.ImGuiCol_.ImGuiCol_Text,                  spectrum.GRAY800 },
    { imgui.ImGuiCol_.ImGuiCol_TextDisabled,          spectrum.GRAY500 },
    { imgui.ImGuiCol_.ImGuiCol_WindowBg,              spectrum.GRAY100 },
    { imgui.ImGuiCol_.ImGuiCol_ChildBg,               imgui.ImVec4(0.00, 0.00, 0.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_PopupBg,               spectrum.GRAY50 },
    { imgui.ImGuiCol_.ImGuiCol_Border,                spectrum.GRAY300 },
    { imgui.ImGuiCol_.ImGuiCol_BorderShadow,          spectrum.NONE },
    { imgui.ImGuiCol_.ImGuiCol_FrameBg,               spectrum.GRAY75 },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgHovered,        spectrum.GRAY50 },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgActive,         spectrum.GRAY200 },
    { imgui.ImGuiCol_.ImGuiCol_TitleBg,               spectrum.GRAY300 },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgActive,         spectrum.GRAY200 },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgCollapsed,      spectrum.GRAY400 },
    { imgui.ImGuiCol_.ImGuiCol_MenuBarBg,             spectrum.GRAY100 },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarBg,           spectrum.GRAY100 },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrab,         spectrum.GRAY400 },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabHovered,  spectrum.GRAY600 },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabActive,   spectrum.GRAY700 },
    { imgui.ImGuiCol_.ImGuiCol_CheckMark,             spectrum.BLUE500 },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrab,            spectrum.GRAY700 },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrabActive,      spectrum.GRAY800 },
    { imgui.ImGuiCol_.ImGuiCol_Button,                spectrum.GRAY75 },
    { imgui.ImGuiCol_.ImGuiCol_ButtonHovered,         spectrum.GRAY50 },
    { imgui.ImGuiCol_.ImGuiCol_ButtonActive,          spectrum.GRAY200 },
    { imgui.ImGuiCol_.ImGuiCol_Header,                spectrum.BLUE400 },
    { imgui.ImGuiCol_.ImGuiCol_HeaderHovered,         spectrum.BLUE500 },
    { imgui.ImGuiCol_.ImGuiCol_HeaderActive,          spectrum.BLUE600 },
    { imgui.ImGuiCol_.ImGuiCol_Separator,             spectrum.GRAY400 },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorHovered,      spectrum.GRAY600 },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorActive,       spectrum.GRAY700 },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGrip,            spectrum.GRAY400 },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripHovered,     spectrum.GRAY600 },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripActive,      spectrum.GRAY700 },
    { imgui.ImGuiCol_.ImGuiCol_PlotLines,             spectrum.BLUE400 },
    { imgui.ImGuiCol_.ImGuiCol_PlotLinesHovered,      spectrum.BLUE600 },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogram,         spectrum.BLUE400 },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogramHovered,  spectrum.BLUE600 },
    { imgui.ImGuiCol_.ImGuiCol_TextSelectedBg,        spectrum.BLUE400 },
    { imgui.ImGuiCol_.ImGuiCol_DragDropTarget,        imgui.ImVec4(1.00, 1.00, 0.00, 0.90) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingHighlight, imgui.ImVec4(1.00, 1.00, 1.00, 0.70) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingDimBg,     imgui.ImVec4(0.80, 0.80, 0.80, 0.20) },
    { imgui.ImGuiCol_.ImGuiCol_ModalWindowDimBg,      imgui.ImVec4(0.20, 0.20, 0.20, 0.35) },
  },

  -- @param style: imgui.ImGuiStyle
  func = function(style)
    style.GrabRounding = 4.0
  end
}

-- https://github.com/ocornut/imgui/issues/707#issuecomment-415097227
local style_dark3 = {
  colors = {
    { imgui.ImGuiCol_.ImGuiCol_Text,                  imgui.ImVec4(1.00, 1.00, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextDisabled,          imgui.ImVec4(0.50, 0.50, 0.50, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_WindowBg,              imgui.ImVec4(0.06, 0.06, 0.06, 0.94) },
    { imgui.ImGuiCol_.ImGuiCol_ChildBg,               imgui.ImVec4(1.00, 1.00, 1.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_PopupBg,               imgui.ImVec4(0.08, 0.08, 0.08, 0.94) },
    { imgui.ImGuiCol_.ImGuiCol_Border,                imgui.ImVec4(0.43, 0.43, 0.50, 0.50) },
    { imgui.ImGuiCol_.ImGuiCol_BorderShadow,          imgui.ImVec4(0.00, 0.00, 0.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBg,               imgui.ImVec4(0.20, 0.21, 0.22, 0.54) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgHovered,        imgui.ImVec4(0.40, 0.40, 0.40, 0.40) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgActive,         imgui.ImVec4(0.18, 0.18, 0.18, 0.67) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBg,               imgui.ImVec4(0.04, 0.04, 0.04, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgActive,         imgui.ImVec4(0.29, 0.29, 0.29, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgCollapsed,      imgui.ImVec4(0.00, 0.00, 0.00, 0.51) },
    { imgui.ImGuiCol_.ImGuiCol_MenuBarBg,             imgui.ImVec4(0.14, 0.14, 0.14, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarBg,           imgui.ImVec4(0.02, 0.02, 0.02, 0.53) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrab,         imgui.ImVec4(0.31, 0.31, 0.31, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabHovered,  imgui.ImVec4(0.41, 0.41, 0.41, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabActive,   imgui.ImVec4(0.51, 0.51, 0.51, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_CheckMark,             imgui.ImVec4(0.94, 0.94, 0.94, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrab,            imgui.ImVec4(0.51, 0.51, 0.51, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrabActive,      imgui.ImVec4(0.86, 0.86, 0.86, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Button,                imgui.ImVec4(0.44, 0.44, 0.44, 0.40) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonHovered,         imgui.ImVec4(0.46, 0.47, 0.48, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonActive,          imgui.ImVec4(0.42, 0.42, 0.42, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Header,                imgui.ImVec4(0.70, 0.70, 0.70, 0.31) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderHovered,         imgui.ImVec4(0.70, 0.70, 0.70, 0.80) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderActive,          imgui.ImVec4(0.48, 0.50, 0.52, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Separator,             imgui.ImVec4(0.43, 0.43, 0.50, 0.50) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorHovered,      imgui.ImVec4(0.72, 0.72, 0.72, 0.78) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorActive,       imgui.ImVec4(0.51, 0.51, 0.51, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGrip,            imgui.ImVec4(0.91, 0.91, 0.91, 0.25) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripHovered,     imgui.ImVec4(0.81, 0.81, 0.81, 0.67) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripActive,      imgui.ImVec4(0.46, 0.46, 0.46, 0.95) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLines,             imgui.ImVec4(0.61, 0.61, 0.61, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLinesHovered,      imgui.ImVec4(1.00, 0.43, 0.35, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogram,         imgui.ImVec4(0.73, 0.60, 0.15, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogramHovered,  imgui.ImVec4(1.00, 0.60, 0.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextSelectedBg,        imgui.ImVec4(0.87, 0.87, 0.87, 0.35) },
    -- { imgui.ImGuiCol_.ImGuiCol_ModalWindowDarkening,    imgui.ImVec4(0.80, 0.80, 0.80, 0.35) },
    { imgui.ImGuiCol_.ImGuiCol_DragDropTarget,        imgui.ImVec4(1.00, 1.00, 0.00, 0.90) },
    -- { imgui.ImGuiCol_.ImGuiCol_NavHighlight,            imgui.ImVec4(0.60, 0.60, 0.60, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingHighlight, imgui.ImVec4(1.00, 1.00, 1.00, 0.70) },
  },
  -- @param style: imgui.ImGuiStyle
  func = function(style)
  end
}

-- https://github.com/ocornut/imgui/issues/707#issuecomment-512669512
--
local style_dark4 = {
  colors = {
    { imgui.ImGuiCol_.ImGuiCol_Text,                  imgui.ImVec4(0.95, 0.96, 0.98, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextDisabled,          imgui.ImVec4(0.36, 0.42, 0.47, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_WindowBg,              imgui.ImVec4(0.11, 0.15, 0.17, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ChildBg,               imgui.ImVec4(0.15, 0.18, 0.22, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PopupBg,               imgui.ImVec4(0.08, 0.08, 0.08, 0.94) },
    { imgui.ImGuiCol_.ImGuiCol_Border,                imgui.ImVec4(0.08, 0.10, 0.12, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_BorderShadow,          imgui.ImVec4(0.00, 0.00, 0.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBg,               imgui.ImVec4(0.20, 0.25, 0.29, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgHovered,        imgui.ImVec4(0.12, 0.20, 0.28, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgActive,         imgui.ImVec4(0.09, 0.12, 0.14, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBg,               imgui.ImVec4(0.09, 0.12, 0.14, 0.65) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgActive,         imgui.ImVec4(0.08, 0.10, 0.12, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgCollapsed,      imgui.ImVec4(0.00, 0.00, 0.00, 0.51) },
    { imgui.ImGuiCol_.ImGuiCol_MenuBarBg,             imgui.ImVec4(0.15, 0.18, 0.22, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarBg,           imgui.ImVec4(0.02, 0.02, 0.02, 0.39) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrab,         imgui.ImVec4(0.20, 0.25, 0.29, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabHovered,  imgui.ImVec4(0.18, 0.22, 0.25, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabActive,   imgui.ImVec4(0.09, 0.21, 0.31, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_CheckMark,             imgui.ImVec4(0.28, 0.56, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrab,            imgui.ImVec4(0.28, 0.56, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrabActive,      imgui.ImVec4(0.37, 0.61, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Button,                imgui.ImVec4(0.20, 0.25, 0.29, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonHovered,         imgui.ImVec4(0.28, 0.56, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonActive,          imgui.ImVec4(0.06, 0.53, 0.98, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Header,                imgui.ImVec4(0.20, 0.25, 0.29, 0.55) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderHovered,         imgui.ImVec4(0.26, 0.59, 0.98, 0.80) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderActive,          imgui.ImVec4(0.26, 0.59, 0.98, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Separator,             imgui.ImVec4(0.20, 0.25, 0.29, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorHovered,      imgui.ImVec4(0.10, 0.40, 0.75, 0.78) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorActive,       imgui.ImVec4(0.10, 0.40, 0.75, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGrip,            imgui.ImVec4(0.26, 0.59, 0.98, 0.25) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripHovered,     imgui.ImVec4(0.26, 0.59, 0.98, 0.67) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripActive,      imgui.ImVec4(0.26, 0.59, 0.98, 0.95) },
    { imgui.ImGuiCol_.ImGuiCol_Tab,                   imgui.ImVec4(0.11, 0.15, 0.17, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TabHovered,            imgui.ImVec4(0.26, 0.59, 0.98, 0.80) },
    -- { imgui.ImGuiCol_.ImGuiCol_TabActive,             imgui.ImVec4(0.20, 0.25, 0.29, 1.00) },
    -- { imgui.ImGuiCol_.ImGuiCol_TabUnfocused,          imgui.ImVec4(0.11, 0.15, 0.17, 1.00) },
    -- { imgui.ImGuiCol_.ImGuiCol_TabUnfocusedActive,    imgui.ImVec4(0.11, 0.15, 0.17, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLines,             imgui.ImVec4(0.61, 0.61, 0.61, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLinesHovered,      imgui.ImVec4(1.00, 0.43, 0.35, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogram,         imgui.ImVec4(0.90, 0.70, 0.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogramHovered,  imgui.ImVec4(1.00, 0.60, 0.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextSelectedBg,        imgui.ImVec4(0.26, 0.59, 0.98, 0.35) },
    { imgui.ImGuiCol_.ImGuiCol_DragDropTarget,        imgui.ImVec4(1.00, 1.00, 0.00, 0.90) },
    -- { imgui.ImGuiCol_.ImGuiCol_NavHighlight,          imgui.ImVec4(0.26, 0.59, 0.98, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingHighlight, imgui.ImVec4(1.00, 1.00, 1.00, 0.70) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingDimBg,     imgui.ImVec4(0.80, 0.80, 0.80, 0.20) },
    { imgui.ImGuiCol_.ImGuiCol_ModalWindowDimBg,      imgui.ImVec4(0.80, 0.80, 0.80, 0.35) },
  },
  -- @param style: imgui.ImGuiStyle
  func = function(style)
    style.FrameRounding = 4.0
    style.GrabRounding = 4.0
  end
}

-- https://github.com/ocornut/imgui/issues/707#issuecomment-678611331
local style_dark5 = {
  colors = {
    { imgui.ImGuiCol_.ImGuiCol_Text,                  imgui.ImVec4(1.00, 1.00, 1.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextDisabled,          imgui.ImVec4(0.50, 0.50, 0.50, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_WindowBg,              imgui.ImVec4(0.13, 0.14, 0.15, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ChildBg,               imgui.ImVec4(0.13, 0.14, 0.15, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PopupBg,               imgui.ImVec4(0.13, 0.14, 0.15, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Border,                imgui.ImVec4(0.43, 0.43, 0.50, 0.50) },
    { imgui.ImGuiCol_.ImGuiCol_BorderShadow,          imgui.ImVec4(0.00, 0.00, 0.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBg,               imgui.ImVec4(0.25, 0.25, 0.25, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgHovered,        imgui.ImVec4(0.38, 0.38, 0.38, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_FrameBgActive,         imgui.ImVec4(0.67, 0.67, 0.67, 0.39) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBg,               imgui.ImVec4(0.08, 0.08, 0.09, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgActive,         imgui.ImVec4(0.08, 0.08, 0.09, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TitleBgCollapsed,      imgui.ImVec4(0.00, 0.00, 0.00, 0.51) },
    { imgui.ImGuiCol_.ImGuiCol_MenuBarBg,             imgui.ImVec4(0.14, 0.14, 0.14, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarBg,           imgui.ImVec4(0.02, 0.02, 0.02, 0.53) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrab,         imgui.ImVec4(0.31, 0.31, 0.31, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabHovered,  imgui.ImVec4(0.41, 0.41, 0.41, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabActive,   imgui.ImVec4(0.51, 0.51, 0.51, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_CheckMark,             imgui.ImVec4(0.11, 0.64, 0.92, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrab,            imgui.ImVec4(0.11, 0.64, 0.92, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SliderGrabActive,      imgui.ImVec4(0.08, 0.50, 0.72, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_Button,                imgui.ImVec4(0.25, 0.25, 0.25, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonHovered,         imgui.ImVec4(0.38, 0.38, 0.38, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_ButtonActive,          imgui.ImVec4(0.67, 0.67, 0.67, 0.39) },
    { imgui.ImGuiCol_.ImGuiCol_Header,                imgui.ImVec4(0.22, 0.22, 0.22, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderHovered,         imgui.ImVec4(0.25, 0.25, 0.25, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_HeaderActive,          imgui.ImVec4(0.67, 0.67, 0.67, 0.39) },
    { imgui.ImGuiCol_.ImGuiCol_Separator,             imgui.ImVec4(0.43, 0.43, 0.50, 0.50) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorHovered,      imgui.ImVec4(0.41, 0.42, 0.44, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_SeparatorActive,       imgui.ImVec4(0.26, 0.59, 0.98, 0.95) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGrip,            imgui.ImVec4(0.00, 0.00, 0.00, 0.00) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripHovered,     imgui.ImVec4(0.29, 0.30, 0.31, 0.67) },
    { imgui.ImGuiCol_.ImGuiCol_ResizeGripActive,      imgui.ImVec4(0.26, 0.59, 0.98, 0.95) },
    { imgui.ImGuiCol_.ImGuiCol_Tab,                   imgui.ImVec4(0.08, 0.08, 0.09, 0.83) },
    { imgui.ImGuiCol_.ImGuiCol_TabHovered,            imgui.ImVec4(0.33, 0.34, 0.36, 0.83) },
    -- {imgui.ImGuiCol_.ImGuiCol_TabActive,            imgui.ImVec4(0.23, 0.23, 0.24, 1.00)},
    -- {imgui.ImGuiCol_.ImGuiCol_TabUnfocused,         imgui.ImVec4(0.08, 0.08, 0.09, 1.00)},
    -- {imgui.ImGuiCol_.ImGuiCol_TabUnfocusedActive,   imgui.ImVec4(0.13, 0.14, 0.15, 1.00)},
    { imgui.ImGuiCol_.ImGuiCol_DockingPreview,        imgui.ImVec4(0.26, 0.59, 0.98, 0.70) },
    { imgui.ImGuiCol_.ImGuiCol_DockingEmptyBg,        imgui.ImVec4(0.20, 0.20, 0.20, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLines,             imgui.ImVec4(0.61, 0.61, 0.61, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotLinesHovered,      imgui.ImVec4(1.00, 0.43, 0.35, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogram,         imgui.ImVec4(0.90, 0.70, 0.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_PlotHistogramHovered,  imgui.ImVec4(1.00, 0.60, 0.00, 1.00) },
    { imgui.ImGuiCol_.ImGuiCol_TextSelectedBg,        imgui.ImVec4(0.26, 0.59, 0.98, 0.35) },
    { imgui.ImGuiCol_.ImGuiCol_DragDropTarget,        imgui.ImVec4(0.11, 0.64, 0.92, 1.00) },
    -- {imgui.ImGuiCol_.ImGuiCol_NavHighlight,         imgui.ImVec4(0.26, 0.59, 0.98, 1.00)},
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingHighlight, imgui.ImVec4(1.00, 1.00, 1.00, 0.70) },
    { imgui.ImGuiCol_.ImGuiCol_NavWindowingDimBg,     imgui.ImVec4(0.80, 0.80, 0.80, 0.20) },
    { imgui.ImGuiCol_.ImGuiCol_ModalWindowDimBg,      imgui.ImVec4(0.80, 0.80, 0.80, 0.35) },
  },
  -- @param style: imgui.ImGuiStyle
  func = function(style)
    style.FrameRounding = 2.3
    style.GrabRounding = 2.3
  end
}

local function init_color_schemes()
  impl_register_style("dark2", style_dark2)
  impl_register_style("dark3", style_dark3)
  impl_register_style("dark4", style_dark4)
  impl_register_style("dark5", style_dark5)
  movutl.apply_imgui_style("dark5")
end

return init_color_schemes
