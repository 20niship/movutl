-- custom lua styles
--

if false then
  imgui = require("imgui")
end

local spectrum = require("imgui_spectrum")

local style_dark2 = {
  colors = {
    {
      {imgui.ImGuiCol_.ImGuiCol_Text, spectrum:GRAY800()},
      {imgui.ImGuiCol_.ImGuiCol_TextDisabled, spectrum:GRAY500()},
      {imgui.ImGuiCol_.ImGuiCol_WindowBg, spectrum:GRAY100()},
      {imgui.ImGuiCol_.ImGuiCol_ChildBg, imgui.ImVec4(0.00, 0.00, 0.00, 0.00)},
      {imgui.ImGuiCol_.ImGuiCol_PopupBg, spectrum:GRAY50()},
      {imgui.ImGuiCol_.ImGuiCol_Border, spectrum:GRAY300()},
      {imgui.ImGuiCol_.ImGuiCol_BorderShadow, spectrum:Static().NONE()},
      {imgui.ImGuiCol_.ImGuiCol_FrameBg, spectrum:GRAY75()},
      {imgui.ImGuiCol_.ImGuiCol_FrameBgHovered, spectrum:GRAY50()},
      {imgui.ImGuiCol_.ImGuiCol_FrameBgActive, spectrum:GRAY200()},
      {imgui.ImGuiCol_.ImGuiCol_TitleBg, spectrum:GRAY300()},
      {imgui.ImGuiCol_.ImGuiCol_TitleBgActive, spectrum:GRAY200()},
      {imgui.ImGuiCol_.ImGuiCol_TitleBgCollapsed, spectrum:GRAY400()},
      {imgui.ImGuiCol_.ImGuiCol_MenuBarBg, spectrum:GRAY100()},
      {imgui.ImGuiCol_.ImGuiCol_ScrollbarBg, spectrum:GRAY100()},
      {imgui.ImGuiCol_.ImGuiCol_ScrollbarGrab, spectrum:GRAY400()},
      {imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabHovered, spectrum:GRAY600()},
      {imgui.ImGuiCol_.ImGuiCol_ScrollbarGrabActive, spectrum:GRAY700()},
      {imgui.ImGuiCol_.ImGuiCol_CheckMark, spectrum:BLUE500()},
      {imgui.ImGuiCol_.ImGuiCol_SliderGrab, spectrum:GRAY700()},
      {imgui.ImGuiCol_.ImGuiCol_SliderGrabActive, spectrum:GRAY800()},
      {imgui.ImGuiCol_.ImGuiCol_Button, spectrum:GRAY75()},
      {imgui.ImGuiCol_.ImGuiCol_ButtonHovered, spectrum:GRAY50()},
      {imgui.ImGuiCol_.ImGuiCol_ButtonActive, spectrum:GRAY200()},
      {imgui.ImGuiCol_.ImGuiCol_Header, spectrum:BLUE400()},
      {imgui.ImGuiCol_.ImGuiCol_HeaderHovered, spectrum:BLUE500()},
      {imgui.ImGuiCol_.ImGuiCol_HeaderActive, spectrum:BLUE600()},
      {imgui.ImGuiCol_.ImGuiCol_Separator, spectrum:GRAY400()},
      {imgui.ImGuiCol_.ImGuiCol_SeparatorHovered, spectrum:GRAY600()},
      {imgui.ImGuiCol_.ImGuiCol_SeparatorActive, spectrum:GRAY700()},
      {imgui.ImGuiCol_.ImGuiCol_ResizeGrip, spectrum:GRAY400()},
      {imgui.ImGuiCol_.ImGuiCol_ResizeGripHovered, spectrum:GRAY600()},
      {imgui.ImGuiCol_.ImGuiCol_ResizeGripActive, spectrum:GRAY700()},
      {imgui.ImGuiCol_.ImGuiCol_PlotLines, spectrum:BLUE400()},
      {imgui.ImGuiCol_.ImGuiCol_PlotLinesHovered, spectrum:BLUE600()},
      {imgui.ImGuiCol_.ImGuiCol_PlotHistogram, spectrum:BLUE400()},
      {imgui.ImGuiCol_.ImGuiCol_PlotHistogramHovered, spectrum:BLUE600()},
      {imgui.ImGuiCol_.ImGuiCol_TextSelectedBg, (spectrum:BLUE400() & 0x00FFFFFF) | 0x33000000},
      {imgui.ImGuiCol_.ImGuiCol_DragDropTarget, imgui.ImVec4(1.00, 1.00, 0.00, 0.90)},
      {imgui.ImGuiCol_.ImGuiCol_NavWindowingHighlight, imgui.ImVec4(1.00, 1.00, 1.00, 0.70)},
      {imgui.ImGuiCol_.ImGuiCol_NavWindowingDimBg, imgui.ImVec4(0.80, 0.80, 0.80, 0.20)},
      {imgui.ImGuiCol_.ImGuiCol_ModalWindowDimBg, imgui.ImVec4(0.20, 0.20, 0.20, 0.35)},
    }
  };
  styles = {
    {imgui.ImGuiStyleVar_.ImGuiStyleVar_GrabRounding, 4.0},
  }
}

