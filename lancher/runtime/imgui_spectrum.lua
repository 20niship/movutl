-- Pick one theme
SPECTRUM_USE_LIGHT_THEME = true
-- SPECTRUM_USE_DARK_THEME = true

local Spectrum = {}

-- Widget parameters
Spectrum.CHECKBOX_BORDER_SIZE = 2.0
Spectrum.CHECKBOX_ROUNDING = 2.0

-- Function to convert color format
local function Color(c)
  -- Add alpha and swap red and blue channels
  local a = 0xFF
  local r = (c >> 16) & 0xFF
  local g = (c >> 8) & 0xFF
  local b = (c >> 0) & 0xFF
  return (a << 24) | (r << 0) | (g << 8) | (b << 16)
end

-- Function to apply alpha to a color
function Spectrum.color_alpha(alpha, c)
  return ((alpha & 0xFF) << 24) | (c & 0x00FFFFFF)
end

-- Static colors
Spectrum.Static = {
  NONE = 0x00000000,
  WHITE = Color(0xFFFFFF),
  BLACK = Color(0x000000),
  GRAY200 = Color(0xF4F4F4),
  GRAY300 = Color(0xEAEAEA),
  GRAY400 = Color(0xD3D3D3),
  GRAY500 = Color(0xBCBCBC),
  GRAY600 = Color(0x959595),
  GRAY700 = Color(0x767676),
  GRAY800 = Color(0x505050),
  GRAY900 = Color(0x323232),
  BLUE400 = Color(0x378EF0),
  BLUE500 = Color(0x2680EB),
  BLUE600 = Color(0x1473E6),
  BLUE700 = Color(0x0D66D0),
  RED400 = Color(0xEC5B62),
  RED500 = Color(0xE34850),
  RED600 = Color(0xD7373F),
  RED700 = Color(0xC9252D),
  ORANGE400 = Color(0xF29423),
  ORANGE500 = Color(0xE68619),
  ORANGE600 = Color(0xDA7B11),
  ORANGE700 = Color(0xCB6F10),
  GREEN400 = Color(0x33AB84),
  GREEN500 = Color(0x2D9D78),
  GREEN600 = Color(0x268E6C),
  GREEN700 = Color(0x12805C)
}

-- Theme-specific colors
if SPECTRUM_USE_LIGHT_THEME then
  Spectrum.GRAY50 = Color(0xFFFFFF)
  Spectrum.GRAY75 = Color(0xFAFAFA)
  Spectrum.GRAY100 = Color(0xF5F5F5)
  Spectrum.GRAY200 = Color(0xEAEAEA)
  Spectrum.GRAY300 = Color(0xE1E1E1)
  Spectrum.GRAY400 = Color(0xCACACA)
  Spectrum.GRAY500 = Color(0xB3B3B3)
  Spectrum.GRAY600 = Color(0x8E8E8E)
  Spectrum.GRAY700 = Color(0x707070)
  Spectrum.GRAY800 = Color(0x4B4B4B)
  Spectrum.GRAY900 = Color(0x2C2C2C)
  Spectrum.BLUE400 = Color(0x2680EB)
  Spectrum.BLUE500 = Color(0x1473E6)
  Spectrum.BLUE600 = Color(0x0D66D0)
  Spectrum.BLUE700 = Color(0x095ABA)
  Spectrum.RED400 = Color(0xE34850)
  Spectrum.RED500 = Color(0xD7373F)
  Spectrum.RED600 = Color(0xC9252D)
  Spectrum.RED700 = Color(0xBB121A)
  Spectrum.ORANGE400 = Color(0xE68619)
  Spectrum.ORANGE500 = Color(0xDA7B11)
  Spectrum.ORANGE600 = Color(0xCB6F10)
  Spectrum.ORANGE700 = Color(0xBD640D)
  Spectrum.GREEN400 = Color(0x2D9D78)
  Spectrum.GREEN500 = Color(0x268E6C)
  Spectrum.GREEN600 = Color(0x12805C)
  Spectrum.GREEN700 = Color(0x107154)
end

return Spectrum

