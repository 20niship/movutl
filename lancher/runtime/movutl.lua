-- movutl pygen auto generated bindings
--@meta
--@class movutl
local movutl = {}

---@class ImVec2
---@field x number
---@field y number
imgui.ImVec2= {}

---@class ImVec4
---@field x number
---@field y number
---@field z number
---@field w number
imgui.ImVec4= {}

---@class ImColor
---@field Value ImVec4
imgui.ImColor= {}

---@class AniInterpType
---@field LINEAR number
---@field EaseIn number
---@field EaseOut number
---@field EaseInOut number
---@field EaseInQuad number
---@field EaseOutQuad number
---@field EaseInOutQuad number
---@field EaseInCubic number
---@field EaseOutCubic number
---@field EaseInOutCubic number
---@field Custom number
movutl.AniInterpType = {}

---@class EntityType
---@field EntityType_Movie number
---@field EntityType_Audio number
---@field EntityType_Image number
---@field EntityType_3DText number
---@field EntityType_Primitive number
---@field EntityType_Framebuffer number
---@field EntityType_Polygon number
---@field EntityType_Group number
---@field EntityType_Scene number
---@field EntityType_SceneAudio number
---@field EntityType_LayerCopy number
---@field EntityType_Particle number
---@field EntityType_Custom number
---@field EntityType_3DModel number
---@field EntityType_Camera number
---@field EntityType_Effect number
movutl.EntityType = {}

---@class Flag
---@field setting_dialog number
---@field frame_alpha number
---@field fast_preview number
---@field preprocessing number
---@field hide_output_gui number
---@field nesting number
---@field invert_field_order number
---@field invert_interlace number
movutl.Flag = {}

---@class ImDrawFlags_
---@field ImDrawFlags_None number
---@field ImDrawFlags_Closed number
---@field ImDrawFlags_RoundCornersTopLeft number
---@field ImDrawFlags_RoundCornersTopRight number
---@field ImDrawFlags_RoundCornersBottomLeft number
---@field ImDrawFlags_RoundCornersBottomRight number
---@field ImDrawFlags_RoundCornersNone number
---@field ImDrawFlags_RoundCornersTop number
---@field ImDrawFlags_RoundCornersBottom number
---@field ImDrawFlags_RoundCornersLeft number
---@field ImDrawFlags_RoundCornersRight number
---@field ImDrawFlags_RoundCornersAll number
---@field ImDrawFlags_RoundCornersDefault_ number
---@field ImDrawFlags_RoundCornersMask_ number
movutl.ImDrawFlags_ = {}

---@class ImDrawListFlags_
---@field ImDrawListFlags_None number
---@field ImDrawListFlags_AntiAliasedLines number
---@field ImDrawListFlags_AntiAliasedLinesUseTex number
---@field ImDrawListFlags_AntiAliasedFill number
---@field ImDrawListFlags_AllowVtxOffset number
movutl.ImDrawListFlags_ = {}

---@class ImFontAtlasFlags_
---@field ImFontAtlasFlags_None number
---@field ImFontAtlasFlags_NoPowerOfTwoHeight number
---@field ImFontAtlasFlags_NoMouseCursors number
---@field ImFontAtlasFlags_NoBakedLines number
movutl.ImFontAtlasFlags_ = {}

---@class ImageFormat
---@field ImageFormatRGB number
---@field ImageFormatRGBA number
---@field ImageFormatGRAYSCALE number
movutl.ImageFormat = {}

---@class Composition
---@field guid number
---@field name FixString
---@field flag Flag
---@field frame_final Ref<Image>
---@field frame_edit Ref<Image>
---@field frame_temp Ref<Image>
---@field framerate_nu number
---@field framerate_de number
---@field fstart number
---@field fend number
---@field frame number
---@field audio_n number
---@field audio_ch number
---@field layers table
movutl.Composition = {}

---@param w number
---@param h number
---@return nil
function movutl.Composition:resize( w, h, ) end

---@return string
function movutl.Composition:str( ) end

---@return string
function movutl.Composition:summary( ) end

---@return Composition 
function movutl.Composition:GetActiveComp( ) end

---@class EntityInfo
---@field flag EntityType
---@field framerate number
---@field nframes number
---@field format ImageFormat
---@field width number
---@field height number
---@field audio_n number
---@field audio_format_size number
movutl.EntityInfo = {}

---@return string
function movutl.EntityInfo:str( ) end

---@class Image
---@field fmt ImageFormat
---@field width number
---@field height number
---@field pos Vec3
---@field scale Vec2
---@field rotation number
---@field alpha number
---@field path string
---@field dirty_ number
movutl.Image = {}

---@return nil
function movutl.Image:dirty( ) end

---@return number
function movutl.Image:data( ) end

---@param x size_t
---@param y size_t
---@param rgb Vec3b 
---@return nil
function movutl.Image:set_rgb( x, y, rgb, ) end

---@param x size_t
---@param y size_t
---@param rgba Vec4b 
---@return nil
function movutl.Image:set_rgba( x, y, rgba, ) end

---@return size_t
function movutl.Image:size( ) end

---@return size_t
function movutl.Image:size_in_bytes( ) end

---@return nil
function movutl.Image:reset( ) end

---@param v number
---@return nil
function movutl.Image:fill( v, ) end

---@return number
function movutl.Image:channels( ) end

---@param x size_t
---@param y size_t
---@return Vec4b
function movutl.Image:rgba( x, y, ) end

---@param name string
---@return nil
function movutl.Image:imshow( name, ) end

---@return EntityType
function movutl.Image:getType( ) end

---@class Movie
---@field pos Vec3
---@field scale Vec2
---@field rotation number
---@field start_frame_ number
---@field speed number
---@field alpha_ number
---@field loop_ boolean
---@field with_alpha_ boolean
---@field path_ string
movutl.Movie = {}

---@param name string
---@param path string
---@return Ref<Movie>
function movutl.Movie:Create( name, path, ) end

---@param path string
---@return boolean
function movutl.Movie:load_file( path, ) end

---@return EntityType
function movutl.Movie:getType( ) end

---@return PropsInfo
function movutl.Movie:getPropsInfo( ) end

---@return Props
function movutl.Movie:getProps( ) end

---@param props Props 
---@return nil
function movutl.Movie:setProps( props, ) end

---@class Project
---@field path string
---@field output_path string
---@field entities table
---@field compos_ table
---@field main_comp_idx number
movutl.Project = {}

---@param width number
---@param height number
---@param fps number
---@return nil
function movutl.Project:New( width, height, fps, ) end

---@return [ [ deprecated ] ] Composition 
function movutl.Project:get_main_comp( ) end

---@return Composition 
function movutl.Project:GetActiveCompo( ) end

---@param idx number
---@return nil
function movutl.Project:SetActiveCompo( idx, ) end

---@class TextEntt
---@field pos_ Vec3
---@field scale_x_ number
---@field scale_y_ number
---@field rot_ number
---@field speed number
---@field alpha_ number
---@field font string
---@field text string
---@field separate boolean
movutl.TextEntt = {}

---@param text string
---@param font string
---@return Ref<TextEntt>
function movutl.TextEntt:Create( text, font, ) end

---@return EntityType
function movutl.TextEntt:getType( ) end

---@return PropsInfo
function movutl.TextEntt:getPropsInfo( ) end

---@return Props
function movutl.TextEntt:getProps( ) end

---@param props Props 
---@return nil
function movutl.TextEntt:setProps( props, ) end

---@class TrackLayer
---@field name FixString
---@field active boolean
---@field entts table
movutl.TrackLayer = {}

---@param guid number
---@return Ref<Entity>
function movutl.TrackLayer:find_entt( guid, ) end

---@return string
function movutl.TrackLayer:str( ) end

---@return string
function movutl.TrackLayer:summary( ) end

---@param name string
---@param path string
---@param start number
---@param layer number
---@return boolean
function movutl.add_new_audio_track( name, path, start, layer, )end

---@param name string
---@param type EntityType
---@param start number
---@param end number
---@return boolean
function movutl.add_new_track( name, type, start, end, )end

---@param name string
---@param path string
---@param start number
---@param layer number
---@return Ref<Entity>
function movutl.add_new_video_track( name, path, start, layer, )end

---@return nil
function movutl.clear_selected_entts( )end

---@param time number
---@return nil
function movutl.cv_waitkey( time, )end

---@param path string
---@param type EntityType
---@return InputPluginTable 
function movutl.get_compatible_plugin( path, type, )end

---@return table
function movutl.get_selected_entts( )end

---@return nil
function movutl.init( )end

---@return nil
function movutl.new_project( )end

---@param path string
---@return nil
function movutl.open_project( path, )end

---@return nil
function movutl.save_project( )end

---@param path string
---@return nil
function movutl.save_project_as( path, )end

---@param entt Ref<Entity> 
---@return nil
function movutl.select_entt( entt, )end

---@param entts table
---@return nil
function movutl.select_entts( entts, )end

---@return boolean
function movutl.should_terminate( )end

---@return nil
function movutl.terminate( )end

---@return nil
function movutl.update( )end

   return movutl