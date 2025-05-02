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

---@class BlendType
---@field Blend_Alpha number
---@field Blend_Add number
---@field Blend_Sub number
---@field Blend_Mul number
---@field Blend_Div number
---@field Blend_Screen number
---@field Blend_Overlay number
---@field Blend_Darken number
---@field Blend_Lighten number
---@field Blend_HardLight number
movutl.BlendType = {}

---@class Composition::Flag
---@field setting_dialog number
---@field frame_alpha number
---@field fast_preview number
---@field preprocessing number
---@field hide_output_gui number
---@field nesting number
---@field invert_field_order number
---@field invert_interlace number
movutl.Composition::Flag = {}

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
movutl.Composition.guid = 0
movutl.Composition.name = "Main"
movutl.Composition.flag = nil
movutl.Composition.frame_final = nil
movutl.Composition.frame_edit = nil
movutl.Composition.frame_temp = nil
movutl.Composition.framerate_nu = 30
movutl.Composition.framerate_de = 1
movutl.Composition.fstart = 0
movutl.Composition.fend = 200
movutl.Composition.frame = 0
movutl.Composition.audio_n = 0
movutl.Composition.audio_ch = 0
movutl.Composition.layers = {}

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

---@return number
function movutl.Composition:insertable_layer_index( ) end

---@param entt Ref<Entity>
---@param layer number
---@return nil
function movutl.Composition:insert_entity( entt, layer, ) end

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
movutl.EntityInfo.flag = EntityType_Movie
movutl.EntityInfo.framerate = 23.98
movutl.EntityInfo.nframes = 0
movutl.EntityInfo.format = ImageFormatRGB
movutl.EntityInfo.width = 0
movutl.EntityInfo.height = 0
movutl.EntityInfo.audio_n = 0
movutl.EntityInfo.audio_format_size = 0

---@return string
function movutl.EntityInfo:str( ) end

---@class Image
---@field fmt ImageFormat
---@field pos Vec3
---@field scale Vec2
---@field rotation number
---@field alpha number
---@field path string
movutl.Image = {}
movutl.Image.fmt = ImageFormatRGBA
movutl.Image.pos = Vec3()
movutl.Image.scale = Vec2 ( 1.0 , 1.0 )
movutl.Image.rotation = 0.0
movutl.Image.alpha = 1.0
movutl.Image.path = ""

---@return size_t
function movutl.Image:size( ) end

---@return number
function movutl.Image:width( ) end

---@return number
function movutl.Image:height( ) end

---@return nil
function movutl.Image:reset( ) end

---@param v number
---@return nil
function movutl.Image:fill( v, ) end

---@return Vec4b 
function movutl.Image:data( ) end

---@return nil
function movutl.Image:dirty( ) end

---@return number
function movutl.Image:get_dirty( ) end

---@return number
function movutl.Image:channels( ) end

---@param name string
---@return nil
function movutl.Image:imshow( name, ) end

---@return EntityType
function movutl.Image:getType( ) end

---@return PropsInfo
function movutl.Image:getPropsInfo( ) end

---@return Props
function movutl.Image:getProps( ) end

---@param props Props 
---@return nil
function movutl.Image:setProps( props, ) end

---@class ImageRGBA
---@field width number
---@field height number
---@field dirty_ number
---@field alpha boolean
movutl.ImageRGBA = {}
movutl.ImageRGBA.width = 0
movutl.ImageRGBA.height = 0
movutl.ImageRGBA.dirty_ = 1
movutl.ImageRGBA.alpha = true

---@return nil
function movutl.ImageRGBA:dirty( ) end

---@return Vec4b 
function movutl.ImageRGBA:data( ) end

---@param x size_t
---@param y size_t
---@param rgb Vec3b 
---@return nil
function movutl.ImageRGBA:set_rgb( x, y, rgb, ) end

---@param x size_t
---@param y size_t
---@param rgba Vec4b 
---@return nil
function movutl.ImageRGBA:set_rgba( x, y, rgba, ) end

---@return size_t
function movutl.ImageRGBA:size( ) end

---@return size_t
function movutl.ImageRGBA:size_in_bytes( ) end

---@return nil
function movutl.ImageRGBA:reset( ) end

---@param v number
---@return nil
function movutl.ImageRGBA:fill( v, ) end

---@param x size_t
---@param y size_t
---@return Vec4b
function movutl.ImageRGBA:rgba( x, y, ) end

---@param name string
---@return nil
function movutl.ImageRGBA:imshow( name, ) end

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
movutl.Movie.pos = Vec3 ( 0 , 0 , 0 )
movutl.Movie.scale = Vec2 ( 100 , 100 )
movutl.Movie.rotation = 0
movutl.Movie.start_frame_ = 0
movutl.Movie.speed = 100.0
movutl.Movie.alpha_ = 255
movutl.Movie.loop_ = false
movutl.Movie.with_alpha_ = false
movutl.Movie.path_ = ""

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
movutl.Project.path = ""
movutl.Project.output_path = ""
movutl.Project.entities = {}
movutl.Project.compos_ = {}
movutl.Project.main_comp_idx = 0

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
---@field dirty_ number
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
movutl.TextEntt.dirty_ = 0
movutl.TextEntt.pos_ = Vec3()
movutl.TextEntt.scale_x_ = 1.0
movutl.TextEntt.scale_y_ = 1.0
movutl.TextEntt.rot_ = 0
movutl.TextEntt.speed = 100.0
movutl.TextEntt.alpha_ = 255
movutl.TextEntt.font = ""
movutl.TextEntt.text = ""
movutl.TextEntt.separate = false

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
movutl.TrackLayer.name = "Layer"
movutl.TrackLayer.active = true
movutl.TrackLayer.entts = {}

---@param guid number
---@return Ref<Entity>
function movutl.TrackLayer:find_entt( guid, ) end

---@return string
function movutl.TrackLayer:str( ) end

---@return string
function movutl.TrackLayer:summary( ) end

---@class TrackObject
---@field fstart number
---@field fend number
---@field anchor Vec2
---@field blend_ BlendType
---@field active_ boolean
---@field solo_ boolean
---@field clipping_up boolean
---@field camera_ctrl boolean
movutl.TrackObject = {}
movutl.TrackObject.fstart = - 1
movutl.TrackObject.fend = - 1
movutl.TrackObject.anchor = Vec2()
movutl.TrackObject.blend_ = Blend_Alpha
movutl.TrackObject.active_ = true
movutl.TrackObject.solo_ = false
movutl.TrackObject.clipping_up = false
movutl.TrackObject.camera_ctrl = false

---@param frame number
---@return boolean
function movutl.TrackObject:visible( frame, ) end

---@return PropsInfo
function movutl.TrackObject:getPropsInfo( ) end

---@return Props
function movutl.TrackObject:getProps( ) end

---@param props Props 
---@return nil
function movutl.TrackObject:setProps( props, ) end

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

---@param name string
---@return nil
function movutl.apply_imgui_style( name, )end

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

---@param name string
---@param style ImGuiStyle 
---@return nil
function movutl.register_imgui_style( name, style, )end

---@param name string
---@return nil
function movutl.remove_imgui_style( name, )end

---@return nil
function movutl.render_main_menu_bar( )end

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