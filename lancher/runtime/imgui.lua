-- movutl pygen auto generated bindings
--@meta
--@class imgui
local imgui = {}

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

---@class ImGuiBackendFlags_
---@field ImGuiBackendFlags_None number
---@field ImGuiBackendFlags_HasGamepad number
---@field ImGuiBackendFlags_HasMouseCursors number
---@field ImGuiBackendFlags_HasSetMousePos number
---@field ImGuiBackendFlags_RendererHasVtxOffset number
---@field ImGuiBackendFlags_PlatformHasViewports number
---@field ImGuiBackendFlags_HasMouseHoveredViewport number
---@field ImGuiBackendFlags_RendererHasViewports number
imgui.ImGuiBackendFlags_ = {}

---@class ImGuiButtonFlags_
---@field ImGuiButtonFlags_None number
---@field ImGuiButtonFlags_MouseButtonLeft number
---@field ImGuiButtonFlags_MouseButtonRight number
---@field ImGuiButtonFlags_MouseButtonMiddle number
---@field ImGuiButtonFlags_MouseButtonMask_ number
---@field ImGuiButtonFlags_EnableNav number
imgui.ImGuiButtonFlags_ = {}

---@class ImGuiCol_
---@field ImGuiCol_Text number
---@field ImGuiCol_TextDisabled number
---@field ImGuiCol_WindowBg number
---@field ImGuiCol_ChildBg number
---@field ImGuiCol_PopupBg number
---@field ImGuiCol_Border number
---@field ImGuiCol_BorderShadow number
---@field ImGuiCol_FrameBg number
---@field ImGuiCol_FrameBgHovered number
---@field ImGuiCol_FrameBgActive number
---@field ImGuiCol_TitleBg number
---@field ImGuiCol_TitleBgActive number
---@field ImGuiCol_TitleBgCollapsed number
---@field ImGuiCol_MenuBarBg number
---@field ImGuiCol_ScrollbarBg number
---@field ImGuiCol_ScrollbarGrab number
---@field ImGuiCol_ScrollbarGrabHovered number
---@field ImGuiCol_ScrollbarGrabActive number
---@field ImGuiCol_CheckMark number
---@field ImGuiCol_SliderGrab number
---@field ImGuiCol_SliderGrabActive number
---@field ImGuiCol_Button number
---@field ImGuiCol_ButtonHovered number
---@field ImGuiCol_ButtonActive number
---@field ImGuiCol_Header number
---@field ImGuiCol_HeaderHovered number
---@field ImGuiCol_HeaderActive number
---@field ImGuiCol_Separator number
---@field ImGuiCol_SeparatorHovered number
---@field ImGuiCol_SeparatorActive number
---@field ImGuiCol_ResizeGrip number
---@field ImGuiCol_ResizeGripHovered number
---@field ImGuiCol_ResizeGripActive number
---@field ImGuiCol_TabHovered number
---@field ImGuiCol_Tab number
---@field ImGuiCol_TabSelected number
---@field ImGuiCol_TabSelectedOverline number
---@field ImGuiCol_TabDimmed number
---@field ImGuiCol_TabDimmedSelected number
---@field ImGuiCol_TabDimmedSelectedOverline number
---@field ImGuiCol_DockingPreview number
---@field ImGuiCol_DockingEmptyBg number
---@field ImGuiCol_PlotLines number
---@field ImGuiCol_PlotLinesHovered number
---@field ImGuiCol_PlotHistogram number
---@field ImGuiCol_PlotHistogramHovered number
---@field ImGuiCol_TableHeaderBg number
---@field ImGuiCol_TableBorderStrong number
---@field ImGuiCol_TableBorderLight number
---@field ImGuiCol_TableRowBg number
---@field ImGuiCol_TableRowBgAlt number
---@field ImGuiCol_TextLink number
---@field ImGuiCol_TextSelectedBg number
---@field ImGuiCol_DragDropTarget number
---@field ImGuiCol_NavCursor number
---@field ImGuiCol_NavWindowingHighlight number
---@field ImGuiCol_NavWindowingDimBg number
---@field ImGuiCol_ModalWindowDimBg number
---@field ImGuiCol_COUNT number
imgui.ImGuiCol_ = {}

---@class ImGuiColorEditFlags_
---@field ImGuiColorEditFlags_None number
---@field ImGuiColorEditFlags_NoAlpha number
---@field ImGuiColorEditFlags_NoPicker number
---@field ImGuiColorEditFlags_NoOptions number
---@field ImGuiColorEditFlags_NoSmallPreview number
---@field ImGuiColorEditFlags_NoInputs number
---@field ImGuiColorEditFlags_NoTooltip number
---@field ImGuiColorEditFlags_NoLabel number
---@field ImGuiColorEditFlags_NoSidePreview number
---@field ImGuiColorEditFlags_NoDragDrop number
---@field ImGuiColorEditFlags_NoBorder number
---@field ImGuiColorEditFlags_AlphaBar number
---@field ImGuiColorEditFlags_AlphaPreview number
---@field ImGuiColorEditFlags_AlphaPreviewHalf number
---@field ImGuiColorEditFlags_HDR number
---@field ImGuiColorEditFlags_DisplayRGB number
---@field ImGuiColorEditFlags_DisplayHSV number
---@field ImGuiColorEditFlags_DisplayHex number
---@field ImGuiColorEditFlags_Uint8 number
---@field ImGuiColorEditFlags_Float number
---@field ImGuiColorEditFlags_PickerHueBar number
---@field ImGuiColorEditFlags_PickerHueWheel number
---@field ImGuiColorEditFlags_InputRGB number
---@field ImGuiColorEditFlags_InputHSV number
---@field ImGuiColorEditFlags_DefaultOptions_ number
---@field ImGuiColorEditFlags_DisplayMask_ number
---@field ImGuiColorEditFlags_DataTypeMask_ number
---@field ImGuiColorEditFlags_PickerMask_ number
---@field ImGuiColorEditFlags_InputMask_ number
imgui.ImGuiColorEditFlags_ = {}

---@class ImGuiComboFlags_
---@field ImGuiComboFlags_None number
---@field ImGuiComboFlags_PopupAlignLeft number
---@field ImGuiComboFlags_HeightSmall number
---@field ImGuiComboFlags_HeightRegular number
---@field ImGuiComboFlags_HeightLarge number
---@field ImGuiComboFlags_HeightLargest number
---@field ImGuiComboFlags_NoArrowButton number
---@field ImGuiComboFlags_NoPreview number
---@field ImGuiComboFlags_WidthFitPreview number
---@field ImGuiComboFlags_HeightMask_ number
imgui.ImGuiComboFlags_ = {}

---@class ImGuiCond_
---@field ImGuiCond_None number
---@field ImGuiCond_Always number
---@field ImGuiCond_Once number
---@field ImGuiCond_FirstUseEver number
---@field ImGuiCond_Appearing number
imgui.ImGuiCond_ = {}

---@class ImGuiDataType_
---@field ImGuiDataType_S8 number
---@field ImGuiDataType_U8 number
---@field ImGuiDataType_S16 number
---@field ImGuiDataType_U16 number
---@field ImGuiDataType_S32 number
---@field ImGuiDataType_U32 number
---@field ImGuiDataType_S64 number
---@field ImGuiDataType_U64 number
---@field ImGuiDataType_Float number
---@field ImGuiDataType_Double number
---@field ImGuiDataType_Bool number
---@field ImGuiDataType_String number
---@field ImGuiDataType_COUNT number
imgui.ImGuiDataType_ = {}

---@class ImGuiDir
---@field ImGuiDir_None number
---@field ImGuiDir_Left number
---@field ImGuiDir_Right number
---@field ImGuiDir_Up number
---@field ImGuiDir_Down number
---@field ImGuiDir_COUNT number
imgui.ImGuiDir = {}

---@class ImGuiFocusedFlags_
---@field ImGuiFocusedFlags_None number
---@field ImGuiFocusedFlags_ChildWindows number
---@field ImGuiFocusedFlags_RootWindow number
---@field ImGuiFocusedFlags_AnyWindow number
---@field ImGuiFocusedFlags_NoPopupHierarchy number
---@field ImGuiFocusedFlags_DockHierarchy number
---@field ImGuiFocusedFlags_RootAndChildWindows number
imgui.ImGuiFocusedFlags_ = {}

---@class ImGuiHoveredFlags_
---@field ImGuiHoveredFlags_None number
---@field ImGuiHoveredFlags_ChildWindows number
---@field ImGuiHoveredFlags_RootWindow number
---@field ImGuiHoveredFlags_AnyWindow number
---@field ImGuiHoveredFlags_NoPopupHierarchy number
---@field ImGuiHoveredFlags_DockHierarchy number
---@field ImGuiHoveredFlags_AllowWhenBlockedByPopup number
---@field ImGuiHoveredFlags_AllowWhenBlockedByActiveItem number
---@field ImGuiHoveredFlags_AllowWhenOverlappedByItem number
---@field ImGuiHoveredFlags_AllowWhenOverlappedByWindow number
---@field ImGuiHoveredFlags_AllowWhenDisabled number
---@field ImGuiHoveredFlags_NoNavOverride number
---@field ImGuiHoveredFlags_AllowWhenOverlapped number
---@field ImGuiHoveredFlags_RectOnly number
---@field ImGuiHoveredFlags_RootAndChildWindows number
---@field ImGuiHoveredFlags_ForTooltip number
---@field ImGuiHoveredFlags_Stationary number
---@field ImGuiHoveredFlags_DelayNone number
---@field ImGuiHoveredFlags_DelayShort number
---@field ImGuiHoveredFlags_DelayNormal number
---@field ImGuiHoveredFlags_NoSharedDelay number
imgui.ImGuiHoveredFlags_ = {}

---@class ImGuiInputFlags_
---@field ImGuiInputFlags_None number
---@field ImGuiInputFlags_Repeat number
---@field ImGuiInputFlags_RouteActive number
---@field ImGuiInputFlags_RouteFocused number
---@field ImGuiInputFlags_RouteGlobal number
---@field ImGuiInputFlags_RouteAlways number
---@field ImGuiInputFlags_RouteOverFocused number
---@field ImGuiInputFlags_RouteOverActive number
---@field ImGuiInputFlags_RouteUnlessBgFocused number
---@field ImGuiInputFlags_RouteFromRootWindow number
---@field ImGuiInputFlags_Tooltip number
imgui.ImGuiInputFlags_ = {}

---@class ImGuiInputTextFlags_
---@field ImGuiInputTextFlags_None number
---@field ImGuiInputTextFlags_CharsDecimal number
---@field ImGuiInputTextFlags_CharsHexadecimal number
---@field ImGuiInputTextFlags_CharsScientific number
---@field ImGuiInputTextFlags_CharsUppercase number
---@field ImGuiInputTextFlags_CharsNoBlank number
---@field ImGuiInputTextFlags_AllowTabInput number
---@field ImGuiInputTextFlags_EnterReturnsTrue number
---@field ImGuiInputTextFlags_EscapeClearsAll number
---@field ImGuiInputTextFlags_CtrlEnterForNewLine number
---@field ImGuiInputTextFlags_ReadOnly number
---@field ImGuiInputTextFlags_Password number
---@field ImGuiInputTextFlags_AlwaysOverwrite number
---@field ImGuiInputTextFlags_AutoSelectAll number
---@field ImGuiInputTextFlags_ParseEmptyRefVal number
---@field ImGuiInputTextFlags_DisplayEmptyRefVal number
---@field ImGuiInputTextFlags_NoHorizontalScroll number
---@field ImGuiInputTextFlags_NoUndoRedo number
---@field ImGuiInputTextFlags_ElideLeft number
---@field ImGuiInputTextFlags_CallbackCompletion number
---@field ImGuiInputTextFlags_CallbackHistory number
---@field ImGuiInputTextFlags_CallbackAlways number
---@field ImGuiInputTextFlags_CallbackCharFilter number
---@field ImGuiInputTextFlags_CallbackResize number
---@field ImGuiInputTextFlags_CallbackEdit number
imgui.ImGuiInputTextFlags_ = {}

---@class ImGuiItemFlags_
---@field ImGuiItemFlags_None number
---@field ImGuiItemFlags_NoTabStop number
---@field ImGuiItemFlags_NoNav number
---@field ImGuiItemFlags_NoNavDefaultFocus number
---@field ImGuiItemFlags_ButtonRepeat number
---@field ImGuiItemFlags_AutoClosePopups number
---@field ImGuiItemFlags_AllowDuplicateId number
imgui.ImGuiItemFlags_ = {}

---@class ImGuiMouseButton_
---@field ImGuiMouseButton_Left number
---@field ImGuiMouseButton_Right number
---@field ImGuiMouseButton_Middle number
---@field ImGuiMouseButton_COUNT number
imgui.ImGuiMouseButton_ = {}

---@class ImGuiMouseCursor_
---@field ImGuiMouseCursor_None number
---@field ImGuiMouseCursor_Arrow number
---@field ImGuiMouseCursor_TextInput number
---@field ImGuiMouseCursor_ResizeAll number
---@field ImGuiMouseCursor_ResizeNS number
---@field ImGuiMouseCursor_ResizeEW number
---@field ImGuiMouseCursor_ResizeNESW number
---@field ImGuiMouseCursor_ResizeNWSE number
---@field ImGuiMouseCursor_Hand number
---@field ImGuiMouseCursor_NotAllowed number
---@field ImGuiMouseCursor_COUNT number
imgui.ImGuiMouseCursor_ = {}

---@class ImGuiMouseSource
---@field ImGuiMouseSource_Mouse number
---@field ImGuiMouseSource_TouchScreen number
---@field ImGuiMouseSource_Pen number
---@field ImGuiMouseSource_COUNT number
imgui.ImGuiMouseSource = {}

---@class ImGuiMultiSelectFlags_
---@field ImGuiMultiSelectFlags_None number
---@field ImGuiMultiSelectFlags_SingleSelect number
---@field ImGuiMultiSelectFlags_NoSelectAll number
---@field ImGuiMultiSelectFlags_NoRangeSelect number
---@field ImGuiMultiSelectFlags_NoAutoSelect number
---@field ImGuiMultiSelectFlags_NoAutoClear number
---@field ImGuiMultiSelectFlags_NoAutoClearOnReselect number
---@field ImGuiMultiSelectFlags_BoxSelect1d number
---@field ImGuiMultiSelectFlags_BoxSelect2d number
---@field ImGuiMultiSelectFlags_BoxSelectNoScroll number
---@field ImGuiMultiSelectFlags_ClearOnEscape number
---@field ImGuiMultiSelectFlags_ClearOnClickVoid number
---@field ImGuiMultiSelectFlags_ScopeWindow number
---@field ImGuiMultiSelectFlags_ScopeRect number
---@field ImGuiMultiSelectFlags_SelectOnClick number
---@field ImGuiMultiSelectFlags_SelectOnClickRelease number
---@field ImGuiMultiSelectFlags_NavWrapX number
imgui.ImGuiMultiSelectFlags_ = {}

---@class ImGuiPopupFlags_
---@field ImGuiPopupFlags_None number
---@field ImGuiPopupFlags_MouseButtonLeft number
---@field ImGuiPopupFlags_MouseButtonRight number
---@field ImGuiPopupFlags_MouseButtonMiddle number
---@field ImGuiPopupFlags_MouseButtonMask_ number
---@field ImGuiPopupFlags_MouseButtonDefault_ number
---@field ImGuiPopupFlags_NoReopen number
---@field ImGuiPopupFlags_NoOpenOverExistingPopup number
---@field ImGuiPopupFlags_NoOpenOverItems number
---@field ImGuiPopupFlags_AnyPopupId number
---@field ImGuiPopupFlags_AnyPopupLevel number
---@field ImGuiPopupFlags_AnyPopup number
imgui.ImGuiPopupFlags_ = {}

---@class ImGuiSelectionRequestType
---@field ImGuiSelectionRequestType_None number
---@field ImGuiSelectionRequestType_SetAll number
---@field ImGuiSelectionRequestType_SetRange number
imgui.ImGuiSelectionRequestType = {}

---@class ImGuiSliderFlags_
---@field ImGuiSliderFlags_None number
---@field ImGuiSliderFlags_Logarithmic number
---@field ImGuiSliderFlags_NoRoundToFormat number
---@field ImGuiSliderFlags_NoInput number
---@field ImGuiSliderFlags_WrapAround number
---@field ImGuiSliderFlags_ClampOnInput number
---@field ImGuiSliderFlags_ClampZeroRange number
---@field ImGuiSliderFlags_NoSpeedTweaks number
---@field ImGuiSliderFlags_AlwaysClamp number
---@field ImGuiSliderFlags_InvalidMask_ number
imgui.ImGuiSliderFlags_ = {}

---@class ImGuiSortDirection
---@field ImGuiSortDirection_None number
---@field ImGuiSortDirection_Ascending number
---@field ImGuiSortDirection_Descending number
imgui.ImGuiSortDirection = {}

---@class ImGuiStyleVar_
---@field ImGuiStyleVar_Alpha number
---@field ImGuiStyleVar_DisabledAlpha number
---@field ImGuiStyleVar_WindowPadding number
---@field ImGuiStyleVar_WindowRounding number
---@field ImGuiStyleVar_WindowBorderSize number
---@field ImGuiStyleVar_WindowMinSize number
---@field ImGuiStyleVar_WindowTitleAlign number
---@field ImGuiStyleVar_ChildRounding number
---@field ImGuiStyleVar_ChildBorderSize number
---@field ImGuiStyleVar_PopupRounding number
---@field ImGuiStyleVar_PopupBorderSize number
---@field ImGuiStyleVar_FramePadding number
---@field ImGuiStyleVar_FrameRounding number
---@field ImGuiStyleVar_FrameBorderSize number
---@field ImGuiStyleVar_ItemSpacing number
---@field ImGuiStyleVar_ItemInnerSpacing number
---@field ImGuiStyleVar_IndentSpacing number
---@field ImGuiStyleVar_CellPadding number
---@field ImGuiStyleVar_ScrollbarSize number
---@field ImGuiStyleVar_ScrollbarRounding number
---@field ImGuiStyleVar_GrabMinSize number
---@field ImGuiStyleVar_GrabRounding number
---@field ImGuiStyleVar_TabRounding number
---@field ImGuiStyleVar_TabBorderSize number
---@field ImGuiStyleVar_TabBarBorderSize number
---@field ImGuiStyleVar_TabBarOverlineSize number
---@field ImGuiStyleVar_TableAngledHeadersAngle number
---@field ImGuiStyleVar_TableAngledHeadersTextAlign number
---@field ImGuiStyleVar_ButtonTextAlign number
---@field ImGuiStyleVar_SelectableTextAlign number
---@field ImGuiStyleVar_SeparatorTextBorderSize number
---@field ImGuiStyleVar_SeparatorTextAlign number
---@field ImGuiStyleVar_SeparatorTextPadding number
---@field ImGuiStyleVar_DockingSeparatorSize number
---@field ImGuiStyleVar_COUNT number
imgui.ImGuiStyleVar_ = {}

---@class ImGuiTabBarFlags_
---@field ImGuiTabBarFlags_None number
---@field ImGuiTabBarFlags_Reorderable number
---@field ImGuiTabBarFlags_AutoSelectNewTabs number
---@field ImGuiTabBarFlags_TabListPopupButton number
---@field ImGuiTabBarFlags_NoCloseWithMiddleMouseButton number
---@field ImGuiTabBarFlags_NoTabListScrollingButtons number
---@field ImGuiTabBarFlags_NoTooltip number
---@field ImGuiTabBarFlags_DrawSelectedOverline number
---@field ImGuiTabBarFlags_FittingPolicyResizeDown number
---@field ImGuiTabBarFlags_FittingPolicyScroll number
---@field ImGuiTabBarFlags_FittingPolicyMask_ number
---@field ImGuiTabBarFlags_FittingPolicyDefault_ number
imgui.ImGuiTabBarFlags_ = {}

---@class ImGuiTabItemFlags_
---@field ImGuiTabItemFlags_None number
---@field ImGuiTabItemFlags_UnsavedDocument number
---@field ImGuiTabItemFlags_SetSelected number
---@field ImGuiTabItemFlags_NoCloseWithMiddleMouseButton number
---@field ImGuiTabItemFlags_NoPushId number
---@field ImGuiTabItemFlags_NoTooltip number
---@field ImGuiTabItemFlags_NoReorder number
---@field ImGuiTabItemFlags_Leading number
---@field ImGuiTabItemFlags_Trailing number
---@field ImGuiTabItemFlags_NoAssumedClosure number
imgui.ImGuiTabItemFlags_ = {}

---@class ImGuiTableBgTarget_
---@field ImGuiTableBgTarget_None number
---@field ImGuiTableBgTarget_RowBg0 number
---@field ImGuiTableBgTarget_RowBg1 number
---@field ImGuiTableBgTarget_CellBg number
imgui.ImGuiTableBgTarget_ = {}

---@class ImGuiTableColumnFlags_
---@field ImGuiTableColumnFlags_None number
---@field ImGuiTableColumnFlags_Disabled number
---@field ImGuiTableColumnFlags_DefaultHide number
---@field ImGuiTableColumnFlags_DefaultSort number
---@field ImGuiTableColumnFlags_WidthStretch number
---@field ImGuiTableColumnFlags_WidthFixed number
---@field ImGuiTableColumnFlags_NoResize number
---@field ImGuiTableColumnFlags_NoReorder number
---@field ImGuiTableColumnFlags_NoHide number
---@field ImGuiTableColumnFlags_NoClip number
---@field ImGuiTableColumnFlags_NoSort number
---@field ImGuiTableColumnFlags_NoSortAscending number
---@field ImGuiTableColumnFlags_NoSortDescending number
---@field ImGuiTableColumnFlags_NoHeaderLabel number
---@field ImGuiTableColumnFlags_NoHeaderWidth number
---@field ImGuiTableColumnFlags_PreferSortAscending number
---@field ImGuiTableColumnFlags_PreferSortDescending number
---@field ImGuiTableColumnFlags_IndentEnable number
---@field ImGuiTableColumnFlags_IndentDisable number
---@field ImGuiTableColumnFlags_AngledHeader number
---@field ImGuiTableColumnFlags_IsEnabled number
---@field ImGuiTableColumnFlags_IsVisible number
---@field ImGuiTableColumnFlags_IsSorted number
---@field ImGuiTableColumnFlags_IsHovered number
---@field ImGuiTableColumnFlags_WidthMask_ number
---@field ImGuiTableColumnFlags_IndentMask_ number
---@field ImGuiTableColumnFlags_StatusMask_ number
---@field ImGuiTableColumnFlags_NoDirectResize_ number
imgui.ImGuiTableColumnFlags_ = {}

---@class ImGuiTableFlags_
---@field ImGuiTableFlags_None number
---@field ImGuiTableFlags_Resizable number
---@field ImGuiTableFlags_Reorderable number
---@field ImGuiTableFlags_Hideable number
---@field ImGuiTableFlags_Sortable number
---@field ImGuiTableFlags_NoSavedSettings number
---@field ImGuiTableFlags_ContextMenuInBody number
---@field ImGuiTableFlags_RowBg number
---@field ImGuiTableFlags_BordersInnerH number
---@field ImGuiTableFlags_BordersOuterH number
---@field ImGuiTableFlags_BordersInnerV number
---@field ImGuiTableFlags_BordersOuterV number
---@field ImGuiTableFlags_BordersH number
---@field ImGuiTableFlags_BordersV number
---@field ImGuiTableFlags_BordersInner number
---@field ImGuiTableFlags_BordersOuter number
---@field ImGuiTableFlags_Borders number
---@field ImGuiTableFlags_NoBordersInBody number
---@field ImGuiTableFlags_NoBordersInBodyUntilResize number
---@field ImGuiTableFlags_SizingFixedFit number
---@field ImGuiTableFlags_SizingFixedSame number
---@field ImGuiTableFlags_SizingStretchProp number
---@field ImGuiTableFlags_SizingStretchSame number
---@field ImGuiTableFlags_NoHostExtendX number
---@field ImGuiTableFlags_NoHostExtendY number
---@field ImGuiTableFlags_NoKeepColumnsVisible number
---@field ImGuiTableFlags_PreciseWidths number
---@field ImGuiTableFlags_NoClip number
---@field ImGuiTableFlags_PadOuterX number
---@field ImGuiTableFlags_NoPadOuterX number
---@field ImGuiTableFlags_NoPadInnerX number
---@field ImGuiTableFlags_ScrollX number
---@field ImGuiTableFlags_ScrollY number
---@field ImGuiTableFlags_SortMulti number
---@field ImGuiTableFlags_SortTristate number
---@field ImGuiTableFlags_HighlightHoveredColumn number
---@field ImGuiTableFlags_SizingMask_ number
imgui.ImGuiTableFlags_ = {}

---@class ImGuiTableRowFlags_
---@field ImGuiTableRowFlags_None number
---@field ImGuiTableRowFlags_Headers number
imgui.ImGuiTableRowFlags_ = {}

---@class ImGuiViewportFlags_
---@field ImGuiViewportFlags_None number
---@field ImGuiViewportFlags_IsPlatformWindow number
---@field ImGuiViewportFlags_IsPlatformMonitor number
---@field ImGuiViewportFlags_OwnedByApp number
---@field ImGuiViewportFlags_NoDecoration number
---@field ImGuiViewportFlags_NoTaskBarIcon number
---@field ImGuiViewportFlags_NoFocusOnAppearing number
---@field ImGuiViewportFlags_NoFocusOnClick number
---@field ImGuiViewportFlags_NoInputs number
---@field ImGuiViewportFlags_NoRendererClear number
---@field ImGuiViewportFlags_NoAutoMerge number
---@field ImGuiViewportFlags_TopMost number
---@field ImGuiViewportFlags_CanHostOtherWindows number
---@field ImGuiViewportFlags_IsMinimized number
---@field ImGuiViewportFlags_IsFocused number
imgui.ImGuiViewportFlags_ = {}

---@class ImDrawVert
---@field pos ImVec2
---@field uv ImVec2
---@field col ImU32
imgui.ImDrawVert = {}

---@class ImGuiIO
---@field ConfigFlags ImGuiConfigFlags
---@field BackendFlags ImGuiBackendFlags
---@field DisplaySize ImVec2
---@field DeltaTime number
---@field IniSavingRate number
---@field IniFilename string
---@field LogFilename string
---@field FontGlobalScale number
---@field DisplayFramebufferScale ImVec2
---@field ConfigNavSwapGamepadButtons boolean
---@field ConfigNavMoveSetMousePos boolean
---@field ConfigNavCaptureKeyboard boolean
---@field ConfigNavEscapeClearFocusItem boolean
---@field ConfigNavEscapeClearFocusWindow boolean
---@field ConfigNavCursorVisibleAuto boolean
---@field ConfigNavCursorVisibleAlways boolean
---@field ConfigDockingNoSplit boolean
---@field ConfigDockingWithShift boolean
---@field ConfigViewportsNoAutoMerge boolean
---@field ConfigViewportsNoTaskBarIcon boolean
---@field ConfigViewportsNoDecoration boolean
---@field ConfigViewportsNoDefaultParent boolean
---@field MouseDrawCursor boolean
---@field ConfigMacOSXBehaviors boolean
---@field ConfigInputTrickleEventQueue boolean
---@field ConfigInputTextCursorBlink boolean
---@field ConfigWindowsResizeFromEdges boolean
---@field ConfigWindowsMoveFromTitleBarOnly boolean
---@field ConfigScrollbarScrollByPage boolean
---@field ConfigMemoryCompactTimer number
---@field MouseDoubleClickTime number
---@field MouseDoubleClickMaxDist number
---@field MouseDragThreshold number
---@field KeyRepeatDelay number
---@field KeyRepeatRate number
---@field ConfigErrorRecovery boolean
---@field ConfigErrorRecoveryEnableAssert boolean
---@field ConfigErrorRecoveryEnableDebugLog boolean
---@field ConfigErrorRecoveryEnableTooltip boolean
---@field ConfigDebugIsDebuggerPresent boolean
---@field ConfigDebugHighlightIdConflicts boolean
---@field ConfigDebugBeginReturnValueOnce boolean
---@field ConfigDebugBeginReturnValueLoop boolean
---@field ConfigDebugIgnoreFocusLoss boolean
---@field ConfigDebugIniSettings boolean
---@field BackendPlatformName string
---@field BackendRendererName string
---@field WantCaptureMouse boolean
---@field WantCaptureKeyboard boolean
---@field WantTextInput boolean
---@field WantSetMousePos boolean
---@field WantSaveIniSettings boolean
---@field NavActive boolean
---@field NavVisible boolean
---@field Framerate number
---@field MetricsRenderVertices number
---@field MetricsRenderIndices number
---@field MetricsRenderWindows number
---@field MetricsActiveWindows number
---@field MouseDelta ImVec2
---@field MousePos ImVec2
---@field MouseWheel number
---@field MouseWheelH number
---@field MouseSource ImGuiMouseSource
---@field MouseHoveredViewport ImGuiID
---@field KeyCtrl boolean
---@field KeyShift boolean
---@field KeyAlt boolean
---@field KeySuper boolean
---@field KeyMods ImGuiKeyChord
---@field WantCaptureMouseUnlessPopupClose boolean
---@field MousePosPrev ImVec2
---@field MouseWheelRequestAxisSwap boolean
---@field MouseCtrlLeftAsRightClick boolean
---@field AppFocusLost boolean
---@field AppAcceptingEvents boolean
---@field InputQueueSurrogate ImWchar16
---@field InputQueueCharacters ImVector<ImWchar>
imgui.ImGuiIO = {}

---@param key ImGuiKey
---@param down boolean
---@return nil
function imgui.ImGuiIO:AddKeyEvent( key, down, ) end

---@param key ImGuiKey
---@param down boolean
---@param v number
---@return nil
function imgui.ImGuiIO:AddKeyAnalogEvent( key, down, v, ) end

---@param x number
---@param y number
---@return nil
function imgui.ImGuiIO:AddMousePosEvent( x, y, ) end

---@param button number
---@param down boolean
---@return nil
function imgui.ImGuiIO:AddMouseButtonEvent( button, down, ) end

---@param wheel_x number
---@param wheel_y number
---@return nil
function imgui.ImGuiIO:AddMouseWheelEvent( wheel_x, wheel_y, ) end

---@param source ImGuiMouseSource
---@return nil
function imgui.ImGuiIO:AddMouseSourceEvent( source, ) end

---@param id ImGuiID
---@return nil
function imgui.ImGuiIO:AddMouseViewportEvent( id, ) end

---@param focused boolean
---@return nil
function imgui.ImGuiIO:AddFocusEvent( focused, ) end

---@param c number
---@return nil
function imgui.ImGuiIO:AddInputCharacter( c, ) end

---@param c ImWchar16
---@return nil
function imgui.ImGuiIO:AddInputCharacterUTF16( c, ) end

---@param str string
---@return nil
function imgui.ImGuiIO:AddInputCharactersUTF8( str, ) end

---@param key ImGuiKey
---@param native_keycode number
---@param native_scancode number
---@param native_legacy_index number
---@return nil
function imgui.ImGuiIO:SetKeyEventNativeData( key, native_keycode, native_scancode, native_legacy_index, ) end

---@param accepting_events boolean
---@return nil
function imgui.ImGuiIO:SetAppAcceptingEvents( accepting_events, ) end

---@return nil
function imgui.ImGuiIO:ClearEventsQueue( ) end

---@return nil
function imgui.ImGuiIO:ClearInputKeys( ) end

---@return nil
function imgui.ImGuiIO:ClearInputMouse( ) end

---@return nil
function imgui.ImGuiIO:ClearInputCharacters( ) end

---@class ImGuiKeyData
---@field Down boolean
---@field DownDuration number
---@field DownDurationPrev number
---@field AnalogValue number
imgui.ImGuiKeyData = {}

---@class ImGuiSelectionBasicStorage
---@field Size number
---@field PreserveOrder boolean
imgui.ImGuiSelectionBasicStorage = {}

---@param id ImGuiID
---@return boolean
function imgui.ImGuiSelectionBasicStorage:Contains( id, ) end

---@return nil
function imgui.ImGuiSelectionBasicStorage:Clear( ) end

---@param r ImGuiSelectionBasicStorage 
---@return nil
function imgui.ImGuiSelectionBasicStorage:Swap( r, ) end

---@param id ImGuiID
---@param selected boolean
---@return nil
function imgui.ImGuiSelectionBasicStorage:SetItemSelected( id, selected, ) end

---@param idx number
---@return ImGuiID
function imgui.ImGuiSelectionBasicStorage:GetStorageIdFromIndex( idx, ) end

---@class ImGuiSelectionRequest
---@field Type ImGuiSelectionRequestType
---@field Selected boolean
---@field RangeDirection ImS8
---@field RangeFirstItem ImGuiSelectionUserData
---@field RangeLastItem ImGuiSelectionUserData
imgui.ImGuiSelectionRequest = {}

---@class ImGuiSizeCallbackData
---@field Pos ImVec2
---@field CurrentSize ImVec2
---@field DesiredSize ImVec2
imgui.ImGuiSizeCallbackData = {}

---@class ImGuiStorage
---@field Data ImVector<ImGuiStoragePair>
imgui.ImGuiStorage = {}

---@return nil
function imgui.ImGuiStorage:Clear( ) end

---@param key ImGuiID
---@param default_val number
---@return number
function imgui.ImGuiStorage:GetInt( key, default_val, ) end

---@param key ImGuiID
---@param val number
---@return nil
function imgui.ImGuiStorage:SetInt( key, val, ) end

---@param key ImGuiID
---@param default_val boolean
---@return boolean
function imgui.ImGuiStorage:GetBool( key, default_val, ) end

---@param key ImGuiID
---@param val boolean
---@return nil
function imgui.ImGuiStorage:SetBool( key, val, ) end

---@param key ImGuiID
---@param default_val number
---@return number
function imgui.ImGuiStorage:GetFloat( key, default_val, ) end

---@param key ImGuiID
---@param val number
---@return nil
function imgui.ImGuiStorage:SetFloat( key, val, ) end

---@param key ImGuiID
---@return void 
function imgui.ImGuiStorage:GetVoidPtr( key, ) end

---@param key ImGuiID
---@param default_val number
---@return number
function imgui.ImGuiStorage:GetIntRef( key, default_val, ) end

---@param key ImGuiID
---@param default_val boolean
---@return bool 
function imgui.ImGuiStorage:GetBoolRef( key, default_val, ) end

---@param key ImGuiID
---@param default_val number
---@return float 
function imgui.ImGuiStorage:GetFloatRef( key, default_val, ) end

---@return nil
function imgui.ImGuiStorage:BuildSortByKey( ) end

---@param val number
---@return nil
function imgui.ImGuiStorage:SetAllInt( val, ) end

---@class ImGuiTableColumnSortSpecs
---@field ColumnUserID ImGuiID
---@field ColumnIndex ImS16
---@field SortOrder ImS16
---@field SortDirection ImGuiSortDirection
imgui.ImGuiTableColumnSortSpecs = {}

---@class ImGuiTableSortSpecs
---@field SpecsCount number
---@field SpecsDirty boolean
imgui.ImGuiTableSortSpecs = {}

---@class ImGuiViewport
---@field ID ImGuiID
---@field Flags ImGuiViewportFlags
---@field Pos ImVec2
---@field Size ImVec2
---@field WorkPos ImVec2
---@field WorkSize ImVec2
---@field DpiScale number
---@field ParentViewportId ImGuiID
---@field PlatformWindowCreated boolean
---@field PlatformRequestMove boolean
---@field PlatformRequestResize boolean
---@field PlatformRequestClose boolean
imgui.ImGuiViewport = {}

---@return ImVec2
function imgui.ImGuiViewport:GetCenter( ) end

---@return ImVec2
function imgui.ImGuiViewport:GetWorkCenter( ) end

---@class ImGuiWindowClass
---@field ClassId ImGuiID
---@field ParentViewportId ImGuiID
---@field FocusRouteParentWindowId ImGuiID
---@field ViewportFlagsOverrideSet ImGuiViewportFlags
---@field ViewportFlagsOverrideClear ImGuiViewportFlags
---@field DockingAlwaysTabBar boolean
---@field DockingAllowUnclassed boolean
imgui.ImGuiWindowClass = {}

---@param type string
---@param flags ImGuiDragDropFlags
---@return ImGuiPayload 
function imgui.AcceptDragDropPayload( type, flags, )end

---@return nil
function imgui.AlignTextToFramePadding( )end

---@param str_id string
---@param dir ImGuiDir
---@return boolean
function imgui.ArrowButton( str_id, dir, )end

---@param str_id string
---@param size ImVec2 
---@param child_flags ImGuiChildFlags
---@param window_flags ImGuiWindowFlags
---@return boolean
function imgui.BeginChild( str_id, size, child_flags, window_flags, )end

---@param label string
---@param preview_value string
---@param flags ImGuiComboFlags
---@return boolean
function imgui.BeginCombo( label, preview_value, flags, )end

---@param disabled boolean
---@return nil
function imgui.BeginDisabled( disabled, )end

---@param flags ImGuiDragDropFlags
---@return boolean
function imgui.BeginDragDropSource( flags, )end

---@return boolean
function imgui.BeginDragDropTarget( )end

---@return nil
function imgui.BeginGroup( )end

---@return boolean
function imgui.BeginItemTooltip( )end

---@param label string
---@param size ImVec2 
---@return boolean
function imgui.BeginListBox( label, size, )end

---@return boolean
function imgui.BeginMainMenuBar( )end

---@param label string
---@param enabled boolean
---@return boolean
function imgui.BeginMenu( label, enabled, )end

---@return boolean
function imgui.BeginMenuBar( )end

---@param flags ImGuiMultiSelectFlags
---@param selection_size number
---@param items_count number
---@return ImGuiMultiSelectIO 
function imgui.BeginMultiSelect( flags, selection_size, items_count, )end

---@param str_id string
---@param flags ImGuiWindowFlags
---@return boolean
function imgui.BeginPopup( str_id, flags, )end

---@param str_id string
---@param popup_flags ImGuiPopupFlags
---@return boolean
function imgui.BeginPopupContextItem( str_id, popup_flags, )end

---@param str_id string
---@param popup_flags ImGuiPopupFlags
---@return boolean
function imgui.BeginPopupContextVoid( str_id, popup_flags, )end

---@param str_id string
---@param popup_flags ImGuiPopupFlags
---@return boolean
function imgui.BeginPopupContextWindow( str_id, popup_flags, )end

---@param str_id string
---@param flags ImGuiTabBarFlags
---@return boolean
function imgui.BeginTabBar( str_id, flags, )end

---@param str_id string
---@param columns number
---@param flags ImGuiTableFlags
---@param outer_size ImVec2 
---@param inner_width number
---@return boolean
function imgui.BeginTable( str_id, columns, flags, outer_size, inner_width, )end

---@return boolean
function imgui.BeginTooltip( )end

---@return nil
function imgui.Bullet( )end

---@param text string
---@return nil
function imgui.BulletText( text, )end

---@param label string
---@param size ImVec2 
---@return boolean
function imgui.Button( label, size, )end

---@return number
function imgui.CalcItemWidth( )end

---@param text string
---@param text_end string
---@param hide_text_after_double_hash boolean
---@param wrap_width number
---@return ImVec2
function imgui.CalcTextSize( text, text_end, hide_text_after_double_hash, wrap_width, )end

---@return nil
function imgui.CloseCurrentPopup( )end

---@param label string
---@param flags ImGuiTreeNodeFlags
---@return boolean
function imgui.CollapsingHeader( label, flags, )end

---@param desc_id string
---@param col ImVec4 
---@param flags ImGuiColorEditFlags
---@param size ImVec2 
---@return boolean
function imgui.ColorButton( desc_id, col, flags, size, )end

---@param in ImVec4 
---@return ImU32
function imgui.ColorConvertFloat4ToU32( in, )end

---@param h number
---@param s number
---@param v number
---@param out_r float 
---@param out_g float 
---@param out_b float 
---@return nil
function imgui.ColorConvertHSVtoRGB( h, s, v, out_r, out_g, out_b, )end

---@param r number
---@param g number
---@param b number
---@param out_h float 
---@param out_s float 
---@param out_v float 
---@return nil
function imgui.ColorConvertRGBtoHSV( r, g, b, out_h, out_s, out_v, )end

---@param in ImU32
---@return ImVec4
function imgui.ColorConvertU32ToFloat4( in, )end

---@param label string
---@param color Vec3 
---@return nil
function imgui.ColorEdit3( label, color, )end

---@param label string
---@param color Vec4 
---@return nil
function imgui.ColorEdit4( label, color, )end

---@param label string
---@param color Vec3 
---@return nil
function imgui.ColorPicker3( label, color, )end

---@param label string
---@param color Vec4 
---@return nil
function imgui.ColorPicker4( label, color, )end

---@param count number
---@param id string
---@param borders boolean
---@return nil
function imgui.Columns( count, id, borders, )end

---@param version_str string
---@param sz_io size_t
---@param sz_style size_t
---@param sz_vec2 size_t
---@param sz_vec4 size_t
---@param sz_drawvert size_t
---@param sz_drawidx size_t
---@return boolean
function imgui.DebugCheckVersionAndDataLayout( version_str, sz_io, sz_style, sz_vec2, sz_vec4, sz_drawvert, sz_drawidx, )end

---@param idx ImGuiCol
---@return nil
function imgui.DebugFlashStyleColor( idx, )end

---@param text string
---@return nil
function imgui.DebugLog( text, )end

---@return nil
function imgui.DebugStartItemPicker( )end

---@param text string
---@return nil
function imgui.DebugTextEncoding( text, )end

---@return nil
function imgui.DestroyPlatformWindows( )end

---@param label string
---@param v Vec2
---@param speed number
---@param min number
---@param max number
---@return Vec2
function imgui.DragFloat2( label, v, speed, min, max, )end

---@param label string
---@param v Vec3
---@param speed number
---@param min number
---@param max number
---@return Vec3
function imgui.DragFloat3( label, v, speed, min, max, )end

---@param label string
---@param v Vec4
---@param speed number
---@param min number
---@param max number
---@return Vec4
function imgui.DragFloat4( label, v, speed, min, max, )end

---@param label string
---@param v Vec2d
---@param speed number
---@param min number
---@param max number
---@return Vec2d
function imgui.DragInt2( label, v, speed, min, max, )end

---@param label string
---@param v Vec3d
---@param speed number
---@param min number
---@param max number
---@return Vec3d
function imgui.DragInt3( label, v, speed, min, max, )end

---@param label string
---@param v Vec4d
---@param speed number
---@param min number
---@param max number
---@return Vec4d
function imgui.DragInt4( label, v, speed, min, max, )end

---@param size ImVec2 
---@return nil
function imgui.Dummy( size, )end

---@return nil
function imgui.End( )end

---@return nil
function imgui.EndChild( )end

---@return nil
function imgui.EndCombo( )end

---@return nil
function imgui.EndDisabled( )end

---@return nil
function imgui.EndDragDropSource( )end

---@return nil
function imgui.EndDragDropTarget( )end

---@return nil
function imgui.EndFrame( )end

---@return nil
function imgui.EndGroup( )end

---@return nil
function imgui.EndListBox( )end

---@return nil
function imgui.EndMainMenuBar( )end

---@return nil
function imgui.EndMenu( )end

---@return nil
function imgui.EndMenuBar( )end

---@return ImGuiMultiSelectIO 
function imgui.EndMultiSelect( )end

---@return nil
function imgui.EndPopup( )end

---@return nil
function imgui.EndTabBar( )end

---@return nil
function imgui.EndTabItem( )end

---@return nil
function imgui.EndTable( )end

---@return nil
function imgui.EndTooltip( )end

---@param id ImGuiID
---@return ImGuiViewport 
function imgui.FindViewportByID( id, )end

---@return string
function imgui.GetClipboardText( )end

---@param idx ImGuiCol
---@param alpha_mul number
---@return ImU32
function imgui.GetColorU32( idx, alpha_mul, )end

---@param col ImVec4 
---@return ImU32
function imgui.GetColorU32( col, )end

---@return number
function imgui.GetColumnIndex( )end

---@param column_index number
---@return number
function imgui.GetColumnOffset( column_index, )end

---@param column_index number
---@return number
function imgui.GetColumnWidth( column_index, )end

---@return number
function imgui.GetColumnsCount( )end

---@return ImVec2
function imgui.GetContentRegionAvail( )end

---@return ImGuiContext 
function imgui.GetCurrentContext( )end

---@return ImVec2
function imgui.GetCursorPos( )end

---@return number
function imgui.GetCursorPosX( )end

---@return number
function imgui.GetCursorPosY( )end

---@return ImVec2
function imgui.GetCursorScreenPos( )end

---@return ImVec2
function imgui.GetCursorStartPos( )end

---@return ImGuiPayload 
function imgui.GetDragDropPayload( )end

---@return ImDrawData 
function imgui.GetDrawData( )end

---@return ImDrawListSharedData 
function imgui.GetDrawListSharedData( )end

---@return ImFont 
function imgui.GetFont( )end

---@return number
function imgui.GetFontSize( )end

---@return ImVec2
function imgui.GetFontTexUvWhitePixel( )end

---@return number
function imgui.GetFrameCount( )end

---@return number
function imgui.GetFrameHeight( )end

---@return number
function imgui.GetFrameHeightWithSpacing( )end

---@param str_id string
---@return ImGuiID
function imgui.GetID( str_id, )end

---@param str_id_begin string
---@param str_id_end string
---@return ImGuiID
function imgui.GetID( str_id_begin, str_id_end, )end

---@return ImGuiIO 
function imgui.GetIO( )end

---@return ImGuiID
function imgui.GetItemID( )end

---@return ImVec2
function imgui.GetItemRectMax( )end

---@return ImVec2
function imgui.GetItemRectMin( )end

---@return ImVec2
function imgui.GetItemRectSize( )end

---@param key ImGuiKey
---@return string
function imgui.GetKeyName( key, )end

---@param key ImGuiKey
---@param repeat_delay number
---@param rate number
---@return number
function imgui.GetKeyPressedAmount( key, repeat_delay, rate, )end

---@return ImGuiViewport 
function imgui.GetMainViewport( )end

---@param button ImGuiMouseButton
---@return number
function imgui.GetMouseClickedCount( button, )end

---@return ImGuiMouseCursor
function imgui.GetMouseCursor( )end

---@param button ImGuiMouseButton
---@param lock_threshold number
---@return ImVec2
function imgui.GetMouseDragDelta( button, lock_threshold, )end

---@return ImVec2
function imgui.GetMousePos( )end

---@return ImVec2
function imgui.GetMousePosOnOpeningCurrentPopup( )end

---@return ImGuiPlatformIO 
function imgui.GetPlatformIO( )end

---@return number
function imgui.GetScrollMaxX( )end

---@return number
function imgui.GetScrollMaxY( )end

---@return number
function imgui.GetScrollX( )end

---@return number
function imgui.GetScrollY( )end

---@return ImGuiStorage 
function imgui.GetStateStorage( )end

---@return ImGuiStyle 
function imgui.GetStyle( )end

---@param idx ImGuiCol
---@return string
function imgui.GetStyleColorName( idx, )end

---@param idx ImGuiCol
---@return ImVec4 
function imgui.GetStyleColorVec4( idx, )end

---@return number
function imgui.GetTextLineHeight( )end

---@return number
function imgui.GetTextLineHeightWithSpacing( )end

---@return number
function imgui.GetTime( )end

---@return number
function imgui.GetTreeNodeToLabelSpacing( )end

---@return string
function imgui.GetVersion( )end

---@return ImGuiID
function imgui.GetWindowDockID( )end

---@return number
function imgui.GetWindowDpiScale( )end

---@return ImDrawList 
function imgui.GetWindowDrawList( )end

---@return number
function imgui.GetWindowHeight( )end

---@return ImVec2
function imgui.GetWindowPos( )end

---@return ImVec2
function imgui.GetWindowSize( )end

---@return ImGuiViewport 
function imgui.GetWindowViewport( )end

---@return number
function imgui.GetWindowWidth( )end

---@param str_id string
---@param user_texture_id ImTextureID
---@param image_size ImVec2 
---@param uv0 ImVec2 
---@param uv1 ImVec2 
---@param bg_col ImVec4 
---@param tint_col ImVec4 
---@return boolean
function imgui.ImageButton( str_id, user_texture_id, image_size, uv0, uv1, bg_col, tint_col, )end

---@param indent_w number
---@return nil
function imgui.Indent( indent_w, )end

---@param label string
---@param v Vec2
---@return Vec2
function imgui.InputFloat2( label, v, )end

---@param label string
---@param v Vec3
---@return Vec3
function imgui.InputFloat3( label, v, )end

---@param label string
---@param v Vec4
---@return Vec4
function imgui.InputFloat4( label, v, )end

---@param label string
---@param v Vec2d
---@return Vec2d
function imgui.InputInt2( label, v, )end

---@param label string
---@param v Vec3d
---@return Vec3d
function imgui.InputInt3( label, v, )end

---@param label string
---@param v Vec4d
---@return Vec4d
function imgui.InputInt4( label, v, )end

---@param str_id string
---@param size ImVec2 
---@param flags ImGuiButtonFlags
---@return boolean
function imgui.InvisibleButton( str_id, size, flags, )end

---@return boolean
function imgui.IsAnyItemActive( )end

---@return boolean
function imgui.IsAnyItemFocused( )end

---@return boolean
function imgui.IsAnyItemHovered( )end

---@return boolean
function imgui.IsAnyMouseDown( )end

---@return boolean
function imgui.IsItemActivated( )end

---@return boolean
function imgui.IsItemActive( )end

---@param mouse_button ImGuiMouseButton
---@return boolean
function imgui.IsItemClicked( mouse_button, )end

---@return boolean
function imgui.IsItemDeactivated( )end

---@return boolean
function imgui.IsItemDeactivatedAfterEdit( )end

---@return boolean
function imgui.IsItemEdited( )end

---@return boolean
function imgui.IsItemFocused( )end

---@param flags ImGuiHoveredFlags
---@return boolean
function imgui.IsItemHovered( flags, )end

---@return boolean
function imgui.IsItemToggledOpen( )end

---@return boolean
function imgui.IsItemToggledSelection( )end

---@return boolean
function imgui.IsItemVisible( )end

---@param key_chord ImGuiKeyChord
---@return boolean
function imgui.IsKeyChordPressed( key_chord, )end

---@param key ImGuiKey
---@return boolean
function imgui.IsKeyDown( key, )end

---@param key ImGuiKey
---@param repeat boolean
---@return boolean
function imgui.IsKeyPressed( key, repeat, )end

---@param key ImGuiKey
---@return boolean
function imgui.IsKeyReleased( key, )end

---@param button ImGuiMouseButton
---@param repeat boolean
---@return boolean
function imgui.IsMouseClicked( button, repeat, )end

---@param button ImGuiMouseButton
---@return boolean
function imgui.IsMouseDoubleClicked( button, )end

---@param button ImGuiMouseButton
---@return boolean
function imgui.IsMouseDown( button, )end

---@param button ImGuiMouseButton
---@param lock_threshold number
---@return boolean
function imgui.IsMouseDragging( button, lock_threshold, )end

---@param r_min ImVec2 
---@param r_max ImVec2 
---@param clip boolean
---@return boolean
function imgui.IsMouseHoveringRect( r_min, r_max, clip, )end

---@param button ImGuiMouseButton
---@return boolean
function imgui.IsMouseReleased( button, )end

---@param str_id string
---@param flags ImGuiPopupFlags
---@return boolean
function imgui.IsPopupOpen( str_id, flags, )end

---@param size ImVec2 
---@return boolean
function imgui.IsRectVisible( size, )end

---@param rect_min ImVec2 
---@param rect_max ImVec2 
---@return boolean
function imgui.IsRectVisible( rect_min, rect_max, )end

---@return boolean
function imgui.IsWindowAppearing( )end

---@return boolean
function imgui.IsWindowCollapsed( )end

---@return boolean
function imgui.IsWindowDocked( )end

---@param flags ImGuiFocusedFlags
---@return boolean
function imgui.IsWindowFocused( flags, )end

---@param flags ImGuiHoveredFlags
---@return boolean
function imgui.IsWindowHovered( flags, )end

---@param label string
---@param text string
---@return nil
function imgui.LabelText( label, text, )end

---@param ini_filename string
---@return nil
function imgui.LoadIniSettingsFromDisk( ini_filename, )end

---@param ini_data string
---@param ini_size size_t
---@return nil
function imgui.LoadIniSettingsFromMemory( ini_data, ini_size, )end

---@return nil
function imgui.LogButtons( )end

---@return nil
function imgui.LogFinish( )end

---@param auto_open_depth number
---@return nil
function imgui.LogToClipboard( auto_open_depth, )end

---@param auto_open_depth number
---@param filename string
---@return nil
function imgui.LogToFile( auto_open_depth, filename, )end

---@param auto_open_depth number
---@return nil
function imgui.LogToTTY( auto_open_depth, )end

---@param size size_t
---@return void 
function imgui.MemAlloc( size, )end

---@param label string
---@param shortcut string
---@param selected boolean
---@param enabled boolean
---@return boolean
function imgui.MenuItem( label, shortcut, selected, enabled, )end

---@return nil
function imgui.NewFrame( )end

---@return nil
function imgui.NewLine( )end

---@return nil
function imgui.NextColumn( )end

---@param str_id string
---@param popup_flags ImGuiPopupFlags
---@return nil
function imgui.OpenPopup( str_id, popup_flags, )end

---@param str_id string
---@param popup_flags ImGuiPopupFlags
---@return nil
function imgui.OpenPopupOnItemClick( str_id, popup_flags, )end

---@return nil
function imgui.PopClipRect( )end

---@return nil
function imgui.PopFont( )end

---@return nil
function imgui.PopID( )end

---@return nil
function imgui.PopItemFlag( )end

---@return nil
function imgui.PopItemWidth( )end

---@param count number
---@return nil
function imgui.PopStyleColor( count, )end

---@param count number
---@return nil
function imgui.PopStyleVar( count, )end

---@return nil
function imgui.PopTextWrapPos( )end

---@param fraction number
---@param size_arg ImVec2 
---@param overlay string
---@return nil
function imgui.ProgressBar( fraction, size_arg, overlay, )end

---@param clip_rect_min ImVec2 
---@param clip_rect_max ImVec2 
---@param intersect_with_current_clip_rect boolean
---@return nil
function imgui.PushClipRect( clip_rect_min, clip_rect_max, intersect_with_current_clip_rect, )end

---@param str_id string
---@return nil
function imgui.PushID( str_id, )end

---@param str_id_begin string
---@param str_id_end string
---@return nil
function imgui.PushID( str_id_begin, str_id_end, )end

---@param option ImGuiItemFlags
---@param enabled boolean
---@return nil
function imgui.PushItemFlag( option, enabled, )end

---@param item_width number
---@return nil
function imgui.PushItemWidth( item_width, )end

---@param idx ImGuiCol
---@param col ImU32
---@return nil
function imgui.PushStyleColor( idx, col, )end

---@param idx ImGuiStyleVar
---@param val number
---@return nil
function imgui.PushStyleVar( idx, val, )end

---@param idx ImGuiStyleVar
---@param val_x number
---@return nil
function imgui.PushStyleVarX( idx, val_x, )end

---@param idx ImGuiStyleVar
---@param val_y number
---@return nil
function imgui.PushStyleVarY( idx, val_y, )end

---@param wrap_local_pos_x number
---@return nil
function imgui.PushTextWrapPos( wrap_local_pos_x, )end

---@param label string
---@param active boolean
---@return boolean
function imgui.RadioButton( label, active, )end

---@return nil
function imgui.Render( )end

---@param button ImGuiMouseButton
---@return nil
function imgui.ResetMouseDragDelta( button, )end

---@param offset_from_start_x number
---@param spacing number
---@return nil
function imgui.SameLine( offset_from_start_x, spacing, )end

---@param ini_filename string
---@return nil
function imgui.SaveIniSettingsToDisk( ini_filename, )end

---@param label string
---@param selected boolean
---@param flags ImGuiSelectableFlags
---@param size ImVec2 
---@return boolean
function imgui.Selectable( label, selected, flags, size, )end

---@return nil
function imgui.Separator( )end

---@param label string
---@return nil
function imgui.SeparatorText( label, )end

---@param text string
---@return nil
function imgui.SetClipboardText( text, )end

---@param flags ImGuiColorEditFlags
---@return nil
function imgui.SetColorEditOptions( flags, )end

---@param column_index number
---@param offset_x number
---@return nil
function imgui.SetColumnOffset( column_index, offset_x, )end

---@param column_index number
---@param width number
---@return nil
function imgui.SetColumnWidth( column_index, width, )end

---@param local_pos ImVec2 
---@return nil
function imgui.SetCursorPos( local_pos, )end

---@param local_x number
---@return nil
function imgui.SetCursorPosX( local_x, )end

---@param local_y number
---@return nil
function imgui.SetCursorPosY( local_y, )end

---@param pos ImVec2 
---@return nil
function imgui.SetCursorScreenPos( pos, )end

---@return nil
function imgui.SetItemDefaultFocus( )end

---@param key ImGuiKey
---@return nil
function imgui.SetItemKeyOwner( key, )end

---@param offset number
---@return nil
function imgui.SetKeyboardFocusHere( offset, )end

---@param cursor_type ImGuiMouseCursor
---@return nil
function imgui.SetMouseCursor( cursor_type, )end

---@param visible boolean
---@return nil
function imgui.SetNavCursorVisible( visible, )end

---@param want_capture_keyboard boolean
---@return nil
function imgui.SetNextFrameWantCaptureKeyboard( want_capture_keyboard, )end

---@param want_capture_mouse boolean
---@return nil
function imgui.SetNextFrameWantCaptureMouse( want_capture_mouse, )end

---@return nil
function imgui.SetNextItemAllowOverlap( )end

---@param is_open boolean
---@param cond ImGuiCond
---@return nil
function imgui.SetNextItemOpen( is_open, cond, )end

---@param selection_user_data ImGuiSelectionUserData
---@return nil
function imgui.SetNextItemSelectionUserData( selection_user_data, )end

---@param key_chord ImGuiKeyChord
---@param flags ImGuiInputFlags
---@return nil
function imgui.SetNextItemShortcut( key_chord, flags, )end

---@param storage_id ImGuiID
---@return nil
function imgui.SetNextItemStorageID( storage_id, )end

---@param item_width number
---@return nil
function imgui.SetNextItemWidth( item_width, )end

---@param alpha number
---@return nil
function imgui.SetNextWindowBgAlpha( alpha, )end

---@param collapsed boolean
---@param cond ImGuiCond
---@return nil
function imgui.SetNextWindowCollapsed( collapsed, cond, )end

---@param size ImVec2 
---@return nil
function imgui.SetNextWindowContentSize( size, )end

---@param dock_id ImGuiID
---@param cond ImGuiCond
---@return nil
function imgui.SetNextWindowDockID( dock_id, cond, )end

---@return nil
function imgui.SetNextWindowFocus( )end

---@param pos ImVec2 
---@param cond ImGuiCond
---@param pivot ImVec2 
---@return nil
function imgui.SetNextWindowPos( pos, cond, pivot, )end

---@param scroll ImVec2 
---@return nil
function imgui.SetNextWindowScroll( scroll, )end

---@param size ImVec2 
---@param cond ImGuiCond
---@return nil
function imgui.SetNextWindowSize( size, cond, )end

---@param viewport_id ImGuiID
---@return nil
function imgui.SetNextWindowViewport( viewport_id, )end

---@param local_x number
---@param center_x_ratio number
---@return nil
function imgui.SetScrollFromPosX( local_x, center_x_ratio, )end

---@param local_y number
---@param center_y_ratio number
---@return nil
function imgui.SetScrollFromPosY( local_y, center_y_ratio, )end

---@param center_x_ratio number
---@return nil
function imgui.SetScrollHereX( center_x_ratio, )end

---@param center_y_ratio number
---@return nil
function imgui.SetScrollHereY( center_y_ratio, )end

---@param scroll_x number
---@return nil
function imgui.SetScrollX( scroll_x, )end

---@param scroll_y number
---@return nil
function imgui.SetScrollY( scroll_y, )end

---@param tab_or_docked_window_label string
---@return nil
function imgui.SetTabItemClosed( tab_or_docked_window_label, )end

---@param collapsed boolean
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowCollapsed( collapsed, cond, )end

---@param name string
---@param collapsed boolean
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowCollapsed( name, collapsed, cond, )end

---@return nil
function imgui.SetWindowFocus( )end

---@param name string
---@return nil
function imgui.SetWindowFocus( name, )end

---@param scale number
---@return nil
function imgui.SetWindowFontScale( scale, )end

---@param pos ImVec2 
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowPos( pos, cond, )end

---@param name string
---@param pos ImVec2 
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowPos( name, pos, cond, )end

---@param size ImVec2 
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowSize( size, cond, )end

---@param name string
---@param size ImVec2 
---@param cond ImGuiCond
---@return nil
function imgui.SetWindowSize( name, size, cond, )end

---@param key_chord ImGuiKeyChord
---@param flags ImGuiInputFlags
---@return boolean
function imgui.Shortcut( key_chord, flags, )end

---@param label string
---@return nil
function imgui.ShowFontSelector( label, )end

---@param label string
---@return boolean
function imgui.ShowStyleSelector( label, )end

---@return nil
function imgui.ShowUserGuide( )end

---@param label string
---@param v Vec2
---@param min number
---@param max number
---@return Vec2
function imgui.SliderFloat2( label, v, min, max, )end

---@param label string
---@param v Vec3
---@param min number
---@param max number
---@return Vec3
function imgui.SliderFloat3( label, v, min, max, )end

---@param label string
---@param v Vec4
---@param min number
---@param max number
---@return Vec4
function imgui.SliderFloat4( label, v, min, max, )end

---@param label string
---@param v Vec2d
---@param min number
---@param max number
---@return Vec2d
function imgui.SliderInt2( label, v, min, max, )end

---@param label string
---@param v Vec3d
---@param min number
---@param max number
---@return Vec3d
function imgui.SliderInt3( label, v, min, max, )end

---@param label string
---@param v Vec4d
---@param min number
---@param max number
---@return Vec4d
function imgui.SliderInt4( label, v, min, max, )end

---@param label string
---@return boolean
function imgui.SmallButton( label, )end

---@return nil
function imgui.Spacing( )end

---@param label string
---@param flags ImGuiTabItemFlags
---@return boolean
function imgui.TabItemButton( label, flags, )end

---@return nil
function imgui.TableAngledHeadersRow( )end

---@return number
function imgui.TableGetColumnCount( )end

---@param column_n number
---@return ImGuiTableColumnFlags
function imgui.TableGetColumnFlags( column_n, )end

---@return number
function imgui.TableGetColumnIndex( )end

---@param column_n number
---@return string
function imgui.TableGetColumnName( column_n, )end

---@return number
function imgui.TableGetHoveredColumn( )end

---@return number
function imgui.TableGetRowIndex( )end

---@return ImGuiTableSortSpecs 
function imgui.TableGetSortSpecs( )end

---@param label string
---@return nil
function imgui.TableHeader( label, )end

---@return nil
function imgui.TableHeadersRow( )end

---@return boolean
function imgui.TableNextColumn( )end

---@param row_flags ImGuiTableRowFlags
---@param min_row_height number
---@return nil
function imgui.TableNextRow( row_flags, min_row_height, )end

---@param target ImGuiTableBgTarget
---@param color ImU32
---@param column_n number
---@return nil
function imgui.TableSetBgColor( target, color, column_n, )end

---@param column_n number
---@param v boolean
---@return nil
function imgui.TableSetColumnEnabled( column_n, v, )end

---@param column_n number
---@return boolean
function imgui.TableSetColumnIndex( column_n, )end

---@param label string
---@param flags ImGuiTableColumnFlags
---@param init_width_or_weight number
---@param user_id ImGuiID
---@return nil
function imgui.TableSetupColumn( label, flags, init_width_or_weight, user_id, )end

---@param cols number
---@param rows number
---@return nil
function imgui.TableSetupScrollFreeze( cols, rows, )end

---@param text string
---@param color ImVec4 
---@return nil
function imgui.TextColored( text, color, )end

---@param text string
---@return nil
function imgui.TextDisabled( text, )end

---@param label string
---@return boolean
function imgui.TextLink( label, )end

---@param label string
---@param url string
---@return nil
function imgui.TextLinkOpenURL( label, url, )end

---@param text string
---@param text_end string
---@return nil
function imgui.TextUnformatted( text, text_end, )end

---@param text string
---@return nil
function imgui.TextWrapped( text, )end

---@param text string
---@return nil
function imgui.Text( text, )end

---@param label string
---@param flags ImGuiTreeNodeFlags
---@return boolean
function imgui.TreeNodeEx( label, flags, )end

---@return nil
function imgui.TreePop( )end

---@param str_id string
---@return nil
function imgui.TreePush( str_id, )end

---@param indent_w number
---@return nil
function imgui.Unindent( indent_w, )end

---@return nil
function imgui.UpdatePlatformWindows( )end

---@param prefix string
---@param b boolean
---@return nil
function imgui.Value( prefix, b, )end

---@param prefix string
---@param v number
---@param float_format string
---@return nil
function imgui.Value( prefix, v, float_format, )end

   return imgui