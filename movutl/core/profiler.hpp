#pragma once

#ifdef MOVUTL_TRACY_ENABLED
#include <tracy/Tracy.hpp>
#define MOVUTL_ZONE_SCOPED ZoneScoped
#define MOVUTL_ZONE_SCOPED_N(name) ZoneScopedN(name)
#define MOVUTL_ZONE_NAME(str, len) ZoneName(str, len) // 実行時文字列(プラグイン名等)をzone名にする
#define MOVUTL_FRAME_MARK FrameMark
#else
#define MOVUTL_ZONE_SCOPED
#define MOVUTL_ZONE_SCOPED_N(name)
#define MOVUTL_ZONE_NAME(str, len)
#define MOVUTL_FRAME_MARK
#endif
