#pragma once
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
//
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/plugin/input.hpp>

namespace mu::detail {
extern InputPluginTable plg_video_reader;
} // namespace mu::detail
