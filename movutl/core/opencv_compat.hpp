#pragma once

// OpenCV 4系と5系の差異を吸収する。OpenCV 5ではgetPerspectiveTransform/getRotationMatrix2D/contourArea/boundingRect等が
// geometryモジュールへ移動し、<opencv2/opencv.hpp>に含まれなくなったため、OpenCVを使うファイルは<opencv2/opencv.hpp>の代わりにこれをincludeする
#include <opencv2/opencv.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry.hpp>
#endif
