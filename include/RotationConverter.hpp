// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef ROTATION_CONVERTER_HPP
#define ROTATION_CONVERTER_HPP

#include <opencv2/core.hpp>

namespace calib
{

  #define CALIB_RADIANS 0
  #define CALIB_DEGREES 1

  void euler(const cv::Mat& src, cv::Mat& dst, int argType = CALIB_RADIANS);
  void rodriguesToEuler(const cv::Mat& src, cv::Mat& dst, int argType = CALIB_RADIANS);
  void eulerToRodrigues(const cv::Mat& src, cv::Mat& dst, int argType = CALIB_RADIANS);

}

#endif
