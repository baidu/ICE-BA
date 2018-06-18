/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <glog/logging.h>
#include "image_utils.h"
//#include <driver/xp_aec_table.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __IMAGE_UTILS_NO_DEBUG__
#endif

namespace XP {

// only use central area in the image
constexpr int kMarginRow = 50;
constexpr int kMarginCol = 100;
constexpr int kPixelStep = 2;

// Compute the histogram of a sampled area of the input image and return the number of
// sampled pixels
// [NOTE] This function is hardcoded for VGA / WVGA images for now
int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr) {
  const int end_row = raw_img.rows - kMarginRow;
  const int end_col = raw_img.cols - kMarginCol;

  // Given the current algorithm, collecting histogram is not
  // necessary. But we still do so in case later we switch to a better
  // algorithm
  int pixel_num = 0;
  int avg_pixel_val = 0;
  histogram->clear();
  histogram->resize(256, 0);
  int over_exposure_pixel_num = 0;
  for (int i = kMarginRow; i < end_row; i += kPixelStep) {
    for (int j = kMarginCol; j < end_col; j += kPixelStep) {
      const uint8_t pixel_val = raw_img.data[i * raw_img.cols + j];
      avg_pixel_val += pixel_val;
      (*histogram)[pixel_val]++;
      ++pixel_num;
    }
  }
  if (avg_pixel_val_ptr) {
    *avg_pixel_val_ptr = avg_pixel_val / pixel_num;
  }
  return pixel_num;
}

// [NOTE] Instead of matching the cdf(s), we brute-force scale the histograms and match them
// directly.  This matchingHistogram is intended to match two histograms of images taken with
// different gain/exposure settings.
float matchingHistogram(const std::vector<int>& hist_src,
                        const std::vector<int>& hist_tgt,
                        const float init_scale) {
  std::vector<int> cdf_tgt(256);
  cdf_tgt[0] = hist_tgt[0];
  for (int i = 1; i < 256; ++i) {
    cdf_tgt[i] = hist_tgt[i] + cdf_tgt[i - 1];
  }
  constexpr float delta_scale = 0.02;
  float best_scale = -1.f;  // an invalid value
  int best_cdf_L1_dist = std::numeric_limits<int>::max();
  for (int s = -4; s < 5; ++s) {
    float scale = init_scale + s * delta_scale;
    std::vector<int> hist_src_scale(256, 0);
    for (int i = 0; i < 256; ++i) {
      int si = i * scale;
      if (si >= 255) {
        int tmp_acc = 0;
        for (int j = i; j < 256; ++j) {
          tmp_acc += hist_src[j];
        }
        hist_src_scale[255] = tmp_acc;
        break;
      }
      hist_src_scale[si] = hist_src_scale[si] + hist_src[i];
    }

    int cdf_L1_dist = 0;
    int cdf_src_cumsum = 0;
    for (int i = 0; i < 256; ++i) {
      cdf_src_cumsum += hist_src_scale[i];
      int L1_dist = std::abs(cdf_src_cumsum - cdf_tgt[i]);
      cdf_L1_dist += L1_dist;
    }
    // We simply assume these histograms are sampled from the same size of images
    CHECK_EQ(cdf_src_cumsum, cdf_tgt[255]);
    VLOG(1) << "scale = " << scale << " cdf_L1_dist = " << cdf_L1_dist;
    if (cdf_L1_dist < best_cdf_L1_dist) {
      best_scale = scale;
      best_cdf_L1_dist = cdf_L1_dist;

      if (VLOG_IS_ON(3)) {
        cv::Mat hist_canvas;
        const int scale = 2;
        const int height = 64 * scale;
        const int width = 256 * scale;
        hist_canvas.create(height * 2, width, CV_8UC3);
        hist_canvas.setTo(0x00);
        cv::Mat hist_img_src = hist_canvas(cv::Rect(0, 0, width, height));
        drawHistogram(&hist_img_src, hist_src_scale);
        cv::Mat hist_img_tgt = hist_canvas(cv::Rect(0, height,  width, height));
        drawHistogram(&hist_img_tgt, hist_tgt);
        cv::imshow("matchingHistogram", hist_canvas);
        cv::waitKey(0);
      }
    }
  }
  CHECK_GT(best_scale, 0.f);
  VLOG(1) << "best scale = " << best_scale << " cdf_L1_dist = " << best_cdf_L1_dist;
  return best_scale;
}

void drawHistogram(cv::Mat* img_hist, const std::vector<int>& histogram) {
  // Get some stats of this histogram
  const int N = static_cast<int>(histogram.size());
  int total_num = 0;
  int avg_pixel_val = 0;
  for (int i = 0; i < N; ++i) {
    total_num += histogram[i];
    avg_pixel_val += histogram[i] * i;
  }
  avg_pixel_val /= total_num;

  int acc_pixel_counts = 0;
  int median_pixel_val;
  for (int i = 0; i < N; ++i) {
    acc_pixel_counts += histogram[i];
    if (acc_pixel_counts >= total_num / 2) {
      median_pixel_val = i;
      break;
    }
  }

  const int scale = 2;
  const int width = N * scale + 1;
  const int height = 64 * scale;
  CHECK_NOTNULL(img_hist);
  if (img_hist->rows == 0) {
    img_hist->create(height, width, CV_8UC3);
  }
  img_hist->setTo(0x00);
  float hist_max = 0.1f;  // scale the max y axis to 10%
  for (int i = 0; i < N; ++i) {
    float val = static_cast<float>(histogram[i]) / total_num / hist_max;
    cv::Rect rect(i * scale, height * (1 - val), scale, height * val);
    cv::rectangle(*img_hist, rect, cv::Scalar(0, 0, 255));
  }
  cv::putText(*img_hist, "mean: " + boost::lexical_cast<std::string>(avg_pixel_val),
              cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 255, 255));
  cv::putText(*img_hist, "median: " + boost::lexical_cast<std::string>(median_pixel_val),
              cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 255, 255));
}

}  // namespace XP
