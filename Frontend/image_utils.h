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
#ifndef XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_

#include <opencv2/core.hpp>
#include <vector>

namespace XP {

int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr = nullptr);

float matchingHistogram(const std::vector<int>& hist_src,
                        const std::vector<int>& hist_tgt,
                        const float init_scale);

void drawHistogram(cv::Mat* img_hist, const std::vector<int>& histogram);

}  // namespace XP

#endif  // XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_
