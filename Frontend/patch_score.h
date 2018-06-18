/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
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
/*
 * patch_score.h
 *
 *  Created on: Dec 5, 2013
 *      Author: cforster
 */

#ifndef _XP_UTIL_PATCH_SCORE_H_
#define _XP_UTIL_PATCH_SCORE_H_

#include <stdint.h>

#if __SSE2__
#include <tmmintrin.h>
#endif

namespace XP {
namespace patch_score {


#if __SSE2__
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target) {  // NOLINT
  uint16_t sums_store[8];
  _mm_storeu_si128((__m128i*)sums_store, target);  // NOLINT
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
      sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}

// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target) {  // NOLINT
  uint32_t sums_store[4];
  _mm_storeu_si128((__m128i*)sums_store, target);  // NOLINT
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

/// Zero Mean Sum of Squared Differences Cost
template<int HALF_PATCH_SIZE>
class ZMSSD {
 public:
  static const int patch_size_ = 2 * HALF_PATCH_SIZE;
  static const int patch_area_ = patch_size_ * patch_size_;
  static const int threshold_  = 2000 * patch_area_;
  uint8_t* ref_patch_;
  int sumA_, sumAA_;
  int sumB_ = 0;

  explicit ZMSSD(uint8_t* ref_patch) : ref_patch_(ref_patch) {
    uint32_t sumA_uint = 0;
    uint32_t sumAA_uint = 0;
    for (int r = 0; r < patch_area_; r++) {
      uint8_t n = ref_patch_[r];
      sumA_uint += n;
      sumAA_uint += n*n;
    }
    sumA_ = sumA_uint;
    sumAA_ = sumAA_uint;
  }

  static int threshold() { return threshold_; }

  int meanA() const { return sumA_ / patch_area_; }
  int meanB() const { return sumB_ / patch_area_; }

  int computeScore(uint8_t* cur_patch) const {
    uint32_t sumB_uint = 0;
    uint32_t sumBB_uint = 0;
    uint32_t sumAB_uint = 0;
    for (int r = 0; r < patch_area_; r++) {
      const uint8_t cur_pixel = cur_patch[r];
      sumB_uint  += cur_pixel;
      sumBB_uint += cur_pixel * cur_pixel;
      sumAB_uint += cur_pixel * ref_patch_[r];
    }
    const int sumB = sumB_uint;
    const int sumBB = sumBB_uint;
    const int sumAB = sumAB_uint;
    return sumAA_ - 2 * sumAB + sumBB
        - (sumA_ * sumA_ - 2 * sumA_ * sumB + sumB * sumB) / patch_area_;
  }

  // Return zero-mean sum of squared difference (zmssd) and sum of squared difference (ssd)
  void computeScore(uint8_t* cur_patch, int stride, int* zmssd, int* ssd) {
    int sumB, sumBB, sumAB;
#if __SSE2__
    if (patch_size_ == 8) {
      // From PTAM-GPL, Copyright 2008 Isis Innovation Limited
      __m128i xImageAsEightBytes;
      __m128i xImageAsWords;
      __m128i xTemplateAsEightBytes;
      __m128i xTemplateAsWords;
      __m128i xZero;
      __m128i xImageSums;    // These sums are 8xuint16
      __m128i xImageSqSums;  // These sums are 4xint32
      __m128i xCrossSums;    // These sums are 4xint32
      __m128i xProduct;

      xImageSums = _mm_setzero_si128();
      xImageSqSums = _mm_setzero_si128();
      xCrossSums = _mm_setzero_si128();
      xZero = _mm_setzero_si128();

      uint8_t* imagepointer = cur_patch;
      uint8_t* templatepointer = ref_patch_;
      size_t cur_stride = stride;

      xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);  // NOLINT
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);  // NOLINT
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);  // NOLINT
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);  // NOLINT
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);  // NOLINT
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      sumB = SumXMM_16(xImageSums);
      sumAB = SumXMM_32(xCrossSums);
      sumBB = SumXMM_32(xImageSqSums);
    } else {
#else
      {
#endif
      uint32_t sumB_uint = 0;
      uint32_t sumBB_uint = 0;
      uint32_t sumAB_uint = 0;
      for (int y = 0, r = 0; y < patch_size_; ++y) {
        uint8_t* cur_patch_ptr = cur_patch + y * stride;
        for (int x = 0; x < patch_size_; ++x, ++r) {
          const uint8_t cur_px = cur_patch_ptr[x];
          sumB_uint  += cur_px;
          sumBB_uint += cur_px * cur_px;
          sumAB_uint += cur_px * ref_patch_[r];
        }
      }
      sumB = sumB_uint;
      sumBB = sumBB_uint;
      sumAB = sumAB_uint;
    }
    sumB_ = sumB;  // store sumB

    *ssd = sumAA_ - 2 * sumAB + sumBB;
    *zmssd = *ssd - (sumA_ * sumA_ - 2 * sumA_ * sumB + sumB * sumB) / patch_area_;
  }

  int computeSsdScoreSlow(uint8_t* cur_patch, int stride) const {
    int32_t sum_ssd = 0;
    int r = 0;
    for (int y = 0; y < patch_size_; ++y) {
      uint8_t* cur_patch_ptr = cur_patch + y * stride;
      for (int x = 0; x < patch_size_; ++x, ++r) {
        const uint8_t cur_px = cur_patch_ptr[x];
        int32_t d = static_cast<int32_t>(cur_px) - static_cast<int32_t>(ref_patch_[r]);
        sum_ssd += d * d;
      }
    }
    return sum_ssd;
  }

  // The result is slightly different (off by 1) comparing to computeScore due to rounding issue
  // in integer division
  int computeZmssdScoreSlow(uint8_t* cur_patch, int stride) const {
    int32_t sumA = 0;
    int32_t sumB = 0;
    int r;
    for (int y = 0, r = 0; y < patch_size_; ++y) {
      uint8_t* cur_patch_ptr = cur_patch + y * stride;
      for (int x = 0; x < patch_size_; ++x, ++r) {
        sumA += ref_patch_[r];
        sumB += cur_patch_ptr[x];
      }
    }

    int zmssd = 0;
    for (int y = 0, r = 0; y < patch_size_; ++y) {
      uint8_t* cur_patch_ptr = cur_patch + y * stride;
      for (int x = 0; x < patch_size_; ++x, ++r) {
        int32_t zmd = static_cast<int32_t>(ref_patch_[r]) * patch_area_ - sumA
            - static_cast<int32_t>(cur_patch_ptr[x]) * patch_area_ + sumB;
        zmssd += zmd * zmd;
      }
    }
    zmssd = zmssd / patch_area_ / patch_area_;
    return zmssd;
  }
};

}  // namespace patch_score
}  // namespace XP

#endif // _XP_UTIL_PATCH_SCORE_H_
