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
#ifndef _UTILITY_WM_H_
#define _UTILITY_WM_H_

#include <cvd/rgb.h>
#include <cvd/image.h>
#include <cvd/image_io.h>
#include <cvd/draw.h>
#include "simd_sse_neon.h"

namespace UT {

template<> inline CVD::Rgb<ubyte> Invalid<CVD::Rgb<ubyte> >() { return CVD::Rgb<ubyte>(0, 0, 0); }
inline bool ImageValid(const CVD::ImageRef size) { return size.x != 0 && size.y != 0; }
template<typename TYPE> inline bool ImageValid(const CVD::Image<TYPE> &I) {
  return ImageValid(I.size());
}
template<typename TYPE> inline bool ImageInvalid(const CVD::Image<TYPE> &I) {
  return I.size().x == 0 || I.size().y == 0;
}
template<typename TYPE> inline void ImageInvalidate(CVD::Image<TYPE> &I) {
  //I.resize(CVD::ImageRef(0, 0));
  if (ImageValid(I)) {
    I = CVD::Image<TYPE>(CVD::ImageRef(0, 0));
  }
}
template<typename TYPE> inline void ImageResize(const CVD::ImageRef size, CVD::Image<TYPE> &I) {
  if (ImageValid(size)) {
    I.resize(size);
  } else {
    ImageInvalidate(I);
  }
}
template<typename TYPE> inline void ImageCopy(const CVD::Image<TYPE> &Is, CVD::Image<TYPE> &Id) {
  if (ImageValid(Is)) {
    Id.resize(Is.size());
    if (Is.data() != Id.data()) {
      memcpy(Id.data(), Is.data(), sizeof(TYPE) * Is.totalsize());
    }
  } else {
    ImageInvalidate(Id);
  }
}
template<typename TYPE> inline void ImageCopy(const TYPE *Is, CVD::Image<TYPE> &Id) {
#ifdef CFG_DEBUG
  UT_ASSERT(ImageValid(Id));
#endif
  if (Is && Is != Id.data()) {
    memcpy(Id.data(), Is, sizeof(TYPE) * Id.totalsize());
  }
}
template<typename TYPE_SRC, typename TYPE_DST> inline void ImageCopy(const CVD::Image<TYPE_SRC> &Is,
                                                                     CVD::Image<TYPE_DST> &Id) {
  if (ImageValid(Is)) {
    Id.resize(Is.size());
    CVD::copy(Is, Id);
  } else {
    ImageInvalidate(Id);
  }
}
template<typename TYPE> inline void ImageSwap(CVD::Image<TYPE> &I1, CVD::Image<TYPE> &I2) {
  CVD::Image<TYPE> It = I1;
  I1 = I2;
  I2 = It;
}

template<typename TYPE> inline void ImageDownSample(const CVD::Image<TYPE> &Is,
                                                    CVD::Image<TYPE> &Id);
//template<> inline void ImageDownSample<ubyte>(const CVD::Image<ubyte> &Is, CVD::Image<ubyte> &Id) {
//  if (ImageInvalid(Is)) {
//    ImageInvalidate(Id);
//    return;
//  }
//  __m128i i1, i2;
//  const __m128i m = _mm_setr_epi32(0x00ff00ff, 0x00ff00ff, 0x00ff00ff, 0x00ff00ff);
//  const int w = Is.size().x >> 1, h = Is.size().y >> 1, wF = SIMD_BYTE_FLOOR(w);
//  Id.resize(CVD::ImageRef(wF, h));
//  for (int y = 0; y < h; ++y) {
//    const int _y1 = y << 1, _y2 = _y1 + 1;
//    const __m128i *_i1 = (__m128i *) Is[_y1], *_i2 = (__m128i *) Is[_y2];
//    __m128i *i = (__m128i *) Id[y];
//    for (int x = 0; x < wF; x += 16, ++i, _i1 += 2, _i2 += 2) {
//      i1 = _mm_avg_epu8(_i1[0], _i2[0]);
//      i1 = _mm_avg_epu8(i1, _mm_srli_si128(i1, 1));
//      i2 = _mm_avg_epu8(_i1[1], _i2[1]);
//      i2 = _mm_avg_epu8(i2, _mm_srli_si128(i2, 1));
//      *i = _mm_packus_epi16(_mm_and_si128(i1, m), _mm_and_si128(i2, m));
//    }
//  }
//}

//template<typename TYPE_GRADIENT> inline void _ImageGradient(const CVD::Image<ubyte> &I,
//                                                            CVD::Image<TYPE_GRADIENT> &G, const bool chkInv) {
//  G.resize(I.size());
//  const int w = I.size().x, h = I.size().y, xMax = w - 1, yMax = h - 1;
//#ifdef CFG_DEBUG
//  UT_ASSERT((w & 15) == 0);
//#endif
//  memset(G[0], 0, sizeof(TYPE_GRADIENT) * w);
//  memset(G[yMax], 0, sizeof(TYPE_GRADIENT) * w);
//
//#ifdef _MSC_VER
//  __m128i i10, i12, gx, gy;
//#else
//  __m128i gx, gy;
//  SIMD::U128i i10, i12;
//#endif  // _MSC_VER
//  const __m128i zero = _mm_setzero_si128();
//  for (int y = 1; y < yMax; ++y) {
//    const ubyte *i = I[y];
//    TYPE_GRADIENT *g = G[y];
//    const __m128i *i0 = (__m128i *) (i - w);
//    const __m128i *i1 = (__m128i *) i;
//    const __m128i *i2 = (__m128i *) (i + w);
//    __m128i *g1 = (__m128i *) g;
//    for (int x = 0; x < w; x += 16, ++i0, ++i1, ++i2) {
//#ifdef _MSC_VER
//      i10 = _mm_slli_si128(*i1, 1);  i10.m128i_u8[0] = i[x - 1];
//      i12 = _mm_srli_si128(*i1, 1);  i12.m128i_u8[15] = i[x + 16];
//      gx = _mm_sub_epi16(_mm_unpacklo_epi8(i12, zero), _mm_unpacklo_epi8(i10, zero));
//      gy = _mm_sub_epi16(_mm_unpacklo_epi8(*i2, zero), _mm_unpacklo_epi8(*i0, zero));
//      *g1++ = _mm_unpacklo_epi16(gx, gy);
//      *g1++ = _mm_unpackhi_epi16(gx, gy);
//      gx = _mm_sub_epi16(_mm_unpackhi_epi8(i12, zero), _mm_unpackhi_epi8(i10, zero));
//      gy = _mm_sub_epi16(_mm_unpackhi_epi8(*i2, zero), _mm_unpackhi_epi8(*i0, zero));
//      *g1++ = _mm_unpacklo_epi16(gx, gy);
//      *g1++ = _mm_unpackhi_epi16(gx, gy);
//#else
//      i10.v = _mm_slli_si128(*i1, 1); i10.m128i_u8[0] = i[x - 1];
//      i12.v = _mm_srli_si128(*i1, 1); i12.m128i_u8[15] = i[x + 16];
//      gx = _mm_sub_epi16(_mm_unpacklo_epi8(i12.v, zero), _mm_unpacklo_epi8(i10.v, zero));
//      gy = _mm_sub_epi16(_mm_unpacklo_epi8(*i2, zero), _mm_unpacklo_epi8(*i0, zero));
//      *g1++ = _mm_unpacklo_epi16(gx, gy);
//      *g1++ = _mm_unpackhi_epi16(gx, gy);
//      gx = _mm_sub_epi16(_mm_unpackhi_epi8(i12.v, zero), _mm_unpackhi_epi8(i10.v, zero));
//      gy = _mm_sub_epi16(_mm_unpackhi_epi8(*i2, zero), _mm_unpackhi_epi8(*i0, zero));
//      *g1++ = _mm_unpacklo_epi16(gx, gy);
//      *g1++ = _mm_unpackhi_epi16(gx, gy);
//#endif  // _MSC_VER
//    }
//    g[0].MakeZero();
//    g[xMax].MakeZero();
//  }
//}

//template<typename TYPE_GRADIENT> inline void _ImageGradient(const CVD::Image<float> &I,
//                                                            CVD::Image<TYPE_GRADIENT> &G, const bool chkInv) {
//  G.resize(I.size());
//  const int w = I.size().x, h = I.size().y, xMax = w - 1, yMax = h - 1;
//#ifdef CFG_DEBUG
//  UT_ASSERT((w & 3) == 0);
//#endif
//  // [NOTE](mingyu): avoid using LA::Vector2f to avoid mutual dependency
//  // memset(G[0], 0, sizeof(LA::Vector2f) * w);
//  // memset(G[yMax], 0, sizeof(LA::Vector2f) * w);
//  memset(G[0], 0, sizeof(TYPE_GRADIENT) * w);
//  memset(G[yMax], 0, sizeof(TYPE_GRADIENT) * w);
//
//  __m128 i10, i12, gx, gy;
//  //__m128i *_i10 = (__m128i *) &i10, *_i12 = (__m128i *) &i12;
//  const __m128 zero = _mm_setzero_ps(), s = _mm_set1_ps(0.5f), iInv = _mm_set1_ps(Invalid<float>());
//  for (int y = 1; y < yMax; ++y) {
//    const float *i = I[y];
//    TYPE_GRADIENT *g = G[y];
//    const __m128 *i0 = (__m128 *) (i - w);
//    const __m128 *i2 = (__m128 *) (i + w);
//    //const __m128 *i1 = (__m128 *) i;
//    //const __m128i *_i1 = (__m128i *) i;
//    __m128 *g1 = (__m128 *) g;
//    for (int x = 0; x < w; x += 4, i += 4, ++i0, ++i2/*, ++i1, ++_i1*/) {
//      //*_i10 = _mm_slli_si128(*_i1, 4);  i10.m128_f32[0] = i[-1];
//      //*_i12 = _mm_srli_si128(*_i1, 4);  i12.m128_f32[3] = i[4];
//      i10 = _mm_loadu_ps(i - 1);
//      i12 = _mm_loadu_ps(i + 1);
//      gx = _mm_mul_ps(_mm_sub_ps(i12, i10), s);
//      gy = _mm_mul_ps(_mm_sub_ps(*i2, *i0), s);
//      if (chkInv) {
//        gx = _mm_and_ps(_mm_and_ps(_mm_cmpneq_ps(i12, iInv), _mm_cmpneq_ps(i10, iInv)), gx);
//        gy = _mm_and_ps(_mm_and_ps(_mm_cmpneq_ps(*i2, iInv), _mm_cmpneq_ps(*i0, iInv)), gy);
//      }
//      *g1++ = _mm_unpacklo_ps(gx, gy);
//      *g1++ = _mm_unpackhi_ps(gx, gy);
//    }
//    g[0].MakeZero();
//    g[xMax].MakeZero();
//  }
//}


//template<typename TYPE_INTENSITY, typename TYPE_GRADIENT> inline void ImageGradient(
//  const CVD::Image<TYPE_INTENSITY> &I, CVD::Image<TYPE_GRADIENT> &G, const bool chkInv) {
//  _ImageGradient<TYPE_GRADIENT>(I, G, chkInv);
//}

template<typename TYPE_GRADIENT_XY, typename TYPE_GRADIENT> inline void ImageGradientX(
  const CVD::Image<TYPE_GRADIENT_XY> &G, CVD::Image<TYPE_GRADIENT> &Gx) {
  Gx.resize(G.size());
  const int w = G.size().x, h = G.size().y;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      Gx[y][x] = G[y][x].x();
    }
  }
}
template<typename TYPE_GRADIENT_XY, typename TYPE_GRADIENT> inline void ImageGradientY(
  const CVD::Image<TYPE_GRADIENT_XY> &G, CVD::Image<TYPE_GRADIENT> &Gy) {
  Gy.resize(G.size());
  const int w = G.size().x, h = G.size().y;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      Gy[y][x] = G[y][x].y();
    }
  }
}

template<> inline void ImageInterpolate<CVD::Rgb<ubyte>, CVD::Rgb<ubyte> >
(const CVD::Rgb<ubyte> i11, const CVD::Rgb<ubyte> i12, const CVD::Rgb<ubyte> i21,
 const CVD::Rgb<ubyte> i22,
 const xp128f &w, CVD::Rgb<ubyte> &ip) {
  ImageInterpolate<ubyte, ubyte>(i11.red, i12.red, i21.red, i22.red, w, ip.red);
  ImageInterpolate<ubyte, ubyte>(i11.green, i12.green, i21.green, i22.green, w, ip.green);
  ImageInterpolate<ubyte, ubyte>(i11.blue, i12.blue, i21.blue, i22.blue, w, ip.blue);
}
template<typename TYPE_SRC, typename TYPE_DST>
inline bool ImageInterpolate(const CVD::Image<TYPE_SRC> &I, const float x, const float y,
                             const bool chkInv, TYPE_DST &ip) {
  if (x < 0.0f || y < 0.0f) {
    return false;
  }
  const int ix1 = int(x), ix2 = ix1 + 1, iy1 = int(y), iy2 = iy1 + 1;
  if (ix2 >= I.size().x || iy2 >= I.size().y) {
    return false;
  }
  const TYPE_SRC i11 = I[iy1][ix1], i12 = I[iy2][ix1], i21 = I[iy1][ix2], i22 = I[iy2][ix2];
  if (chkInv && (i11 == Invalid<TYPE_SRC>() || i12 == Invalid<TYPE_SRC>() ||
                 i21 == Invalid<TYPE_SRC>() || i22 == Invalid<TYPE_SRC>())) {
    return false;
  }
  xp128f w;
  ImageInterpolateWeight(x - ix1, y - iy1, w);
  ImageInterpolate<TYPE_SRC, TYPE_DST>(i11, i12, i21, i22, w, ip);
  return true;
}
template<typename TYPE_INTENSITY, typename TYPE_GRADIENT, class TYPE_GRADIENT_VECTOR>
inline bool ImageInterpolateGradient(const CVD::Image<TYPE_INTENSITY> &I,
                                     const CVD::Image<TYPE_GRADIENT_VECTOR> &G, const float x, const float y,
                                     const bool chkInv, float &ip, float *gp) {
  if (x < 1.0f || y < 1.0f) {
    return false;
  }
  const int ix1 = int(x), ix2 = ix1 + 1, iy1 = int(y), iy2 = iy1 + 1;
  if (ix2 >= I.size().x - 1 || iy2 >= I.size().y - 1) {
    return false;
  }
  const TYPE_INTENSITY i11 = I[iy1][ix1], i12 = I[iy2][ix1], i21 = I[iy1][ix2], i22 = I[iy2][ix2];
  if (chkInv && (i11 == Invalid<TYPE_INTENSITY>() || i12 == Invalid<TYPE_INTENSITY>() ||
                 i21 == Invalid<TYPE_INTENSITY>() || i22 == Invalid<TYPE_INTENSITY>())) {
    return false;
  }
  xp128f w;
  ImageInterpolateWeight(x - ix1, y - iy1, w);
  ImageInterpolate<TYPE_INTENSITY, float>(i11, i12, i21, i22, w, ip);
  const TYPE_GRADIENT *g11 = G[iy1][ix1], *g12 = G[iy2][ix1];
  ImageInterpolate<TYPE_GRADIENT, float>(g11[0], g12[0], g11[2], g12[2], w, gp[0]);
  ImageInterpolate<TYPE_GRADIENT, float>(g11[1], g12[1], g11[3], g12[3], w, gp[1]);
  return true;
}
template<typename TYPE> inline void ImageSave(const std::string fileName, const CVD::Image<TYPE> &I,
                                              const bool verbose = true) {
  CVD::img_save(I, fileName);
  if (verbose) {
    PrintSaved(fileName);
  }
}
template<typename TYPE> inline void ImageCrop(CVD::Image<TYPE> &I, CVD::Image<TYPE> &ITmp) {
  const int w = I.size().x, wF = SIMD_BYTE_FLOOR(w);
  if (wF == w) {
    return;
  }
  const int h = I.size().y;
  ITmp.resize(CVD::ImageRef(wF, h));
  const int size = sizeof(TYPE) * wF;
  for (int y = 0; y < h; ++y) {
    memcpy(ITmp[y], I[y], size);
  }
  ImageSwap(I, ITmp);
}
#if 0
template<> inline void ImageCrop<float>(CVD::Image<float> &I, CVD::Image<float> &ITmp) {
  const int w = I.size().x, wF = SIMD_FLOAT_FLOOR(w);
  if (wF == w) {
    return;
  }
  const int h = I.size().y;
  ITmp.resize(CVD::ImageRef(wF, h));
  const int size = sizeof(float) * wF;
  for (int y = 0; y < h; ++y) {
    memcpy(ITmp[y], I[y], size);
  }
  ImageSwap(I, ITmp);
}
#endif
template<typename TYPE> inline bool ImageLoad(const std::string fileName, CVD::Image<TYPE> &I,
                                              CVD::Image<TYPE> &ITmp, const bool verbose = false) {
  if (!FileExists(fileName)) {
    return false;
  }
  CVD::img_load(I, fileName);
  ImageCrop<TYPE>(I, ITmp);
  if (verbose) {
    PrintLoaded(fileName);
  }
  return true;
}
template<typename TYPE> inline void ImageSaveB(const CVD::Image<TYPE> &I, FILE *fp) {
  const CVD::ImageRef size = I.size();
  SaveB<CVD::ImageRef>(size, fp);
  if (ImageValid(I)) {
    fwrite(I.data(), sizeof(TYPE), I.totalsize(), fp);
  }
}
template<typename TYPE> inline void ImageLoadB(CVD::Image<TYPE> &I, FILE *fp) {
  const CVD::ImageRef size = LoadB<CVD::ImageRef>(fp);
  if (ImageValid(size)) {
    I.resize(size);
    fread(I.data(), sizeof(TYPE), I.totalsize(), fp);
  } else {
    ImageInvalidate(I);
  }
}
template<class TYPE> inline float ImageMemoryMB(const CVD::Image<TYPE> &I) { return MemoryMB<TYPE>(I.totalsize()); }
template<typename TYPE> inline void ImageDrawLine(CVD::Image<TYPE> &I, const float *x1,
                                                  const float *x2, const TYPE clr) {
  CVD::drawLine(I, double(x1[0]), double(x1[1]), double(x2[0]), double(x2[1]), clr);
}
template<typename TYPE> inline void ImageDrawLine(CVD::Image<TYPE> &I, const int x1, const int y1,
                                                  const int x2, const int y2, const TYPE clr) {
  CVD::drawLine(I, CVD::ImageRef(x1, y1), CVD::ImageRef(x2, y2), clr);
}
template<typename TYPE> inline void ImageDrawCross(CVD::Image<TYPE> &I, const int x, const int y,
                                                   const int size, const TYPE clr) {
  CVD::drawLine(I, x - size, y, x + size, y, clr);
  CVD::drawLine(I, x, y - size, x, y + size, clr);
}
template<typename TYPE> inline void ImageDrawBox(CVD::Image<TYPE> &I, const CVD::ImageRef &x,
                                                 const int size, const TYPE clr) {
  const int w = I.size().x, h = I.size().y;
  const int x1 = x.x - size, x2 = x.x + size;
  const int y1 = x.y - size, y2 = x.y + size;
  for (int _y = y1; _y <= y2; ++_y) {
    for (int _x = x1; _x <= x2; ++_x) {
      if (_x >= 0 && _x < w && _y >= 0 && _y < h) {
        I[_y][_x] = clr;
      }
    }
  }
}
template<typename TYPE> inline void ImageDrawBox(CVD::Image<TYPE> &I, const float *x,
                                                 const int size, const TYPE clr) {
  const int w = I.size().x, h = I.size().y;
  const int x1 = int(x[0] - size + 0.5f), x2 = int(x[0] + size + 0.5f);
  const int y1 = int(x[1] - size + 0.5f), y2 = int(x[1] + size + 0.5f);
  for (int _y = y1; _y <= y2; ++_y) {
    for (int _x = x1; _x <= x2; ++_x) {
      if (_x >= 0 && _x < w && _y >= 0 && _y < h) {
        I[_y][_x] = clr;
      }
    }
  }
}
template<typename TYPE> inline void ImageDrawDisc(CVD::Image<TYPE> &I, const CVD::ImageRef x,
                                                  const int radius, const TYPE clr) {
  CVD::drawShape(I, x, CVD::getDisc(radius), clr);
}
template<typename TYPE> inline void ImageDrawCurve(CVD::Image<TYPE> &I, const float *vs,
                                                   const int N, const float vMin, const float vMax, const TYPE clr) {
  if (N == 0) {
    return;
  }
  const float dx = (I.size().x - 1) / (N - 1.0f), dy = (I.size().y - 1) / (vMax - vMin);
  float x1[2], x2[2];
  x2[0] = 0.0f;
  x2[1] = (vs[0] - vMin) * dy;
  for (int i = 1; i < N; ++i) {
    memcpy(x1, x2, sizeof(x1));
    x2[0] = dx + x2[0];
    x2[1] = (vs[i] - vMin) * dy;
    ImageDrawLine(I, x1, x2, clr);
  }
}
inline CVD::Rgb<ubyte> ImageColor(const ubyte c) { return CVD::Rgb<ubyte>(c, c, c); }

}

#endif
