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
template<int CHANNELS> inline const GLenum GetFormat();
template<> inline const GLenum GetFormat<1>() { return GL_LUMINANCE; }
template<> inline const GLenum GetFormat<2>() { return GL_RG; }
template<> inline const GLenum GetFormat<3>() { return GL_RGB; }
template<> inline const GLenum GetFormat<4>() { return GL_RGBA; }

template<typename TYPE>  inline const GLenum GetType();
template<> inline const GLenum GetType<ubyte >() { return GL_UNSIGNED_BYTE; }
template<> inline const GLenum GetType<ushort>() { return GL_UNSIGNED_SHORT; }
template<> inline const GLenum GetType<float >() { return GL_FLOAT; }

template<int CHANNELS> inline void TextureGL<CHANNELS>::Resize(const int w, const int h) {
  if (m_w == w && m_h == h)
    return;
  m_w = w;
  m_h = h;
  m_nPixels = w * h;
  ComputeTotalSize();
  Bind();
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetFormat<CHANNELS>(), m_w, m_h, 0, GetFormat<CHANNELS>(),
               GL_FLOAT, NULL);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels) const {
#ifdef CFG_DEBUG
  AssertBound();
#endif
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetFormat<CHANNELS>(), m_w, m_h, 0, GetFormat<CHANNELS>(),
               GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels, const GLenum components) const {
#ifdef CFG_DEBUG
  AssertBound();
  AssertComponentsCompitable(components);
#endif
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetFormat<CHANNELS>(), m_w, m_h, 0, components,
               GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels, const int w, const int h) const {
#ifdef CFG_DEBUG
  AssertBound();
#endif
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, w, h, GetFormat<CHANNELS>(), GetType<TYPE>(),
                  pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels, const int w, const int h,
                                               const GLenum components) const {
#ifdef CFG_DEBUG
  AssertBound();
  AssertComponentsCompitable(components);
#endif
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, w, h, components, GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels, const int x, const int y,
                                               const int w, const int h) const {
#ifdef CFG_DEBUG
  AssertBound();
#endif
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, x, y, w, h, GetFormat<CHANNELS>(), GetType<TYPE>(),
                  pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::UploadFromCPU(const TYPE *pixels, const int x, const int y,
                                               const int w, const int h, const GLenum components) const {
#ifdef CFG_DEBUG
  AssertBound();
  AssertComponentsCompitable(components);
#endif
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, x, y, w, h, components, GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels) const {
  glReadBuffer(GetAttachment());
  glReadPixels(0, 0, m_w, m_h, GetFormat<CHANNELS>(), GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels, const GLenum components) const {
#ifdef CFG_DEBUG
  AssertComponentsCompitable(components);
#endif
  glReadBuffer(GetAttachment());
  glReadPixels(0, 0, m_w, m_h, components, GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels, const int w, const int h) const {
  glReadBuffer(GetAttachment());
  glReadPixels(0, 0, w, h, GetFormat<CHANNELS>(), GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels, const int w, const int h,
                                               const GLenum components) const {
#ifdef CFG_DEBUG
  AssertComponentsCompitable(components);
#endif
  glReadBuffer(GetAttachment());
  glReadPixels(0, 0, w, h, components, GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels, const int x, const int y, const int w,
                                               const int h) const {
  glReadBuffer(GetAttachment());
  glReadPixels(x, y, w, h, GetFormat<CHANNELS>(), GetType<TYPE>(), pixels);
}

template<int CHANNELS> template<typename TYPE>
inline void TextureGL<CHANNELS>::DownloadToCPU(TYPE *pixels, const int x, const int y, const int w,
                                               const int h, const GLenum components) const {
#ifdef CFG_DEBUG
  AssertComponentsCompitable(components);
#endif
  glReadBuffer(GetAttachment());
  glReadPixels(x, y, w, h, components, GetType<TYPE>(), pixels);
}
template<> inline void TextureGL<1>::ComputeTotalSize() { m_totalSize = m_nPixels; }
template<> inline void TextureGL<2>::ComputeTotalSize() { m_totalSize = (m_nPixels << 1); }
template<> inline void TextureGL<3>::ComputeTotalSize() { m_totalSize = (m_nPixels << 1) + m_nPixels; }
template<> inline void TextureGL<4>::ComputeTotalSize() { m_totalSize = (m_nPixels << 2); }
