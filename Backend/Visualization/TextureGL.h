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
#ifndef _TEXTURE_GL_H_
#define _TEXTURE_GL_H_

#include <GL/glew.h>

template<int CHANNELS>
class TextureGL {

 public:

  inline TextureGL() { memset(this, 0, sizeof(TextureGL<CHANNELS>)); }
  inline ~TextureGL() { Delete(); }
  inline void Generate(const GLint filterType = GL_NEAREST) {
#ifdef CFG_DEBUG
    UT_ASSERT(m_texture == 0);
#endif
    glGenTextures(1, &m_texture);
    Bind();
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, filterType);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, filterType);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  }
  inline void Generate(const int w, const int h, const GLint filterType = GL_NEAREST) {
    if (m_texture == 0)
      Generate(filterType);
    else
      Bind();
    Resize(w, h);
  }
  inline void Bind() const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_texture != 0);
#endif
    GLint boundTexture;
    glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
    if (boundTexture != m_texture)
      glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_texture);
  }
  inline void Unbind() const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_texture != 0);
    GLint boundTexture;
    glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
    UT_ASSERT(boundTexture == m_texture);
#endif
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
  }
  inline void SetFilterType(const GLint filterType) const {
#ifdef CFG_DEBUG
    AssertBound();
#endif
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, filterType);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, filterType);
  }
  inline void Delete() {
    if (m_texture != 0) {
      GLint boundTexture;
      glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
      if (boundTexture == m_texture)
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
      glDeleteTextures(1, &m_texture);
      m_texture = 0;
    }
  }

  inline void Resize(const int w, const int h);

  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels) const;
  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels,
                                                    const GLenum components) const;
  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const int w,
                                                    const int h) const;
  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const int w, const int h,
                                                    const GLenum components) const;
  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const int x, const int y,
                                                    const int w, const int h) const;
  template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const int x, const int y,
                                                    const int w, const int h, const GLenum components) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const GLenum components) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const int w, const int h) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const int w, const int h,
                                                    const GLenum components) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const int x, const int y,
                                                    const int w, const int h) const;
  template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const int x, const int y,
                                                    const int w, const int h, const GLenum components) const;

  inline GLenum GetAttachment() const {
    GLint attachedTexture;
    GLenum i = 0, attachment;
    for (i = 0, attachment = GL_COLOR_ATTACHMENT0_EXT; i < 8; ++i, ++attachment) {
      glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, attachment,
                                               GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
      if (attachedTexture == m_texture)
        return attachment;
    }
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB,
                              m_texture, 0);
    return GL_COLOR_ATTACHMENT0_EXT;
  }
  inline int GetWidth() const { return m_w; }
  inline int GetHeight() const { return m_h; }
  inline int GetPixelsNumber() const { return m_nPixels; }
  inline int GetTotalSize() const { return m_totalSize; }
  inline GLuint GetTexture() const { return m_texture; }
  inline void ComputeTotalSize();

#ifdef CFG_DEBUG
  inline void AssertBound() const {
    UT_ASSERT(m_texture != 0);
    GLint boundTexture;
    glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
    UT_ASSERT(boundTexture == m_texture);
  }
  inline void AssertComponentsCompitable(const GLenum components) const {
    UT_ASSERT(components == GL_RED || components == GL_GREEN || components == GL_BLUE ||
              components == GL_ALPHA
              || CHANNELS >= 2 && components == GL_RG
              || CHANNELS >= 3 && components == GL_RGB
              || CHANNELS == 4 && components == GL_RGBA);
  }
#endif

 protected:

  GLuint m_texture;
  int m_w, m_h, m_nPixels, m_totalSize;

};

typedef TextureGL<1> TextureGL1;
typedef TextureGL<2> TextureGL2;
typedef TextureGL<3> TextureGL3;
typedef TextureGL<4> TextureGL4;

#include "TextureGL.hpp"

#endif
