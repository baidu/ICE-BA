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
#ifndef _VIEWER_H_
#define _VIEWER_H_

#include "EventHandler.h"
#include "Point.h"
#include "TextureGL.h"
#include "ViewerIBAKey.hpp"
#include <cvd/glwindow.h>
#include <GL/glew.h>
#include <GL/glut.h>

#define VW_STRING_FONT      GLUT_BITMAP_HELVETICA_18
#define VW_STRING_HEIGHT    18
#define VW_STRING_BORDER_X  15
#define VW_STRING_BORDER_Y  15
#define VW_STRING_GAP_Y     10

class Viewer {

 public:

  inline Viewer() : m_pWnd(NULL) {
#ifdef __linux__
    m_ctrlDown = false;
#endif
  }
  inline ~Viewer() { if (m_pWnd) delete m_pWnd; }

  virtual void Initialize(const int w, const int h);
  virtual bool Run(const bool visualize = true);

 protected:

  int GetCharacterWidth(void *front, const char c);
  int GetStringWidth(void *font, const char *str);
  int GetStringHeight(void *font);
  int GetStringYTopLeft(const int row, void *font);
  void DrawString(void *font, const char *str);
  void DrawStringCurrent(const char *format, ...);
  void DrawStringCurrent(void *font, const char *format, ...);
  void DrawStringAt(const int x, const int y, void *font, const char *format, ...);
  void DrawStringTopLeft(const char *format, ...);
  void DrawStringTopLeft(const int row, void *font, const char *format, ...);
  void DrawStringBottomLeft(const char *format, ...);
  //void DrawStringBottomLeft(const int row, const char *format, ...);

  template<int CHANNELS> inline void DrawTexture(const TextureGL<CHANNELS> &tex) {
    tex.Bind();
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0);                             glVertex2i(0, 0);
    glTexCoord2i(0, tex.GetHeight());               glVertex2i(0, m_pWnd->size().y);
    glTexCoord2i(tex.GetWidth(), tex.GetHeight());  glVertex2i(m_pWnd->size().x, m_pWnd->size().y);
    glTexCoord2i(tex.GetWidth(), 0);                glVertex2i(m_pWnd->size().x, 0);
    glEnd();
  }
  template<int CHANNELS> inline void DrawTexture(const TextureGL<CHANNELS> &tex, const Point2D &x,
                                                 const LA::Vector2f size) {
    tex.Bind();
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0);                             glVertex2f(x.x() - size.x(), x.y() - size.y());
    glTexCoord2i(0, tex.GetHeight());               glVertex2f(x.x() - size.x(), x.y() + size.y());
    glTexCoord2i(tex.GetWidth(), tex.GetHeight());  glVertex2i(x.x() + size.x(), x.y() + size.y());
    glTexCoord2i(tex.GetWidth(), 0);                glVertex2i(x.x() + size.x(), x.y() - size.y());
    glEnd();
  }
  template<int CHANNELS> inline void DrawTexture(const TextureGL<CHANNELS> &tex,
                                                 const Point3D *Xs) {
    tex.Bind();
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0);                             glVertex3fv(Xs[0]);
    glTexCoord2i(0, tex.GetHeight());               glVertex3fv(Xs[1]);
    glTexCoord2i(tex.GetWidth(), tex.GetHeight());  glVertex3fv(Xs[2]);
    glTexCoord2i(tex.GetWidth(), 0);                glVertex3fv(Xs[3]);
    glEnd();
  }

  static void DrawBox(const Point2D &x1, const Point2D &x2);
  static void DrawBox(const float x, const float y, const float size);
  static void DrawBox(const float x, const float y, const LA::Vector2f &size);

  static void DrawQuad(const Point2D &x1, const Point2D &x2);
  static void DrawQuad(const float x, const float y, const LA::Vector2f &size);

  static void DrawCross(const Point2D &x, const LA::Vector2f &size);
  static void DrawCross(const int x, const int y, const int size);

  static void DrawWireCircle(const Point2D &x, const float r);
  static void DrawSolidCircle(const Point2D &x, const float r);
  static void DrawCovariance(const Point2D &x, const Point2DCovariance &S, const float X2);
  static void PrepareCircle();

 protected:

  virtual void Draw(const bool swapBuffers = true);
  virtual void SwapBuffers();

  virtual bool OnKeyDown(const int key);
  virtual bool OnKeyUp(const int key);
  virtual void OnMouseMove(const CVD::ImageRef &where);
  virtual void OnMouseDown(const CVD::ImageRef &where, const int button);
  virtual void OnMouseUp(const CVD::ImageRef &where, const int button);
  virtual bool OnMouseDraging(const CVD::ImageRef &from, const CVD::ImageRef &to, const int button);
  virtual bool OnMouseDoubleClicked(const CVD::ImageRef &where, const int button);
  virtual void OnResize(const CVD::ImageRef &size);

 protected:

  CVD::GLWindow *m_pWnd;
  EventHandler<Viewer> m_handler;
#ifdef __linux__
  bool m_ctrlDown;
#endif

 private:

  static std::vector<LA::Vector2f> g_xsCircle;
};

#endif
