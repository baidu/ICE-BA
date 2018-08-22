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
#include "stdafx.h"
#include "Viewer.h"

std::vector<LA::Vector2f> Viewer::g_xsCircle;

#ifdef WIN32
#include <windows.h>
#elif defined(__linux__)
#include <X11/Xlib.h>
#include <X11/extensions/Xrandr.h>
#else
// TODO(Mingyu): support OSX later on
#endif

void Viewer::Initialize(const int w, const int h) {
  if (!m_pWnd) {
#ifndef WIN32
    // TODO(mingyu): Why no need to glutInit on Windows?
    int argc = 0;
    char** argv;
    glutInit(&argc, argv);
#endif
    m_pWnd = new CVD::GLWindow(CVD::ImageRef(w, h));
    int x_pos, y_pos;
#ifdef WIN32
    x_pos = (GetSystemMetrics(SM_CXSCREEN) - w) / 2;
    y_pos = (GetSystemMetrics(SM_CYSCREEN) - h) / 2;
#elif defined(__linux__)
    int num_sizes;
    Display *dpy = XOpenDisplay(NULL);
    Window root = RootWindow(dpy, 0);
    XRRScreenSize *xrrs = XRRSizes(dpy, 0, &num_sizes);
    XRRScreenConfiguration *conf = XRRGetScreenInfo(dpy, root);
    Rotation original_rotation;
    SizeID original_size_id = XRRConfigCurrentConfiguration(conf, &original_rotation);
    x_pos = (xrrs[original_size_id].width - w) / 2;
    y_pos = (xrrs[original_size_id].height - h) / 2;
    XCloseDisplay(dpy);
#else
    // TODO(mingyu): OSX support
#endif
    m_pWnd->set_position(CVD::ImageRef(x_pos, y_pos));
    glewInit();
  }
  OnResize(CVD::ImageRef(w, h));
  m_handler.Initialize(this, &Viewer::OnKeyDown, &Viewer::OnKeyUp, &Viewer::OnMouseMove,
                       &Viewer::OnMouseDown, &Viewer::OnMouseUp, &Viewer::OnMouseDraging,
                       &Viewer::OnMouseDoubleClicked, &Viewer::OnResize);
}

bool Viewer::Run(const bool visualize) {
  m_handler.Restart();
  while (!m_handler.Quitted()) {
    Draw();
  }
  return !m_handler.Quitted();
}

int Viewer::GetCharacterWidth(void *front, const char c) {
  return glutBitmapWidth(front, c);
}

int Viewer::GetStringWidth(void *font, const char *str) {
  return glutBitmapLength(font, (unsigned char *) str);
}

int Viewer::GetStringHeight(void *font) {
  if (font == GLUT_BITMAP_HELVETICA_10)
    return 10;
  else if (font == GLUT_BITMAP_HELVETICA_12)
    return 12;
  else if (font == GLUT_BITMAP_HELVETICA_18)
    return 18;
  else
    UT::Error("Invalid Font!\n");
  return -1;
}

int Viewer::GetStringYTopLeft(const int row, void *font) {
  return VW_STRING_BORDER_Y + VW_STRING_HEIGHT + (VW_STRING_GAP_Y + GetStringHeight(font)) * row;
}

void Viewer::DrawString(void *font, const char *str) {
  for (const char *c = str; *c; c++)
    glutBitmapCharacter(font, *c);
}

void Viewer::DrawStringCurrent(const char *format, ...) {
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(VW_STRING_FONT, str);
}

void Viewer::DrawStringCurrent(void *font, const char *format, ...) {
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(font, str);
}

void Viewer::DrawStringAt(const int x, const int y, void *font, const char *format, ...) {
  glRasterPos2i(x, y);
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(font, str);
}

void Viewer::DrawStringTopLeft(const char *format, ...) {
  glRasterPos2i(VW_STRING_BORDER_X, VW_STRING_BORDER_Y + VW_STRING_HEIGHT);
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(VW_STRING_FONT, str);
}

void Viewer::DrawStringTopLeft(const int row, void *font, const char *format, ...) {
#ifdef CFG_DEBUG
  UT_ASSERT(row >= 1);
#endif
  glRasterPos2i(VW_STRING_BORDER_X, GetStringYTopLeft(row, font));
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(font, str);
}

void Viewer::DrawStringBottomLeft(const char *format, ...) {
  glRasterPos2i(VW_STRING_BORDER_X, m_pWnd->size().y - VW_STRING_BORDER_Y);
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  DrawString(VW_STRING_FONT, str);
}

//void Viewer::DrawStringBottomLeft(const int row, const char *format, ...)
//{
//  glRasterPos2i(VW_STRING_BORDER_X, m_pWnd->size().y - VW_STRING_BORDER_Y - VW_STRING_HEIGHT * row);
//  char str[UT_STRING_WIDTH_MAX];
//  UT_GET_STRING(format, str);
//  DrawString(VW_STRING_FONT, str);
//}

void Viewer::DrawBox(const Point2D &x1, const Point2D &x2) {
  glBegin(GL_LINE_LOOP);
  glVertex2f(x1.x(), x1.y());
  glVertex2f(x1.x(), x2.y());
  glVertex2f(x2.x(), x2.y());
  glVertex2f(x2.x(), x1.y());
  glEnd();
}

void Viewer::DrawBox(const float x, const float y, const float size) {
  DrawBox(Point2D(x - size, y - size), Point2D(x + size, y + size));
}

void Viewer::DrawBox(const float x, const float y, const LA::Vector2f &size) {
  DrawBox(Point2D(x - size.x(), y - size.y()), Point2D(x + size.x(), y + size.y()));
}

void Viewer::DrawQuad(const Point2D &x1, const Point2D &x2) {
  glBegin(GL_QUADS);
  glVertex2f(x1.x(), x1.y());
  glVertex2f(x1.x(), x2.y());
  glVertex2f(x2.x(), x2.y());
  glVertex2f(x2.x(), x1.y());
  glEnd();
}

void Viewer::DrawQuad(const float x, const float y, const LA::Vector2f &size) {
  DrawQuad(Point2D(x - size.x(), y - size.y()), Point2D(x + size.x(), y + size.y()));
}

void Viewer::DrawCross(const Point2D &x, const LA::Vector2f &size) {
  glBegin(GL_LINES);
  glVertex2f(x.x() - size.x(), x.y());
  glVertex2f(x.x() + size.x(), x.y());
  glVertex2f(x.x(), x.y() - size.y());
  glVertex2f(x.x(), x.y() + size.y());
  glEnd();
}

void Viewer::DrawCross(const int x, const int y, const int size) {
  glBegin(GL_LINES);
  glVertex2i(x - size, y);
  glVertex2i(x + size, y);
  glVertex2i(x, y - size);
  glVertex2i(x, y + size);
  glEnd();
}

void Viewer::DrawWireCircle(const Point2D &x, const float r) {
  PrepareCircle();

  float dx, dy;
  glBegin(GL_LINE_LOOP);
  const int N = int(g_xsCircle.size());
  for (int i = 0; i < N; ++i) {
    dx = r * g_xsCircle[i].v0();
    dy = r * g_xsCircle[i].v1();
    glVertex2f(dx + x.x(), dy + x.y());
  }
  glEnd();
}

void Viewer::DrawSolidCircle(const Point2D &x, const float r) {
  PrepareCircle();

  float dx, dy;
  glBegin(GL_TRIANGLE_FAN);
  glVertex2fv(x);
  const int N = int(g_xsCircle.size());
  //for(int i = 0; i < N; ++i)
  for (int i = N - 1; i >= 0; --i) {
    dx = r * g_xsCircle[i].v0();
    dy = r * g_xsCircle[i].v1();
    glVertex2f(dx + x.x(), dy + x.y());
  }
  glEnd();
}

void Viewer::DrawCovariance(const Point2D &x, const Point2DCovariance &S, const float X2) {
  PrepareCircle();

  LA::AlignedMatrix2x2f U;
  LA::Vector2f l;
  S.EigenDecompose(U, l, 0.0f);
  l.x() = sqrtf(l.x() * X2);
  l.y() = sqrtf(l.y() * X2);
  xp128f tmp; tmp.vset_all_lane(l.x(), l.y(), l.x(), l.y());
  U.m_00_01_10_11() = tmp * U.m_00_01_10_11();

  glBegin(GL_LINE_LOOP);
  const int N = int(g_xsCircle.size());
  for (int i = 0; i < N; ++i) {
    const LA::Vector2f dx = U * g_xsCircle[i];
    glVertex2f(dx.x() + x.x(), dx.y() + x.y());
  }
  glEnd();
}

void Viewer::PrepareCircle() {
  if (!g_xsCircle.empty())
    return;
  LA::Vector2f csth;
  const float dth = 1.0f * UT_FACTOR_DEG_TO_RAD;
  //const float dth = 5.0f * UT_FACTOR_DEG_TO_RAD;
  //const float dth = 10.0f * UT_FACTOR_DEG_TO_RAD;
  //const float dth = 20.0f * UT_FACTOR_DEG_TO_RAD;
  //const float dth = 90.0f * UT_FACTOR_DEG_TO_RAD;
  for (float th = 0; th < UT_2PI; th = dth + th) {
    csth.v0() = cosf(th);
    csth.v1() = sinf(th);
    g_xsCircle.push_back(csth);
  }
}

void Viewer::Draw(const bool swapBuffers) {
  if (swapBuffers) {
    SwapBuffers();
  }
}

void Viewer::SwapBuffers() {
  m_pWnd->swap_buffers();
  m_pWnd->handle_events(m_handler);
}

bool Viewer::OnKeyDown(const int key) {
  if (key == VW_KEY_QUIT) {
    m_handler.Quit();
    return true;
  }
  return false;
}

bool Viewer::OnKeyUp(const int key) {
#ifdef __linux__
  if (key == VW_KEY_CTRL_KEY_VALUE)
    m_ctrlDown = false;
#endif
  return false;
}

void Viewer::OnMouseMove(const CVD::ImageRef &where) {
}

void Viewer::OnMouseDown(const CVD::ImageRef &where, const int button) {
}

void Viewer::OnMouseUp(const CVD::ImageRef &where, const int button) {
}

bool Viewer::OnMouseDraging(const CVD::ImageRef &from, const CVD::ImageRef &to, const int button) {
  return false;
}

bool Viewer::OnMouseDoubleClicked(const CVD::ImageRef &where, const int button) {
  return false;
}

void Viewer::OnResize(const CVD::ImageRef &size) {
  m_pWnd->set_size(size);
  int x_pos, y_pos;
#ifdef WIN32
  x_pos = (GetSystemMetrics(SM_CXSCREEN) - size.x) / 2;
  y_pos = (GetSystemMetrics(SM_CYSCREEN) - size.y) / 2;
#elif defined(__linux__)
  int num_sizes;
  Display *dpy = XOpenDisplay(NULL);
  Window root = RootWindow(dpy, 0);
  XRRScreenSize *xrrs = XRRSizes(dpy, 0, &num_sizes);
  XRRScreenConfiguration *conf = XRRGetScreenInfo(dpy, root);
  Rotation original_rotation;
  SizeID original_size_id = XRRConfigCurrentConfiguration(conf, &original_rotation);
  x_pos = (xrrs[original_size_id].width - size.x) / 2;
  y_pos = (xrrs[original_size_id].height - size.y) / 2;
  XCloseDisplay(dpy);
#else
  // TODO(mingyu): OSX support
#endif
  m_pWnd->set_position(CVD::ImageRef(x_pos, y_pos));
  glViewport(0, 0, size.x, size.y);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, size.x, size.y, 0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
