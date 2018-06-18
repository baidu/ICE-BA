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
#ifndef _EVENT_HANDLER_H_
#define _EVENT_HANDLER_H_

#include <cvd/glwindow.h>
#ifdef WIN32
#define VW_KEY_QUIT   27
#else
#define VW_KEY_QUIT   0xff1b  // XK_Escape
#endif

#define VW_MOUSE_BUTTON_INVALID   -1
#define VW_MOUSE_BUTTON_LEFT    1
#define VW_MOUSE_BUTTON_RIGHT   4
#define VW_MOUSE_BUTTON_MIDDLE_UP 32
#define VW_MOUSE_BUTTON_MIDDLE_DOWN 64

template<class HANDLER>
class EventHandler : public CVD::GLWindow::EventHandler {

 public:

  typedef bool (HANDLER::*CallBackKeyDown)     (const int key);
  typedef bool (HANDLER::*CallBackKeyUp)       (const int key);
  typedef void (HANDLER::*CallBackMouseMove)     (const CVD::ImageRef &where);
  typedef void (HANDLER::*CallBackMouseDown)     (const CVD::ImageRef &where, const int button);
  typedef void (HANDLER::*CallBackMouseUp)     (const CVD::ImageRef &where, const int button);
  typedef bool (HANDLER::*CallBackMouseDraging)  (const CVD::ImageRef &from, const CVD::ImageRef &to,
                                                  const int button);
  typedef bool (HANDLER::*CallBackMouseDoubleClick)(const CVD::ImageRef &where, const int button);
  typedef void (HANDLER::*CallBackResize)      (const CVD::ImageRef &size);

 public:

  inline EventHandler() : m_quit(false), m_keyDown(false), m_mouseDown(false), m_handler(NULL),
    m_keyDownCB(NULL), m_keyUpCB(NULL), m_mouseMoveCB(NULL),
    m_mouseDownCB(NULL), m_mouseUpCB(NULL), m_mouseDragingCB(NULL), m_mouseDoubleClickCB(NULL),
    m_resizeCB(NULL) {}

  /// Called for key press events
  virtual void on_key_down(CVD::GLWindow &win, int key) {
    m_keyDown = true;
    m_key = key;
    m_quit = (key == VW_KEY_QUIT);
    if (m_handler && m_keyDownCB)
      (m_handler->*m_keyDownCB)(key);
  }
  /// Called for key release events
  virtual void on_key_up(CVD::GLWindow &win, int key) {
    m_keyDown = false;
    m_key = -1;
    if (m_handler && m_keyUpCB)
      (m_handler->*m_keyUpCB)(key);
  }
  /// Called for mouse movement events
  virtual void on_mouse_move(CVD::GLWindow &win, CVD::ImageRef where, int state) {
    m_mouseDraging = m_mouseDown && where != m_whereMouseMove;
    m_whereMouseMove = where;
    if (m_mouseDown && m_handler && m_mouseDragingCB)
      (m_handler->*m_mouseDragingCB)(m_whereMouseDown, where, m_button);
    else if (m_handler && m_mouseMoveCB)
      (m_handler->*m_mouseMoveCB)(where);
  }
  /// Called for mouse button press events
  virtual void on_mouse_down(CVD::GLWindow &win, CVD::ImageRef where, int state, int button) {
    m_mouseDown = true;
    m_button = button;
    if (m_whereMouseDown == where) // Double click
      m_whereMouseDown.x = -1;
    else {
      m_whereMouseDown = where;
      if (m_handler && m_mouseDownCB)
        (m_handler->*m_mouseDownCB)(where, button);
    }
  }
  /// Called for mouse button release events
  virtual void on_mouse_up(CVD::GLWindow &win, CVD::ImageRef where, int state, int button) {
    if ((button == VW_MOUSE_BUTTON_LEFT || button == VW_MOUSE_BUTTON_RIGHT) &&
        m_whereMouseDown.x == -1) { //Double click
      if (m_handler && m_mouseDoubleClickCB)
        (m_handler->*m_mouseDoubleClickCB)(where, button);
    } else {
      if (m_handler && m_mouseUpCB)
        (m_handler->*m_mouseUpCB)(where, button);
    }
    m_mouseDown = false;
    m_mouseDraging = false;
  }
  /// Called for window resize events
  virtual void on_resize(CVD::GLWindow &win, CVD::ImageRef size) {
    if (m_handler && m_resizeCB)
      (m_handler->*m_resizeCB)(size);
  }
  /// Called for general window events (such as EVENT_CLOSE)
  virtual void on_event(CVD::GLWindow &win, int event) {
    m_quit = (event == CVD::GLWindow::EVENT_CLOSE);
  }

  inline void Initialize(HANDLER *handler, CallBackKeyDown keyDownCB, CallBackKeyUp keyUpCB,
                         CallBackMouseMove mouseMoveCB, CallBackMouseDown mouseDownCB,
                         CallBackMouseUp mouseUpCB, CallBackMouseDraging mouseDragingCB,
                         CallBackMouseDoubleClick mouseDoubleClickCB, CallBackResize resizeCB) {
    m_quit = false;
    m_keyDown = false;
    m_mouseDown = false;
    m_handler = handler;
    m_keyDownCB = keyDownCB;
    m_keyUpCB = keyUpCB;
    m_mouseMoveCB = mouseMoveCB;
    m_mouseDownCB = mouseDownCB;
    m_mouseUpCB = mouseUpCB;
    m_mouseDragingCB = mouseDragingCB;
    m_mouseDoubleClickCB = mouseDoubleClickCB;
    m_resizeCB = resizeCB;
  }

  inline void Restart() { m_quit = false; }
  inline void Quit() { m_quit = true; }
  inline const bool Quitted() const { return m_quit; }
  inline const bool& MouseDown() const { return m_mouseDown; }
  inline const bool& MouseDraging() const { return m_mouseDraging; }
  inline const CVD::ImageRef& MouseMove() const { return m_whereMouseMove; }

 protected:

  bool m_quit, m_keyDown, m_mouseDown, m_mouseDraging;
  int m_key, m_button;
  CVD::ImageRef m_whereMouseDown, m_whereMouseMove;
  HANDLER *m_handler;
  CallBackKeyDown      m_keyDownCB;
  CallBackKeyUp      m_keyUpCB;
  CallBackMouseMove    m_mouseMoveCB;
  CallBackMouseDown    m_mouseDownCB;
  CallBackMouseUp      m_mouseUpCB;
  CallBackMouseDraging   m_mouseDragingCB;
  CallBackMouseDoubleClick m_mouseDoubleClickCB;
  CallBackResize       m_resizeCB;

};

#endif
