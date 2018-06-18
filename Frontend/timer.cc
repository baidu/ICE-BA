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
#include "timer.h"
#include <glog/logging.h>

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __HELPER_TIMER_NO_DEBUG__
#endif

namespace XP {
ScopedMicrosecondTimer::ScopedMicrosecondTimer(const std::string& text_id, int vlog_level) :
    text_id_(text_id),
    vlog_level_(vlog_level),
    t_start_(std::chrono::steady_clock::now()) {
}

ScopedMicrosecondTimer::~ScopedMicrosecondTimer () {
#ifndef __HELPER_TIMER_NO_DEBUG__
  VLOG(vlog_level_) << "ScopedTimer " << text_id_ << "=["
      << std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - t_start_).count()
      << "] microseconds";
#endif
}
ScopedLoopProfilingTimer::ScopedLoopProfilingTimer(const std::string& text_id, int vlog_level) :
  text_id_(text_id),
  vlog_level_(vlog_level),
  t_start_(std::chrono::steady_clock::now()){}
ScopedLoopProfilingTimer::~ScopedLoopProfilingTimer() {
  using namespace std::chrono;
  // print timing info even if in release mode
  if (VLOG_IS_ON(vlog_level_)) {
    VLOG(vlog_level_) << "ScopedLoopProfilingTimer " << text_id_
    << " start_end=[" << duration_cast<microseconds>(t_start_.time_since_epoch()).count()
    << " " << duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count() << "]";
  }
}

MicrosecondTimer::MicrosecondTimer(const std::string& text_id, int vlog_level) :
    has_ended_(false),
    text_id_(text_id),
    vlog_level_(vlog_level) {
  t_start_ = std::chrono::steady_clock::now();
}
MicrosecondTimer::MicrosecondTimer() :
    has_ended_(false),
    text_id_(""),
    vlog_level_(99) {
  t_start_ = std::chrono::steady_clock::now();
}
int MicrosecondTimer::end () {
  CHECK(!has_ended_);
  has_ended_ = true;
  int micro_sec_passed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - t_start_).count();
#ifndef __HELPER_TIMER_NO_DEBUG__
  VLOG(vlog_level_) << "Timer " << text_id_ << "=[" << micro_sec_passed << "] microseconds";
#endif
  return micro_sec_passed;
}
MicrosecondTimer::~MicrosecondTimer() {
  if (!has_ended_) {
    VLOG(vlog_level_) << "MicrosecondTimer " << text_id_ << " is not used";
  }
}

}  // namespace XP
