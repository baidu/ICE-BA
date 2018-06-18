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

#ifndef XP_HELPER_TIMER_H
#define XP_HELPER_TIMER_H

#include <chrono>
#include <string>

namespace XP {
// Create a scoped timer. Result will be printed to vlog
class ScopedMicrosecondTimer {
 public:
  explicit ScopedMicrosecondTimer(const std::string& text_id, int vlog_level);
  ~ScopedMicrosecondTimer();
 private:
  const std::string text_id_;
  const int vlog_level_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
};

// Create a scoped timer.
// The construction time and the deconstruction timestamp will be logged
class ScopedLoopProfilingTimer {
public:
  explicit ScopedLoopProfilingTimer(const std::string& text_id, int vlog_level);
  ~ScopedLoopProfilingTimer();
private:
  const std::string text_id_;
  const int vlog_level_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
};

// Create a timer. Result will be printed to vlog
class MicrosecondTimer {
 public:
  explicit MicrosecondTimer(const std::string& text_id, int vlog_level);
  MicrosecondTimer();
  int end();
  ~MicrosecondTimer();
 private:
  bool has_ended_;
  const std::string text_id_;
  const int vlog_level_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
};

}  // namespace XP
#endif  // XP_HELPER_TIMER_H
