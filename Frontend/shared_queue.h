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
#ifndef XP_INCLUDE_XP_HELPER_SHARED_QUEUE_H_
#define XP_INCLUDE_XP_HELPER_SHARED_QUEUE_H_

#include <glog/logging.h>
#include <deque>
#include <mutex>
#include <string>
#include <condition_variable>

namespace XP {

namespace internal {

// Internal helper function to get the last element and then empty the container.
// The caller should be responsible for checking non-empty before pop_to_back.
// std::deque has back()
template <typename T>
void pop_to_back(std::deque<T>* container, T* elem) {
  CHECK(!container->empty());
  *elem = std::move(container->back());
  container->clear();
}

// For other container types that may not have back()
template <typename T, typename OtherContainer>
void pop_to_back(OtherContainer* container, T* elem) {
  CHECK(!container->empty());
  while (container->size() > 1) {
    container->pop_front();
  }
  *elem = std::move(container->front());
  container->pop_front();
}
}  // namespace internal

template <typename T, typename Container = std::deque<T> >
class shared_queue {
 public:
  // Remove copy and assign
  shared_queue& operator=(const shared_queue&) = delete;
  shared_queue(const shared_queue& other) = delete;

  explicit shared_queue(const std::string& name) : name_(name), kill_(false) {}
  ~shared_queue() {
    // make sure you always call kill before destruction
    if (!kill_ && !queue_.empty()) {
      LOG(ERROR) << "shared_queue : " << name_ << " is destructed without getting killed";
    }
  }

  // Use this function to kill the shared_queue before the application exists
  // to prevent potential deadlock.
  void kill() {
    kill_ = true;
    cond_.notify_one();
  }

  T front() {
    std::lock_guard<std::mutex> lock(m_);
    return queue_.front();
  }

  void push_back(T elem) {
    {
      std::lock_guard<std::mutex> lock(m_);
      queue_.push_back(std::move(elem));
    }
    // Unlock mutex m_ before notifying
    cond_.notify_one();
  }

  void pop_front() {
    std::lock_guard<std::mutex> lock(m_);
    if (!queue_.empty()) {
      queue_.pop_front();
    }
  }

  void pop_to_back(T* elem) {
    std::lock_guard<std::mutex> lock(m_);
    internal::pop_to_back(&queue_, elem);
  }

  bool wait_and_pop_front(T* elem) {
    std::unique_lock<std::mutex> lock(m_);
    cond_.wait(lock, [this](){return !queue_.empty() || kill_; });
    if (kill_) {
      return false;
    } else {
      *elem = std::move(queue_.front());
      queue_.pop_front();
      return true;
    }
  }

  bool wait_and_pop_to_back(T* elem) {
    std::unique_lock<std::mutex> lock(m_);
    cond_.wait(lock, [this](){ return !queue_.empty() || kill_; });
    if (kill_) {
      return false;
    } else {
      internal::pop_to_back(&queue_, elem);
      return true;
    }
  }

  bool wait_and_peek_front(T* elem) {
    std::unique_lock<std::mutex> lock(m_);
    cond_.wait(lock, [this](){ return !queue_.empty() || kill_; });
    if (kill_) {
      return false;
    } else {
      *elem = queue_.front();
      return true;
    }
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(m_);
    return queue_.empty();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(m_);
    return queue_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(m_);
    queue_.clear();
  }

 private:
  Container queue_;
  std::mutex m_;
  std::condition_variable cond_;
  std::string name_;
  bool kill_;
};
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_SHARED_QUEUE_H_
