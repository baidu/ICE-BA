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
#ifndef _MULTI_THREAD_H_
#define _MULTI_THREAD_H_

// TODO(mingyu): Update to use c++11 std::mutex & lock
#define MT_SCOPE_LOCK_BEGIN(m) { boost::mutex::scoped_lock sl(m);
#define MT_SCOPE_LOCK_END(m) }
#ifdef CFG_DEBUG_MT
#define MT_READ_LOCK_BEGIN(m, iFrm, iTask) MT::Begin(iFrm, iTask);\
                                           { boost::shared_lock<boost::shared_mutex> rl(m);\

#define MT_READ_LOCK_END(m, iFrm, iTask) } MT::End(iFrm, iTask);

#define MT_WRITE_LOCK_BEGIN(m, iFrm, iTask) MT::Begin(iFrm, iTask);\
                                            { boost::upgrade_lock<boost::shared_mutex> wl1(m);\
                                              boost::upgrade_to_unique_lock<boost::shared_mutex> wl2(wl1);\

#define MT_WRITE_LOCK_END(m, iFrm, iTask) } MT::End(iFrm, iTask);
#else
#define MT_READ_LOCK_BEGIN(m, iFrm, iTask) { boost::shared_lock<boost::shared_mutex> rl(m);
#define MT_READ_LOCK_END(m, iFrm, iTask) }
#define MT_WRITE_LOCK_BEGIN(m, iFrm, iTask) { boost::upgrade_lock<boost::shared_mutex> wl1(m);\
                                              boost::upgrade_to_unique_lock<boost::shared_mutex> wl2(wl1);\

#define MT_WRITE_LOCK_END(m, iFrm, iTask) }
#endif

#define MT_FLAG_DEFAULT 0
#define MT_FLAG_SAVE    1
#define MT_FLAG_LOAD    2

#define MT_TASK_NONE                      -1
#define MT_TASK_LM_IBA_Reset              0
#define MT_TASK_LM_IBA_PushCurrentFrame   1
#define MT_TASK_LM_IBA_Synchronize        2
#define MT_TASK_LM_LBA_Update             3
#define MT_TASK_GM_LBA_Reset              4
#define MT_TASK_GM_LBA_Push               5
#define MT_TASK_GM_LBA_Synchronize        6
#define MT_TASK_GM_GBA_Update             7
#define MT_TASK_LBA_Reset                 8
#define MT_TASK_LBA_PushCurrentFrame      9
#define MT_TASK_LBA_GetCamera             10
#define MT_TASK_LBA_SynchronizeData       11
#define MT_TASK_LBA_UpdateData            12
#define MT_TASK_LBA_BufferDataEmpty       13
#define MT_TASK_GBA_Reset                 14
#define MT_TASK_GBA_PushKeyFrame          15
#define MT_TASK_GBA_UpdateCameras         16
#define MT_TASK_GBA_PushCameraPriorPose   17
#define MT_TASK_GBA_PushCameraPriorMotion 18
#define MT_TASK_GBA_SynchronizeData       19
#define MT_TASK_GBA_BufferDataEmpty       20

#include "Utility.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/shared_mutex.hpp>
#ifdef __ARM_NEON__
#include <sched.h>
#include <string.h>
#include <unistd.h>
#endif  // __ARM_NEON__

#ifdef WIN32
#include "Windows.h"
#else
#include <thread>
// TODO(mingyu): __CYGWIN__ for now uses pthread API instead of Win32 API
#endif

namespace MT {

#ifdef CFG_DEBUG_MT
void Start(const std::string fileName, const int flag = MT_FLAG_DEFAULT);
void Stop();
void Begin(const int iFrm, const int task);
void End(const int iFrm, const int task);
void Synchronize(const int iFrm, const int task, const std::string str = "");
void Print(const int iFrm, const int task, const std::string str = "");
void SaveB(FILE *fp);
void LoadB(FILE *fp);
#endif

class Thread {

 public:

  virtual void Initialize(const int serial = 0, const int iCore = -1, const std::string name = "") {
    m_serial = serial;
    m_serialCnt = 0;
    m_iCore = iCore;
    m_name = name;
  }

  virtual void Reset() {}

  virtual void Start() {
    Reset();
    MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    m_busy = 0;
    m_stop = 0;
    MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
#ifdef CFG_SERIAL
    if (m_serial > 0) {
      m_serialCnt = m_serial - 1;
    } else
#endif  // CFG_SERIAL
    {
#ifdef WIN32
      // Run here is the static function
      CloseHandle(CreateThread(NULL, 0, Run, this, 0, NULL));
#else
      m_thread = std::thread(&Thread::Spin, this);
#endif  // WIN32
    }
  }

  virtual void Stop() {
#ifdef CFG_SERIAL
    if (m_serial > 0) {
      return;
    }
#endif
    MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    if (m_stop == 2) {
      return;
    }
    MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    m_stop = 1;
    m_CDT.notify_all();
    MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);

    MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    while (m_stop != 2) {
      m_CDT.wait(rl);
    }
    MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
#ifndef WIN32
    m_thread.join();
#endif
  }

  virtual void WakeUp() {
#ifdef CFG_SERIAL
    if (m_serial > 0) {
      if (++m_serialCnt == m_serial) {
        m_serialCnt = 0;
        Run();
      }
      return;
    }
#endif
    //printf("%s woke up\n", m_name.c_str());
    m_CDT.notify_one();
    //m_CDT.notify_all();
  }

  virtual void Run() = 0;
  virtual bool BufferDataEmpty() = 0;
  virtual void SaveB(FILE *fp) { UT::SaveB(m_serialCnt, fp); }
  virtual void LoadB(FILE *fp) { UT::LoadB(m_serialCnt, fp); }

#ifdef WIN32
  // TODO(mingyu): Rename to "Spin" to avoid confusion with Run()
  static inline DWORD WINAPI Run(LPVOID pParam) {
    Thread *pThread = (Thread *) pParam;
    while (1) {
      MT_READ_LOCK_BEGIN(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      while (pThread->BufferDataEmpty() && pThread->m_stop == 0) {
        //printf("%s waiting...(empty)\n", pThread->m_name.c_str());
        pThread->m_CDT.wait(rl);
      }
      if (pThread->m_stop == 1) {
        break;
      }
      MT_READ_LOCK_END(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      MT_WRITE_LOCK_BEGIN(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      pThread->m_busy = 1;
      MT_WRITE_LOCK_END(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      //printf("%s running...", pThread->m_name.c_str());
      pThread->Run();
      //printf("done\n");
      MT_WRITE_LOCK_BEGIN(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      pThread->m_busy = 0;
      pThread->m_CDT.notify_one();
      MT_WRITE_LOCK_END(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
    }
    MT_WRITE_LOCK_BEGIN(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
    pThread->m_stop = 2;
    pThread->m_CDT.notify_all();
    MT_WRITE_LOCK_END(pThread->m_MT, MT_TASK_NONE, MT_TASK_NONE);
    return 0;
  }
#else
  void Spin() {
#ifdef __ARM_NEON__
    if (m_iCore >= -1) {
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(m_iCore, &set);
      if (0 != sched_setaffinity(getpid(), sizeof(cpu_set_t), &set)) {
        exit(1);
      }
      UT::Print("Bound thread %s to CPU CORE: #%d", m_name.c_str(), m_iCore);
    }
#endif  // __ARM_NEON__
    //UT::Print("[%s] Run1\n", this->m_name.c_str());
    while (1) {
      MT_READ_LOCK_BEGIN(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      while (this->BufferDataEmpty() && this->m_stop == 0) {
        //printf("%s waiting...(empty)\n", this->m_name.c_str());
        this->m_CDT.wait(rl);
      }
      if (this->m_stop == 1) {
        break;
      }
      MT_READ_LOCK_END(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      MT_WRITE_LOCK_BEGIN(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      this->m_busy = 1;
      MT_WRITE_LOCK_END(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      //printf("%s running...", this->m_name.c_str());
      this->Run();
      //printf("done\n");
      MT_WRITE_LOCK_BEGIN(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
      this->m_busy = 0;
      this->m_CDT.notify_one();
      MT_WRITE_LOCK_END(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
    }
    MT_WRITE_LOCK_BEGIN(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
    this->m_stop = 2;
    this->m_CDT.notify_all();
    MT_WRITE_LOCK_END(this->m_MT, MT_TASK_NONE, MT_TASK_NONE);
  }
#endif  // WIN32

  virtual void Synchronize() {
#ifdef CFG_SERIAL
    if (m_serial > 0) {
      return;
    }
#endif
    MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
    while (m_busy || !BufferDataEmpty()) {
      //printf("%s waiting...(not empty)\n", m_name.c_str());
      m_CDT.wait(rl);
    }
    MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  }

 protected:

  boost::condition m_CDT;
  boost::shared_mutex m_MT;

  int m_busy, m_stop;

  int m_serial, m_serialCnt;
  int m_iCore;
  std::string m_name;
#ifndef WIN32
  std::thread m_thread;
#endif
};

}  // namespace MT

#endif
