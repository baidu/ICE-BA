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
#include "MultiThread.h"

namespace MT {

#ifdef CFG_DEBUG_MT
static boost::shared_mutex g_MT;
static boost::condition g_CDT;
static int g_flag;
class Task {
 public:
  inline Task() {}
  inline Task(const int iFrm, const int iTask) : m_iFrm(iFrm), m_iTask(iTask) {}
  inline bool operator == (const Task &task) const {
    return m_iFrm == task.m_iFrm && m_iTask == task.m_iTask;
  }
  inline bool operator != (const Task &task) const {
    return m_iFrm != task.m_iFrm || m_iTask != task.m_iTask;
  }
 public:
  int m_iFrm, m_iTask;
};
class TaskHistory {
 public:
  inline TaskHistory() {}
  inline ~TaskHistory() {
    if (!Empty()) {
      if (g_flag & MT_FLAG_SAVE) {
        Save();
      }
    }
  }
  inline void Initialize(const std::string fileName) {
    Clear();
    m_fileName = fileName;
  }
  inline void Clear() { m_fileName = ""; m_tasks.resize(0); }
  inline bool Empty() const { return m_tasks.empty(); }
  inline void Push(const Task &task) {
    m_tasks.push_back(task);
  }
  inline void Pop() { m_tasks.pop_front(); }
  inline const Task& Front() const { return m_tasks.front(); }
  inline int Size() const { return static_cast<int>(m_tasks.size()); }
  inline bool Save() const {
    if (UT::FileExists(m_fileName)) {
      UT::FileCopy(m_fileName, UT::FileNameAppendSuffix(m_fileName));
    }
    FILE *fp = fopen(m_fileName.c_str(), "w");
    if (!fp) {
      return false;
    }
    for (std::list<Task>::const_iterator task = m_tasks.begin(); task != m_tasks.end(); ++task) {
      fprintf(fp, "%d %d\n", task->m_iFrm, task->m_iTask);
    }
    fclose(fp);
    UT::PrintSaved(m_fileName);
    return true;
  }
  inline bool Load() {
    FILE *fp = fopen(m_fileName.c_str(), "r");
    if (!fp) {
      return false;
    }
    Task task;
    m_tasks.resize(0);
    while (UT::Load<int>(task.m_iFrm, fp) && UT::Load<int>(task.m_iTask, fp)) {
      m_tasks.push_back(task);
    }
    fclose(fp);
    UT::PrintLoaded(m_fileName);
    //m_fileName = "";
    return true;
  }
  inline void SaveB(FILE *fp) const { UT::ListSaveB(m_tasks, fp); }
  inline void LoadB(FILE *fp) { UT::ListLoadB(m_tasks, fp); }
 public:
  std::string m_fileName;
  std::list<Task> m_tasks;
};
static TaskHistory g_TH;
static int g_N = 0;

void Start(const std::string fileName, const int flag) {
  MT_WRITE_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  g_TH.Initialize(fileName);
  g_flag = flag;
  if (g_flag & MT_FLAG_LOAD) {
    if (g_TH.Load()) {
      g_N = g_TH.Size();
    } else {
      g_flag &= ~MT_FLAG_LOAD;
    }
  }
  MT_WRITE_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void Stop() {
  MT_WRITE_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  if (!g_TH.Empty()) {
    if (g_flag & MT_FLAG_SAVE) {
      g_TH.Save();
    }
    g_TH.Clear();
  }
  MT_WRITE_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void Begin(const int iFrm, const int iTask) {
  Synchronize(iFrm, iTask, "Begin\t");
}

void End(const int iFrm, const int iTask) {
  Synchronize(iFrm, iTask, "End\t");
}

void Synchronize(const int iFrm, const int iTask, const std::string str) {
  if (iTask == MT_TASK_NONE) {
    return;
  }
  const Task task(iFrm, iTask);
  if (g_flag & MT_FLAG_LOAD) {
    MT_READ_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
    //const std::string _str = str + UT::String("%d\t", g_N - g_TH.Size());
    while (g_TH.Front() != task) {
      //Print(iFrm, iTask, _str);
      //Print(g_TH.Front().m_iFrm, g_TH.Front().m_iTask, "Waiting for\t");
      g_CDT.wait(rl);
    }
    //Print(iFrm, iTask, _str);
    MT_READ_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
    MT_WRITE_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
    g_TH.Pop();
    g_CDT.notify_all();
    MT_WRITE_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  } else if (g_flag & MT_FLAG_SAVE) {
    MT_WRITE_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
    g_TH.Push(task);
    MT_WRITE_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  }
}

void Print(const int iFrm, const int iTask, const std::string str) {
  std::string _str = str + UT::String("%d\t", iTask);
  switch(iTask) {
  case MT_TASK_NONE:  return;
  //case MT_TASK_NONE:                    _str += "NONE";                       break;
  case MT_TASK_LM_IBA_Reset:              _str += "LM_IBA_Reset";               break;
  case MT_TASK_LM_IBA_PushLocalFrame:     _str += "LM_IBA_PushLocalFrame";      break;
  case MT_TASK_LM_IBA_PushKeyFrame:       _str += "LM_IBA_PushKeyFrame";        break;
  case MT_TASK_LM_IBA_DeleteKeyFrame:     _str += "LM_IBA_DeleteKeyFrame";      break;
  case MT_TASK_LM_IBA_Synchronize:        _str += "LM_IBA_Synchronize";         break;
  case MT_TASK_LM_LBA_Update:             _str += "LM_LBA_Update";              break;
  case MT_TASK_GM_LBA_Reset:              _str += "GM_LBA_Reset";               break;
  case MT_TASK_GM_LBA_PushKeyFrame:       _str += "GM_LBA_PushKeyFrame";        break;
  case MT_TASK_GM_LBA_DeleteKeyFrame:     _str += "GM_LBA_DeleteKeyFrame";      break;
  case MT_TASK_GM_LBA_Synchronize:        _str += "GM_LBA_Synchronize";         break;
  case MT_TASK_GM_GBA_Update:             _str += "GM_GBA_Update";              break;
  case MT_TASK_LBA_Reset:                 _str += "LBA_Reset";                  break;
  case MT_TASK_LBA_PushLocalFrame:        _str += "LBA_PushLocalFrame";         break;
  case MT_TASK_LBA_PushKeyFrame:          _str += "LBA_PushKeyFrame";           break;
  case MT_TASK_LBA_PushDeleteKeyFrame:    _str += "LBA_PushDeleteKeyFrame";     break;
  case MT_TASK_LBA_PushUpdateCameras:     _str += "LBA_PushUpdateCameras";      break;
  case MT_TASK_LBA_PushDeleteMapPoints:   _str += "LBA_PushDeleteMapPoints";    break;
  case MT_TASK_LBA_GetCamera:             _str += "LBA_GetCamera";              break;
  case MT_TASK_LBA_SynchronizeData:       _str += "LBA_SynchronizeData";        break;
  case MT_TASK_LBA_UpdateData:            _str += "LBA_UpdateData";             break;
  case MT_TASK_LBA_BufferDataEmpty:       _str += "LBA_BufferDataEmpty";        break;
  case MT_TASK_GBA_Reset:                 _str += "GBA_Reset";                  break;
  case MT_TASK_GBA_PushKeyFrame:          _str += "GBA_PushKeyFrame";           break;
  case MT_TASK_GBA_PushDeleteKeyFrame:    _str += "GBA_PushDeleteKeyFrame";     break;
  case MT_TASK_GBA_PushDeleteMapPoints:   _str += "GBA_PushDeleteMapPoints";    break;
  case MT_TASK_GBA_PushCameraPriorPose:   _str += "GBA_PushCameraPriorPose";    break;
  case MT_TASK_GBA_PushCameraPriorMotion: _str += "GBA_PushCameraPriorMotion";  break;
  case MT_TASK_GBA_PushUpdateCameras:     _str += "GBA_PushUpdateCameras";      break;
  case MT_TASK_GBA_SynchronizeData:       _str += "GBA_SynchronizeData";        break;
  case MT_TASK_GBA_BufferDataEmpty:       _str += "GBA_BufferDataEmpty";        break;
  }
  UT::Print("[%d] %s\n", iFrm, _str.c_str());
}

void SaveB(FILE *fp) {
  MT_READ_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  g_TH.SaveB(fp);
  MT_READ_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void LoadB(FILE *fp) {
  MT_WRITE_LOCK_BEGIN(g_MT, MT_TASK_NONE, MT_TASK_NONE);
  g_TH.LoadB(fp);
  MT_WRITE_LOCK_END(g_MT, MT_TASK_NONE, MT_TASK_NONE);
}
#endif

}
