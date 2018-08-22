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
#include "Utility.h"
#include "MultiThread.h"

#ifndef _WIN32
#include <boost/filesystem.hpp>
#endif

#define UT_PRINT_CHECK

namespace UT {
static boost::mutex g_mutex;
//static std::string g_fileName = "";
//static FILE *g_fp = NULL;
class File {
 public:
  inline File() : m_fp(NULL) {}
  inline ~File() { Stop(); }
  inline void Start(const std::string fileName) {
    m_fileName = fileName;
    m_fp = fopen(fileName.c_str(), "w");
    if (m_fp) {
      fclose(m_fp);
    }
  }
  inline void Stop() {
    //if (!m_fp)
    //  return;
    //fclose(m_fp);
    //m_fp = NULL;
    //UT::PrintSaved(m_fileName);
  }
  inline bool Valid() const { return m_fp != NULL; }
  inline void Print(const char *str) {
    if (m_fp) {
      m_fp = fopen(m_fileName.c_str(), "a");
      fprintf(m_fp, "%s", str);
      fclose(m_fp);
    } else {
      printf("%s", str);
    }
  }
 public:
  FILE *m_fp;
  std::string m_fileName;
};
static File g_fp;
static int g_N = 0;

void Assert(const bool expression, const char *format, ...) {
  //assert(expression);
  if (expression) {
    return;
  }
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  Print("%s", str);
  exit(-1);
  //EXPECT_TRUE(expression);
}

static bool g_debug = false;

void DebugStart() {
  PrintSeparator('!');
  Print("Debug...\n");
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  UT_ASSERT(!g_debug);
  g_debug = true;
  MT_SCOPE_LOCK_END(g_mutex);
}

bool Debugging() {
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  return g_debug;
  MT_SCOPE_LOCK_END(g_mutex);
}

void DebugStop() {
  Print("...Debug\n");
  PrintSeparator('!');
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  UT_ASSERT(g_debug);
  g_debug = false;
  MT_SCOPE_LOCK_END(g_mutex);
}

void PrintStart(const std::string fileName) {
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  g_fp.Start(fileName);
  MT_SCOPE_LOCK_END(g_mutex);
}

void PrintStop() {
  bool v;
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  v = g_fp.Valid();
  g_fp.Stop();
  MT_SCOPE_LOCK_END(g_mutex);
  if (v) {
    UT::PrintSaved(g_fp.m_fileName);
  }
}

void Print(const char *format, ...) {
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  //MT_SCOPE_LOCK_BEGIN(g_mutex);
  //g_fp.Print(str);
  //const int len = int(strlen(str));
  //if (len == 0 || str[len - 1] == '\n') {
  //  g_N = 0;
  //} else {
  //  g_N += len;
  //}
  //MT_SCOPE_LOCK_END(g_mutex);
  printf("%s", str);
}

void PrintSeparator(const char c) {
  Print("\r");
  for (int i = 0; i < UT_STRING_WIDTH; ++i) {
    Print("%c", c);
  }
  Print("\n");
}

int PrintStringWidth() {
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  return g_N;
  MT_SCOPE_LOCK_END(g_mutex);
}

void SaveSeparator(FILE *fp, const char c) {
  fprintf(fp, "\r");
  for (int i = 0; i < 79; ++i) {
    fprintf(fp, "%c", c);
  }
  fprintf(fp, "\n");
}

void Error(const char *format, ...) {
  PrintSeparator('!');
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  Print("ERROR: %s", str);
  exit(0);
}

void Check(const char *format, ...) {
#ifdef UT_PRINT_CHECK
  PrintSeparator('!');
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  Print("CHECK: %s", str);
#endif
}


bool FileExists(const std::string fileName) {
#ifdef WIN32
  return _access(fileName.c_str(), 0) == 0;
#else
  return access(fileName.c_str(), 0) == 0;
#endif
}

FILE* FileOpen(const std::string fileName, const char *mode, const char *format, ...) {
  FILE *fp = fopen(fileName.c_str(), mode);
  if (fp) {
    return fp;
  }
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  const std::string _fileName = Input<std::string>(str);
  return fopen(_fileName.c_str(), mode);
}

bool FileCopy(const std::string fileNameSrc, const std::string fileNameDst) {
  if (FileExists(fileNameDst)) {
    return false;
  }
#ifdef _WIN32
  char command[UT_STRING_WIDTH_MAX];
  sprintf(command, "copy %s %s", StringReplace(fileNameSrc, "/", "\\").c_str(),
          StringReplace(fileNameDst, "/", "\\").c_str());
  printf("%s\n", command);
  system(command);
  return true;
#else
  boost::filesystem::copy(fileNameSrc, fileNameDst);
  printf("boost copy %s to %s\n", fileNameSrc.c_str(), fileNameDst.c_str());
  return true;
#endif
}

bool FileDelete(const std::string fileName, const bool check, const bool verbose) {
#ifdef CFG_DEBUG
  UT_ASSERT(UT::FileExists(fileName));
#endif
  if (check) {
    char input[UT_STRING_WIDTH_MAX];
    Print("Delete \'%s\'? (Y/N) ", fileName.c_str());
    scanf("%s", input);
    if (strlen(input) != 1 || (input[0] != 'Y' && input[0] != 'y')) {
      return false;
    }
  }

#ifdef _WIN32
  char command[UT_STRING_WIDTH_MAX];
  sprintf(command, "del %s", fileName.c_str());
  system(command);
#else
  boost::filesystem::remove(fileName);
#endif
  if (verbose) {
    Print("Deleted \'%s\'\n", fileName.c_str());
  }
  return true;
}

// [NOTE] Example usage:
// /path/to/files/ contains log_0000.txt -> log_1000.txt
// 1) UT::FilesSearch("/path/to/files/*", 4, 2, 10, false)
//    Return:  /path/to/files/log_0004.txt   <-- absolute iStart
//             /path/to/files/log_0006.txt   <-- step is 2
//             /path/to/files/log_0008.txt
//             /path/to/files/log_0010.txt   <-- absolute iEnd (inclusive)
// 2) UT::FilesSearch("/path/to/files/log_0001.txt", 5, 2, 11, false)
//    Return:  /path/to/files/log_0006.txt   <-- start at 6 = 1 + 5
//    Return:  /path/to/files/log_0008.txt   <-- step is 2
//    Return:  /path/to/files/log_0010.txt
//    Return:  /path/to/files/log_0012.txt   <-- ends at 12 <= 12 (12 = 1 + 11)
std::vector<std::string> FilesSearch(const std::string fileNameFirst, const int iStart,
                                     const int iStep, const int iEnd, const bool verbose) {
  std::vector<std::string> fileNames;
  if (FileNameRemoveDirectoryExtension(fileNameFirst) == "*") {
    const std::string dir = FileNameExtractDirectory(fileNameFirst);
    if (!FileExists(dir)) {
      return fileNames;
    }
#ifdef _WIN32
    _finddata_t fd;
    intptr_t fh = _findfirst(fileNameFirst.c_str(), &fd);
    if (fh == -1) {
      return fileNames;
    }
    do {
      const std::string fileName = dir + fd.name;
      fileNames.push_back(fileName);
#ifdef CFG_VERBOSE
      if (verbose) {
        printf("\rFound \'%s\'", fileName.c_str());
      }
#endif
    } while (_findnext(fh, &fd) == 0);
#else
    using boost::filesystem::path;
    using boost::filesystem::directory_iterator;
    path filename(fileNameFirst);
    path extension = filename.extension();
    for (directory_iterator it(dir); it != directory_iterator(); ++it) {
      if (boost::filesystem::is_regular_file(it->path()) &&
          it->path().extension() == extension) {
        fileNames.push_back(it->path().string());
      }
    }
    // Boost directory iterator does NOT guarantee order. Sort here!
    sort(fileNames.begin(), fileNames.end());
#ifdef CFG_VERBOSE
    if (verbose) {
      for (const std::string& f : fileNames) {
        printf("\rFound \'%s\'", f.c_str());
      }
    }
#endif
#endif
    int i, j;
    const int N = static_cast<int>(fileNames.size());
    for (i = iStart, j = 0; i < N && i <= iEnd; i += iStep) {
      fileNames[j++] = fileNames[i];
    }
    fileNames.resize(j);
  } else {
    int i = iStart;
    while (i <= iEnd) {
      const std::string fileName = FileNameIncreaseSuffix(fileNameFirst, i);
      if (!FileExists(fileName)) {
        break;
      }
      fileNames.push_back(fileName);
#ifdef CFG_VERBOSE
      if (verbose)
        printf("\rFound \'%s\'", fileName.c_str());
#endif
      if (iStep == 0) {
        break;
      }
      i += iStep;
    }
  }
#ifdef CFG_VERBOSE
  if (verbose) {
    if (fileNames.empty()) {
      printf("Not found \'%s\'", fileNameFirst.c_str());
    }
    printf("\n");
  }
#endif
  return fileNames;
}

std::vector<std::string> FilesSearch(const std::vector<std::string> &fileNames,
                                     const std::string dir, const std::string ext) {
  std::string fileName;
  std::vector<std::string> fileNamesReplace;
  const int N = static_cast<int>(fileNames.size());
  fileNamesReplace.resize(N);
  for (int i = 0; i < N; ++i) {
    fileName = dir + FileNameRemoveDirectoryExtension(fileNames[i]);
    while (FileNameExtractExtension(fileName) != "")
      fileName = FileNameRemoveExtension(fileName);
    fileName += "." + ext;
    fileNamesReplace[i] = FileExists(fileName) ? fileName : "";
  }
  return fileNamesReplace;
}

// If the target dir doesn't exist, create it.  Otherwise, delete all related files.
void FilesStartSaving(const std::string fileNameFirst, const bool check, const bool verbose) {
  const std::string dir = FileNameExtractDirectory(fileNameFirst);
  if (FileExists(dir)) {
    const std::vector<std::string> fileNames = FilesSearch(fileNameFirst, 0, 1, INT_MAX, verbose);
    const int N = static_cast<int>(fileNames.size());
    for (int i = 0; i < N; ++i) {
      if (!FileDelete(fileNames[i], check && i == 0, verbose)) {
        break;
      }
    }
  } else {
#ifdef WIN32
    CreateDirectory(dir.c_str(), 0);
#else
    boost::filesystem::create_directories(dir);
#endif
  }
}

std::string FileNameExtractDirectory(const std::string fileName) {
  const std::string::size_type i1 = fileName.rfind('/'), i2 = fileName.rfind('\\');
  if (i1 == std::string::npos && i2 == std::string::npos) {
    return std::string();
  } else if (i1 != std::string::npos && i2 == std::string::npos) {
    return fileName.substr(0, i1 + 1);
  } else if (i1 == std::string::npos && i2 != std::string::npos) {
    return fileName.substr(0, i2 + 1);
  } else if (i1 > i2) {
    return fileName.substr(0, i1 + 1);
  } else {
    return fileName.substr(0, i2 + 1);
  }
}

std::string FileNameRemoveDirectory(const std::string fileName) {
  const std::string::size_type i1 = fileName.rfind('/'), i2 = fileName.rfind('\\');
  if (i1 == std::string::npos && i2 == std::string::npos) {
    return fileName;
  } else if (i1 != std::string::npos && i2 == std::string::npos) {
    return fileName.substr(i1 + 1, fileName.size());
  } else if (i1 == std::string::npos && i2 != std::string::npos) {
    return fileName.substr(i2 + 1, fileName.size());
  } else if (i1 > i2) {
    return fileName.substr(i1 + 1, fileName.size());
  } else {
    return fileName.substr(i2 + 1, fileName.size());
  }
}

std::string FileNameExtractExtension(const std::string fileName) {
  const std::string::size_type i = fileName.rfind('.');
  if (i == std::string::npos) {
    return std::string();
  } else {
    return fileName.substr(i + 1, fileName.size());
  }
}

std::string FileNameRemoveExtension(const std::string fileName) {
  const std::string::size_type i = fileName.rfind('.');
  if (i == std::string::npos) {
    return fileName;
  } else {
    return fileName.substr(0, i);
  }
}

std::string FileNameRemoveDirectoryExtension(const std::string fileName) {
  return FileNameRemoveDirectory(FileNameRemoveExtension(fileName));
}

std::string FileNameRemovePrefix(const std::string fileName, const std::string prefix) {
  if (fileName == "" || fileName.find(prefix) != 0) {
    return fileName;
  } else {
    return fileName.substr(prefix.length(), fileName.length());
  }
}

std::string FileNameAppendSuffix(const std::string fileName, const std::string suffix) {
  return FileNameRemoveExtension(fileName) + suffix + "." + FileNameExtractExtension(fileName);
}

std::string FileNameAppendSuffix(const std::string fileName, const int suffix) {
  char buf[UT_STRING_WIDTH_MAX];
  sprintf(buf, "_%d", suffix);
  return FileNameAppendSuffix(fileName, buf);
}

std::string FileNameAppendSuffix(const std::string fileName) {
  std::string fileNameAppend = fileName;
  for (int i = 0; FileExists(fileNameAppend); ++i) {
    fileNameAppend = UT::FileNameAppendSuffix(fileName, i);
  }
  return fileNameAppend;
}

std::string FileNameReplaceDirectory(const std::string fileName, const std::string dirSrc,
                                     const std::string dirDst) {
  if (fileName == "" || (dirSrc != "" && fileName.find(dirSrc) != 0)) {
    return fileName;
  } else if (fileName[0] == '.') {
    return dirDst + fileName;
  } else {
    return dirDst + fileName.substr(dirSrc.length(), fileName.length());
  }
}

// Example: abc01.txt -> abc02.txt
std::string FileNameIncreaseSuffix(const std::string fileName, const int incr) {
  const int len = int(fileName.length());
  if (len == 0) {
    return "";
  }
  int i2 = len;
  while (--i2 >= 0 && !isdigit(fileName[i2]));
  int i1 = ++i2;
  while (--i1 >= 0 && isdigit(fileName[i1]));
  const int number = ++i1 == i2 ? incr : atoi(fileName.substr(i1, i2 - i1).c_str()) + incr;
  const int width1 = i2 - i1, width2 = int(log10f(float(number)));
  const int width = width1 > width2 ? width1 : width2;
  char buf[UT_STRING_WIDTH_MAX];
  switch (width) {
  case 0:   return "";
  case 2:   sprintf(buf, "%.2d", number); break;
  case 3:   sprintf(buf, "%.3d", number); break;
  case 4:   sprintf(buf, "%.4d", number); break;
  case 5:   sprintf(buf, "%.5d", number); break;
  case 6:   sprintf(buf, "%.6d", number); break;
  case 7:   sprintf(buf, "%.7d", number); break;
  case 8:   sprintf(buf, "%.8d", number); break;
  case 9:   sprintf(buf, "%.9d", number); break;
  case 10:  sprintf(buf, "%.10d", number);  break;
  default:  sprintf(buf, "%d", number);   break;
  }
  return fileName.substr(0, i1) + buf + fileName.substr(i2, len - i2);
}

std::vector<std::string> Strings(const std::string str0) {
  std::vector<std::string> strs(1);
  strs[0] = str0;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1) {
  std::vector<std::string> strs(2);
  strs[0] = str0;
  strs[1] = str1;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2) {
  std::vector<std::string> strs(3);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3) {
  std::vector<std::string> strs(4);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  strs[3] = str3;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3, const std::string str4) {
  std::vector<std::string> strs(5);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  strs[3] = str3;
  strs[4] = str4;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3, const std::string str4, const std::string str5) {
  std::vector<std::string> strs(6);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  strs[3] = str3;
  strs[4] = str4;
  strs[5] = str5;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3, const std::string str4, const std::string str5,
                                 const std::string str6) {
  std::vector<std::string> strs(7);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  strs[3] = str3;
  strs[4] = str4;
  strs[5] = str5;
  strs[6] = str6;
  return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3, const std::string str4, const std::string str5,
                                 const std::string str6, const std::string str7) {
  std::vector<std::string> strs(8);
  strs[0] = str0;
  strs[1] = str1;
  strs[2] = str2;
  strs[3] = str3;
  strs[4] = str4;
  strs[5] = str5;
  strs[6] = str6;
  strs[7] = str7;
  return strs;
}

std::vector<std::string> Strings(const std::string *strs, const int N) {
  std::vector<std::string> _strs(N);
  for (int i = 0; i < N; ++i)
    _strs[i] = strs[i];
  return _strs;
}

void StringSaveB(const std::string &str, FILE *fp) {
  char buf[UT_STRING_WIDTH_MAX];
  sprintf(buf, "%s\n", str.c_str());
  fwrite(buf, 1, strlen(buf), fp);
}

void StringLoadB(std::string &str, FILE *fp) {
  char buf[UT_STRING_WIDTH_MAX];
  fgets(buf, UT_STRING_WIDTH_MAX, fp);
  const int len = static_cast<int>(strlen(buf));
  if (buf[len - 1] == 10) {
    buf[len - 1] = 0;
  }
  str = buf;
}

std::string String(const char *format, ...) {
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  return std::string(str);
}

std::string StringInput() {
  std::string input;
  MT_SCOPE_LOCK_BEGIN(g_mutex);
  char buf[UT_STRING_WIDTH_MAX];
  fgets(buf, UT_STRING_WIDTH_MAX, stdin);
  input = std::string(buf);
  MT_SCOPE_LOCK_END(g_mutex);
  if (input.back() == 10) {
    input.resize(input.size() - 1);
  }
  return input;
}

std::string StringReplace(const std::string str, const std::string strSrc,
                          const std::string strDst) {
  std::string::size_type pos;
  std::string res = str;
  while (1) {
    if ((pos = res.find(strSrc)) == std::string::npos)
      break;
    res.replace(pos, strSrc.length(), strDst);
  }
  return res;
}

void StringsSaveB(const std::vector<std::string> &strs, FILE *fp) {
  const int size = int(strs.size());
  SaveB<int>(size, fp);
  for (int i = 0; i < size; ++i)
    StringSaveB(strs[i], fp);
}

void StringsLoadB(std::vector<std::string> &strs, FILE *fp) {
  const int size = LoadB<int>(fp);
  strs.resize(size);
  for (int i = 0; i < size; ++i)
    StringLoadB(strs[i], fp);
}

void SaveValues(const std::string fileName, const std::vector<float> &vs) {
  FILE *fp = fopen(fileName.c_str(), "w");
  const int N = int(vs.size());
  for (int i = 0; i < N; ++i) {
    fprintf(fp, "%f\n", vs[i]);
  }
  fclose(fp);
  PrintSaved(fileName);
}

void SaveHistogram(const std::string fileName, const std::vector<float> &vs, const int Nb,
                   const float vMin, const float vMax) {
  float _vMin, _vMax;
  const int N = int(vs.size());
  if (vMin == FLT_MAX) {
    _vMin = FLT_MAX;
    for (int i = 0; i < N; ++i) {
      _vMin = std::min(vs[i], _vMin);
    }
  } else {
    _vMin = vMin;
  }
  if (vMax == FLT_MAX) {
    _vMax = FLT_MAX;
    for (int i = 0; i < N; ++i) {
      _vMax = std::max(vs[i], _vMax);
    }
  } else {
    _vMax = vMax;
  }

  std::vector<int> hs(Nb, 0);
  const float w = (_vMax - _vMin) / (Nb - 1), s = 1.0f / w;
  for (int i = 0; i < N; ++i) {
    const int ib = int((vs[i] - _vMin) * s);
    if (ib >= 0 && ib < Nb) {
      ++hs[ib];
    }
  }

  FILE *fp = fopen(fileName.c_str(), "w");
  float v = _vMin;
  for (int ib = 0; ib < Nb; ++ib, v = w + v) {
    fprintf(fp, "%f %d\n", v, hs[ib]);
  }
  fclose(fp);
  PrintSaved(fileName);
}

}
