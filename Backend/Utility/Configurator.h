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
#ifndef _CONFIGURATOR_H_
#define _CONFIGURATOR_H_

#include "Utility.h"
#include <map>

class Configurator {

 public:

  inline Configurator() {}
  inline Configurator(const char *fileName) { Load(fileName); }
  inline Configurator(const std::string& fileName) { Load(fileName.c_str()); }

  inline std::string GetArgument(const std::string directive,
                                 const std::string default_val = "") const {
    const DirectiveArgumentMap::const_iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      return default_val;
    } else {
      return i->second;
    }
  }
  inline int GetArgument(const std::string directive, const int default_val) const {
    const DirectiveArgumentMap::const_iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      return default_val;
    } else {
      return atoi(i->second.c_str());
    }
  }
  inline float GetArgument(const std::string directive, const float default_val) const {
    const DirectiveArgumentMap::const_iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      return default_val;
    } else {
      return float(atof(i->second.c_str()));
    }
  }
  inline double GetArgument(const std::string directive, const double default_val) const {
    const DirectiveArgumentMap::const_iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      return default_val;
    } else {
      return double(atof(i->second.c_str()));
    }
  }
  inline float GetArgumentInverse(const std::string directive, const float default_val) const {
    const float argument = GetArgument(directive, UT::Inverse(default_val));
    return UT::Inverse(argument);
  }
  inline float GetArgumentScaled(const std::string directive, const float default_val,
                                 const float scalar) const {
    const float argument = GetArgument(directive, default_val / scalar);
    return argument * scalar;
  }
  inline int GetArgumentScaled(const std::string directive, const int default_val,
                               const float scalar) const {
    const float argument = GetArgument(directive, default_val / scalar);
    return int(argument * scalar + 0.5f);
  }
  inline float GetArgumentSquared(const std::string directive, const float default_val) const {
    const float argument = GetArgument(directive, sqrtf(default_val));
    return argument * argument;
  }
  inline float GetArgumentScaledSquared(const std::string directive, const float default_val,
                                        const float scalar) const {
    const float argument = GetArgumentScaled(directive, sqrtf(default_val), scalar);
    return argument * argument;
  }
  inline float GetArgumentRadian(const std::string directive, const float default_val) const {
    return GetArgumentScaled(directive, default_val, UT_FACTOR_DEG_TO_RAD);
  }
  inline float GetArgumentRadianSquared(const std::string directive, const float default_val) const {
    return GetArgumentScaledSquared(directive, default_val, UT_FACTOR_DEG_TO_RAD);
  }

  inline void SetArgument(const std::string directive, const std::string argument) {
    DirectiveArgumentMap::iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, argument));
    } else {
      i->second = argument;
    }
  }
  inline void SetArgument(const std::string directive, const int argument) {
    char buf[UT_STRING_WIDTH_MAX];
    sprintf(buf, "%d", argument);
    DirectiveArgumentMap::iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
    } else {
      i->second = buf;
    }
  }
  inline void SetArgument(const std::string directive, const float argument) {
    char buf[UT_STRING_WIDTH_MAX];
    sprintf(buf, "%f", argument);
    DirectiveArgumentMap::iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
    } else {
      i->second = buf;
    }
  }
  inline void SetArgument(const std::string directive, const double argument) {
    char buf[UT_STRING_WIDTH_MAX];
    sprintf(buf, "%lf", argument);
    DirectiveArgumentMap::iterator i = m_directiveArgumentMap.find(directive);
    if (i == m_directiveArgumentMap.end()) {
      m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
    } else {
      i->second = buf;
    }
  }

  inline bool Load(const char *fileName) {
    m_directiveArgumentMap.clear();
    FILE *fp = fopen(fileName, "r");
    if (!fp) {
      return false;
    }
    char line[UT_STRING_WIDTH_MAX], *end;
    while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
      end = std::remove(line, line + strlen(line), 10);
      end = std::remove(line, end, ' ');
      end = std::remove(line, end, 13);
      const int len = static_cast<int>(end - line);
      if (len < 2 || (line[0] == '/' && line[1] == '/')) {
        continue;
      }
      line[len] = 0;
      const std::string directive = strtok(line, "=");
      const char *argument = strtok(NULL, "=");
      if (argument) {
        m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, argument));
      }
    }
    fclose(fp);
    UT::PrintLoaded(fileName);
    return true;
  }

  void Print() const {
    UT::Print("[Configurator]\n");
    for (DirectiveArgumentMap::const_iterator i = m_directiveArgumentMap.begin();
         i != m_directiveArgumentMap.end(); i++) {
      UT::Print("  %s = %s\n", i->first.c_str(), i->second.c_str());
    }
  }

 private:

  typedef std::map<std::string, std::string> DirectiveArgumentMap;
  DirectiveArgumentMap m_directiveArgumentMap;

};

#endif
