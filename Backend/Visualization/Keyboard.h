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
#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

class Keyboard {

 public:

  class Binary {
   public:
    inline Binary() {}
    inline Binary(const bool value) : m_value(value) {}
    inline operator const bool& () const { return m_value; }
    inline const bool& operator = (const bool value) { m_value = value; return m_value; }
    inline void Press() { m_value = !m_value; }
    inline void SaveB(FILE *fp) const { UT::SaveB(m_value, fp); }
    inline void LoadB(FILE *fp) { UT::LoadB(m_value, fp); }
   public:
    bool m_value;
  };

  class Switch {
   public:
    inline Switch() {}
    inline Switch(const int value, const std::vector<std::string> &info,
                  const bool verbose = true) {
      Set(value, info, verbose);
    }
    inline operator const int& () const { return m_value; }
    inline const int& operator = (const int value) { m_value = value; return m_value; }
    inline bool operator == (const Switch &key) const { return m_value == key.m_value; }
    inline void Set(const int value, const std::vector<std::string> &info,
                    const bool verbose = true) {
      m_value = value;
      m_info = info;
      m_verbose = verbose;
    }
    inline bool Set(const int value) {
      if (value < 0 || value >= static_cast<int>(m_info.size())) {
        return false;
      }
      m_value = value;
      if (m_verbose) {
        Keyboard::Print("[%s]", m_info[m_value].c_str());
      }
      return true;
    }
    inline bool Press(const bool next = true, const bool circle = true) {
      const int valueBkp = m_value;
      const int N = static_cast<int>(m_info.size());
      do {
        if (next) {
          if (circle) {
            m_value = (m_value + 1) % N;
          } else if (++m_value == N) {
            --m_value;
          }
        } else {
          if (circle) {
            m_value = (m_value - 1 + N) % N;
          } else if (--m_value == -1) {
            ++m_value;
          }
        }
      } while (m_info[m_value] == "");
      if (m_verbose) {
        Keyboard::Print("[%s]", m_info[m_value].c_str());
      }
      return m_value != valueBkp;
    }
    inline void SaveB(FILE *fp) const {
      UT::SaveB(m_value, fp);
      UT::StringsSaveB(m_info, fp);
      UT::SaveB(m_verbose, fp);
    }
    inline void LoadB(FILE *fp) {
      UT::LoadB(m_value, fp);
      UT::StringsLoadB(m_info, fp);
      UT::LoadB(m_verbose, fp);
    }
   public:
    int m_value;
    std::vector<std::string> m_info;
    bool m_verbose;
  };

  class Scalar {
   public:
    inline Scalar() {}
    inline Scalar(const float value, const std::string info, const bool verbose = true,
                  const float incr = 0.0f) { Set(value, info, verbose, incr); }
    inline operator const float& () const { return m_value; }
    inline const float& operator = (const float value) { m_value = value; return m_value; }
    inline void Set(const float value, const std::string info, const bool verbose = true,
                    const float incr = 0.0f) {
      m_value = value;
      m_info = info;
      m_verbose = verbose;
      m_incr = incr;
    }
    inline void Press(const bool incr) {
      if (incr) {
        Increase();
      } else {
        Decrease();
      }
    }
    inline void Increase() {
      if (m_incr == 0.0f) {
        //if (m_value >= 1.0f) {
        //  ++m_value;
        //} else {
        //  m_value = 1.0f / int(1 / m_value + 0.5f - 1.0f);
        //}
        m_value += m_value;
      } else {
        m_value = m_incr + m_value;
      }
      if (m_verbose) {
        Print();
      }
    }
    inline void Decrease() {
      if (m_incr == 0.0f) {
        //if (m_value <= 1.0f)
        //  m_value = 1.0f / int(1 / m_value + 0.5f + 1.0f);
        //else
        //  --m_value;
        m_value *= 0.5f;
        if (m_value < FLT_EPSILON) {
          m_value = FLT_EPSILON;
        } else if (m_verbose) {
          Print();
        }
      } else {
        m_value = -m_incr + m_value;
        if (m_value < FLT_EPSILON) {
          m_value = m_incr;
        } else if (m_verbose) {
          Print();
        }
      }
    }
    inline void Print() const { Keyboard::Print("[%s] = %f", m_info.c_str(), m_value); }
    inline void SaveB(FILE *fp) const {
      UT::SaveB(m_value, fp);
      UT::StringSaveB(m_info, fp);
      UT::SaveB(m_verbose, fp);
      UT::SaveB(m_incr, fp);
    }
    inline void LoadB(FILE *fp) {
      UT::LoadB(m_value, fp);
      UT::StringLoadB(m_info, fp);
      UT::LoadB(m_verbose, fp);
      UT::LoadB(m_incr, fp);
    }
   public:
    float m_value;
    std::string m_info;
    bool m_verbose;
    float m_incr;
  };

  class KeyPair {
   public:
    inline KeyPair() {}
    inline KeyPair(const int key1, const int key2) : m_key1(key1), m_key2(key2) {}
    inline bool operator < (const KeyPair &keyPair) const { return m_key1 < keyPair.m_key1 || m_key1 == keyPair.m_key1 && m_key2 < keyPair.m_key2; }
    inline bool operator == (const KeyPair &keyPair) const { return m_key1 == keyPair.m_key1 && m_key2 == keyPair.m_key2; }
    inline void SaveB(FILE *fp) const { UT::SaveB(*this, fp); }
    inline void LoadB(FILE *fp) { UT::LoadB(*this, fp); }
   public:
    int m_key1, m_key2;
  };

 public:

  inline void Initialize() {
    m_mapBinary.clear();
    m_mapSwitch.clear();
    m_mapScalar.clear();
    m_mapScalarIncr.clear();
    m_mapScalarDecr.clear();
  }

  template<class KEY, class VALUE> inline void Set(const KEY key, const VALUE &value);
  inline void Set(const int keyIncr, const int keyDecr, const Scalar &value) {
#ifdef CFG_DEBUG
    UT_ASSERT(keyIncr != keyDecr);
#endif
    const KeyPair key(keyIncr, keyDecr);
    Set(key, value, m_mapScalar);
    Set(keyIncr, key, m_mapScalarIncr);
    Set(keyDecr, key, m_mapScalarDecr);
  }
  inline void Set(const int keyIncr, const int keyDecr, const float value) {
    Set(KeyPair(keyIncr, keyDecr), value, m_mapScalar);
  }

  template<class VALUE> inline VALUE Get(const int key);
  inline float Get(const int keyIncr, const int keyDecr) {
#ifdef CFG_DEBUG
    UT_ASSERT(Get(keyIncr, m_mapScalarIncr) == Get(keyDecr, m_mapScalarDecr));
#endif
    return Get(Get(keyIncr, m_mapScalarIncr), m_mapScalar).m_value;
  }

  inline bool Press(const int key) {
    const std::map<int, Binary>::iterator itBinary = m_mapBinary.find(key);
    if (itBinary != m_mapBinary.end()) {
      itBinary->second.Press();
      return true;
    }
    const std::map<int, Switch>::iterator itSwitch = m_mapSwitch.find(key);
    if (itSwitch != m_mapSwitch.end()) {
      itSwitch->second.Press();
      return true;
    }
    const std::map<int, KeyPair>::iterator itScalarIncr = m_mapScalarIncr.find(key);
    if (itScalarIncr != m_mapScalarIncr.end()) {
      m_mapScalar.find(itScalarIncr->second)->second.Increase();
      return true;
    }
    const std::map<int, KeyPair>::iterator itScalarDecr = m_mapScalarDecr.find(key);
    if (itScalarDecr != m_mapScalarDecr.end()) {
      m_mapScalar.find(itScalarDecr->second)->second.Decrease();
      return true;
    }
    return false;
  }

  static inline void Print(const char *format, ...) {
    char str[UT_STRING_WIDTH_MAX];
    UT_GET_STRING(format, str);
    UT::Print("\r%s", str);
    for (int i = int(strlen(str)); i < UT_STRING_WIDTH; ++i) {
      UT::Print(" ");
    }
  }

  inline void SaveB(FILE *fp) const {
    SaveB(m_mapBinary, fp);
    SaveB(m_mapSwitch, fp);
    SaveB(m_mapScalar, fp);
    SaveB(m_mapScalarIncr, fp);
    SaveB(m_mapScalarDecr, fp);
  }
  inline void LoadB(FILE *fp) {
    LoadB(m_mapBinary, fp);
    LoadB(m_mapSwitch, fp);
    LoadB(m_mapScalar, fp);
    LoadB(m_mapScalarIncr, fp);
    LoadB(m_mapScalarDecr, fp);
  }

 protected:

  template<class KEY, class VALUE>
  static inline void Set(const KEY key, const VALUE &value, std::map<KEY, VALUE> &map) {
    const typename std::map<KEY, VALUE>::iterator it = map.find(key);
    if (it == map.end()) {
      map.insert(typename std::map<KEY, VALUE>::value_type(key, value));
    } else {
      it->second = value;
    }
  }

  template<class KEY, class VALUE, typename VALUE_TYPE>
  static inline void Set(const KEY key, const VALUE_TYPE &value, std::map<KEY, VALUE> &map) {
#ifdef CFG_DEBUG
    UT_ASSERT(map.find(key) != map.end());
#endif
    map.find(key)->second.m_value = value;
  }

  template<class KEY, class VALUE>
  static inline const VALUE& Get(const KEY key, const std::map<KEY, VALUE> &map) {
#ifdef CFG_DEBUG
    UT_ASSERT(map.find(key) != map.end());
#endif
    return map.find(key)->second;
  }

  template<class KEY, class VALUE>
  static inline void SaveB(const std::map<KEY, VALUE> &map, FILE *fp) {
    const int N = int(map.size());
    UT::SaveB(N, fp);
    for (typename std::map<KEY, VALUE>::const_iterator it = map.begin(); it != map.end(); it++) {
      UT::SaveB(it->first, fp);
      it->second.SaveB(fp);
    }
  }

  template<class KEY, class VALUE>
  static inline void LoadB(std::map<KEY, VALUE> &map, FILE *fp) {
    map.clear();
    KEY key;
    VALUE value;
    const int N = UT::LoadB<int>(fp);
    for (int i = 0; i < N; ++i) {
      UT::LoadB(key, fp);
      value.LoadB(fp);
      map.insert(typename std::map<KEY, VALUE>::value_type(key, value));
    }
  }

 protected:

  std::map<int, Binary> m_mapBinary;
  std::map<int, Switch> m_mapSwitch;
  std::map<KeyPair, Scalar> m_mapScalar;
  std::map<int, KeyPair> m_mapScalarIncr, m_mapScalarDecr;

};

// Template specialization
template<> inline void Keyboard::Set<int, bool>(const int key, const bool &value) { Set(key, Binary(value), m_mapBinary); }
template<> inline void Keyboard::Set<int, Keyboard::Switch>(const int key, const Keyboard::Switch &value) { Set(key, value, m_mapSwitch); }
template<> inline void Keyboard::Set<int, int>(const int key, const int &value) { Set(key, value, m_mapSwitch); }

template<> inline bool Keyboard::Get<bool>(const int key) { return Get(key, m_mapBinary).m_value; }
template<> inline int Keyboard::Get<int>(const int key) { return Get(key, m_mapSwitch).m_value; }

#endif  // _KEYBOARD_H_
