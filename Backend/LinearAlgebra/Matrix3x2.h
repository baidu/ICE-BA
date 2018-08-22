
#ifndef _MATRIX_3x2_H_
#define _MATRIX_3x2_H_

namespace LA {

template<typename TYPE> class Matrix3x2 {
 public:
  inline const TYPE* operator [] (const int i) const { return m_data[i]; }
  inline       TYPE* operator [] (const int i)       { return m_data[i]; }
 public:
  TYPE m_data[3][2];
};

typedef Matrix3x2<float> Matrix3x2f;
typedef Matrix3x2<double> Matrix3x2d;

}

#endif