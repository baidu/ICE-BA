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
#ifndef XP_INCLUDE_XP_DATA_ATOM_BASIC_DATATYPE_H_
#define XP_INCLUDE_XP_DATA_ATOM_BASIC_DATATYPE_H_

#include <Eigen/Core>
#include <vector>

namespace XP {

class ImuData {
 public:
  Eigen::Vector3f accel;
  Eigen::Vector3f ang_v;
  float time_stamp;
 public:
#ifndef __ANDROID__
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

}  // namespace XP
#endif  // XP_INCLUDE_XP_DATA_ATOM_BASIC_DATATYPE_H_
