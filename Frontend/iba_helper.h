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
#ifndef XP_INCLUDE_XP_HELPER_IBA_HELPER_H_
#define XP_INCLUDE_XP_HELPER_IBA_HELPER_H_

#include "param.h"
#include "basic_datatype.h"  // for ImuData
#include <IBA_datatype.h>

namespace XP {

// Helper functions to convert from DuoCalibParam to IBA::Calibration
IBA::Calibration to_iba_calibration(const DuoCalibParam& calib);

IBA::Intrinsic to_iba_intrinsic(const DuoCalibParam::Camera_t& cam, const int lr);

IBA::IMUMeasurement to_iba_imu(const ImuData& imu_in);

}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_IBA_HELPER_H_
