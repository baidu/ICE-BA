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
#ifndef _IBA_DATATYPE_H_
#define _IBA_DATATYPE_H_

#include <IBA_config.h>

#define IBA_SERIAL_NONE   0x00000000
#define IBA_SERIAL_LBA    0x00000001
#define IBA_SERIAL_GBA    0x00000100

#define IBA_VERBOSE_NONE  0x00000000
#define IBA_VERBOSE_LBA   0x00000001
#define IBA_VERBOSE_GBA   0x00000100

#define IBA_DEBUG_NONE    0x00000000
#define IBA_DEBUG_LBA     0x00000001
#define IBA_DEBUG_GBA     0x00000100

#define IBA_HISTORY_NONE  0x00000000
#define IBA_HISTORY_LBA   0x00000001
#define IBA_HISTORY_GBA   0x00000100

namespace IBA {

struct Intrinsic {
  float fx, fy;   // focal length
  float cx, cy;   // optic center
  float ds[8];    // distortion parameters
};

struct Calibration {
  int w, h;       // image resolution
  bool fishEye;   // fish eye distortion model
  float Tu[3][4]; // X_cam = Tu * X_imu
  float ba[3];    // initial acceleration bias
  float bw[3];    // initial gyroscope bias
  //float sa[3];
  Intrinsic K;    // intrinsic parameters
#ifdef CFG_STEREO
  float Tr[3][4]; // X_left = Tr * X_right
  Intrinsic Kr;   // intrinsic parameters for right camera
#endif
};

struct CameraPose {
  float R[3][3];  // rotation matrix, R[0][0] = FLT_MAX for unknown camera pose
  float p[3];     // position
};                // for a 3D point in world frame X, its coordinate in camera frame is obtained by R * (X - p)

struct CameraPoseCovariance {
  float S[6][6];  // position + rotation
                  // p = \hat p + \tilde p
                  // R = \hat R * exp(\tilde\theta)
};

struct CameraIMUState {
  CameraPose C;   // camera pose
  float v[3];     // velocity, v[0] = FLT_MAX for unknown velocity
  float ba[3];    // acceleration bias, ba[0] = FLT_MAX for unknown acceleration bias
  float bw[3];    // gyroscope bias, bw[0] = FLT_MAX for unknown gyroscope bias
};

struct Depth {
  float d;   // inverse depth, d = 0 for unknown depth
  float s2;  // variance
};

struct Point2D {
  float x[2];     // feature location in the original image
  float S[2][2];  // covariance matrix in the original image
};

struct Point3D {
  int idx;    // global point index
  float X[3]; // 3D position, X[0] = FLT_MAX for unknown 3D position
};

struct MapPointMeasurement {
  union {
    int iFrm; // frame ID
    int idx;  // global point ID
  };
  inline bool operator < (const MapPointMeasurement &X) const {
    return iFrm < X.iFrm
#ifdef CFG_STEREO
//#if 1
        || iFrm <= X.iFrm && !right && X.right
#endif
        ;
  }
  Point2D x;
#ifdef CFG_STEREO
//#if 1
  ubyte right;
#endif
};

struct MapPoint {
  Point3D X;
  std::vector<MapPointMeasurement> zs;
};

struct FeatureTrack {
  int idx;
  Point2D x;
};

struct IMUMeasurement {
  float a[3];     // acceleration
  float w[3];     // gyroscope
  float t;        // timestamp
};

struct CurrentFrame {
  int iFrm;                             // frame index
  CameraIMUState C;                     // initial camera/IMU state of current frame
  std::vector<MapPointMeasurement> zs;  // feature measurements of current frame
  std::vector<IMUMeasurement> us;       // IMU measurements between last frame and current frame;
                                        // the timestamp of first IMU must be the same as last frame
  float t;                              // timestamp of current frame, should be greater than the timestamp of last IMU
  Depth d;                              // a rough depth estimate for current frame
                                        // (e.g. average over all visible points in current frame)
  std::string fileName;                 // image file name, just for visualization
#ifdef CFG_STEREO
//#if 1
  std::string fileNameRight;
#endif
};

struct KeyFrame {
  int iFrm;                             // frame index, -1 for invalid keyframe
  CameraPose C;                         // initial camera pose of keyframe
  std::vector<MapPointMeasurement> zs;  // feature measurements of keyframe
  std::vector<MapPoint> Xs;             // new map points
  Depth d;                              // a rough depth estimate
};

struct SlidingWindow {
  std::vector<int> iFrms;           // frame indexes of those sliding window frames whose
                                    // camera/IMU state has been updated since last call
  std::vector<CameraIMUState> CsLF; // camera/IMU states corresponding to iFrms
  std::vector<int> iFrmsKF;         // frame indexes of those keyframes whose camera pose
                                    // has been updated since last call
  std::vector<CameraPose> CsKF;     // camera poses corresponding to iFrmsKF
  std::vector<Point3D> Xs;          // updated 3D points since last call
#ifdef CFG_CHECK_REPROJECTION
  std::vector<std::pair<float, float> > esLF, esKF;
#endif
};

struct RelativeConstraint {
  int iFrm1, iFrm2;
  CameraPose T;       // X2 = T * X1 = R * (X1 - p)
  CameraPoseCovariance S;      
};

struct Error {
  float ex;                         // feature reprojection error
  float eur, eup, euv, euba, eubw;  // IMU delta error
  float edr, edp;                   // drift error compared to ground truth
};

struct Time {
  float t;
  int n;
};

}  // namespace IBA

#endif  //  _IBA_DATATYPE_H_
