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
//#ifndef CFG_DEBUG
//#define CFG_DEBUG
//#endif
#include "Parameter.h"

int KF_FIRST_LOCAL_FRAMES        = 1;
int KF_MIN_FRAME_STEP            = 0;
int KF_MIN_FEATURE_SROUCES       = 20;
//int KF_MIN_FEATURE_SROUCES     = 50;
int KF_MIN_FEATURE_MEASUREMENTS  = 20;

float FTR_VARIANCE                              = 1.0f;      // 1.0^2
//float FTR_VARIANCE                            = 9.0f;      // 3.0^2
float FTR_VARIANCE_EPSILON                      = 0.01f;     // 0.1^2

  int FTR_UNDIST_MAX_ITERATIONS                 = 10;
float FTR_UNDIST_CONVERGE                       = 0.01f;     // 0.1^2
  int FTR_UNDIST_LUT_SIZE                       = 11;
  int FTR_UNDIST_DL_MAX_ITERATIONS              = 10;
float FTR_UNDIST_DL_RADIUS_INITIAL              = 1.0f;      // 1.0^2
float FTR_UNDIST_DL_RADIUS_MIN                  = 1.0e-10f;  // 0.00001^2
float FTR_UNDIST_DL_RADIUS_MAX                  = 1.0e4f;    // 100.0^2
float FTR_UNDIST_DL_RADIUS_FACTOR_INCREASE      = 9.0f;      // 3.0^2
float FTR_UNDIST_DL_RADIUS_FACTOR_DECREASE      = 0.25f;     // 0.5^2
float FTR_UNDIST_DL_GAIN_RATIO_MIN              = 0.25f;
float FTR_UNDIST_DL_GAIN_RATIO_MAX              = 0.75f;

float IMU_GRAVITY_MAGNITUDE                   = 9.81f;
 bool IMU_GRAVITY_EXCLUDED                    = false;
//bool IMU_GRAVITY_EXCLUDED                   = true;
float IMU_VARIANCE_ACCELERATION_NOISE         = 1.0e-4f;            // 0.01^2
float IMU_VARIANCE_ACCELERATION_BIAS_WALK     = 1.0e-6f;            // 0.001^2
float IMU_VARIANCE_GYROSCOPE_NOISE            = 3.046174198662e-8f; // (0.01*pi/180)^2
float IMU_VARIANCE_GYROSCOPE_BIAS_WALK        = 3.046174198662e-8f; // (0.01*pi/180)^2
float IMU_VARIANCE_EPSILON_ROTATION           = 3.046174198662e-8f; // (0.01*pi/180)^2
float IMU_VARIANCE_EPSILON_VELOCITY           = 1.0e-6f;            // 0.001^2
float IMU_VARIANCE_EPSILON_POSITION           = 1.0e-6f;            // 0.001^2
float IMU_VARIANCE_EPSILON_BIAS_ACCELERATION  = 1.0e-6f;            // 0.001^2
float IMU_VARIANCE_EPSILON_BIAS_GYROSCOPE     = 3.046174198662e-8f; // (0.01*pi/180)^2
float IMU_WEIGHT_ROTATION                     = 1.0f;
float IMU_WEIGHT_VELOCITY                     = 1.0f;
float IMU_WEIGHT_POSITION                     = 1.0f;
float IMU_WEIGHT_BIAS_ACCELERATION            = 1.0f;
float IMU_WEIGHT_BIAS_GYROSCOPE               = 1.0f;

//float DEPTH_MIN                 = 0.0;
float DEPTH_MIN                   = 0.02f;    // 1/50.0
float DEPTH_MAX                   = 10.0f;    // 1/0.1
float DEPTH_RANGE                 = 9.98f;    // DEPTH_MAX - DEPTH_MIN
float DEPTH_INITIAL               = 0.2f;     // 1/5.0
float DEPTH_VARIANCE_INITIAL      = 1.0f;     // 1.0^2
float DEPTH_VARIANCE_WALK         = 1.0e-2f;  // 0.1^2
float DEPTH_VARIANCE_CONVERGE     = 1.0e-6f;  // 0.001^2
float DEPTH_VARIANCE_EPSILON      = 1.0e-6f;  // 0.001^2
float DEPTH_MIN_INLIER_RATIO      = 0.4f;
//float DEPTH_EPSILON             = 1.0e-3f;  // 1/1000
float DEPTH_PROJECTION_MIN        = 1.0e-3f;  // 1/1000
float DEPTH_PROJECTION_MAX        = 1.0e3f;   // 1/0.001
//float DEPTH_PROJECTION_MAX      = FLT_MAX;

  int DEPTH_TRI_MAX_ITERATIONS             = 10;
float DEPTH_TRI_MAX_ERROR                  = 1.0e2f;
 bool DEPTH_TRI_ROBUST                     = false;
float DEPTH_TRI_CONVERGE                   = 1.0e-3f;
  int DEPTH_TRI_DL_MAX_ITERATIONS          = 10;
float DEPTH_TRI_DL_RADIUS_INITIAL          = 1.0f;
float DEPTH_TRI_DL_RADIUS_MIN              = 1.0e-5f;
float DEPTH_TRI_DL_RADIUS_MAX              = 1.0e2f;
float DEPTH_TRI_DL_RADIUS_FACTOR_INCREASE  = 3.0f;
float DEPTH_TRI_DL_RADIUS_FACTOR_DECREASE  = 0.5f;
float DEPTH_TRI_DL_GAIN_RATIO_MIN          = 0.25f;
float DEPTH_TRI_DL_GAIN_RATIO_MAX          = 0.75f;

  int BA_MAX_ITERATIONS                               = 10;
float BA_WEIGHT_FEATURE                               = 1.0e-5f;
float BA_WEIGHT_FEATURE_KEY_FRAME                     = 1.0e-3f;
float BA_WEIGHT_PRIOR_CAMERA_INITIAL                  = 1.0e-5f;
//float BA_WEIGHT_PRIOR_CAMERA_RELATIVE_CONSTRAINT    = 1.0e-5f;
float BA_WEIGHT_PRIOR_CAMERA_RELATIVE_CONSTRAINT      = 1.0f;
//float BA_WEIGHT_PRIOR_CAMERA_POSE                   = 1.0f;
float BA_WEIGHT_PRIOR_CAMERA_POSE                     = 1.0e1f;
//float BA_WEIGHT_PRIOR_CAMERA_MOTION                 = 0.0f;
float BA_WEIGHT_PRIOR_CAMERA_MOTION                   = 1.0f;
float BA_WEIGHT_PRIOR_DEPTH                           = 0.0f;
//float BA_WEIGHT_PRIOR_DEPTH                         = 1.0e-10f;
float BA_WEIGHT_IMU                                   = 1.0e-5f;
float BA_WEIGHT_FIX_ORIGIN                            = 1.0e-5f;
float BA_WEIGHT_FIX_POSITION_Z                        = 1.0e-5f;
float BA_WEIGHT_FIX_MOTION                            = 1.0e-5f;
float BA_VARIANCE_FIX_ORIGIN_ROTATION_X               = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_ORIGIN_ROTATION_Y               = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_ORIGIN_ROTATION_Z               = 3.046174198662e-8f;   // (0.01*pi/180)^2
float BA_VARIANCE_FIX_ORIGIN_POSITION                 = 1.0e-6f;              // 0.001^2
float BA_VARIANCE_FIX_POSITION_Z                      = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_VELOCITY                        = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_VELOCITY_INITIAL                = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_VELOCITY_INVALID                = 1.0e-2f;              // 0.1^2
float BA_VARIANCE_FIX_BIAS_ACCELERATION               = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL       = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID       = 1.0;                  // 1.0^2
float BA_VARIANCE_FIX_BIAS_GYROSCOPE                  = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL          = 0.0f;                 // 0.0^2
float BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID          = 3.046174198662e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_GRAVITY_FIRST                 = 3.046174093942e-2f;   // (10.0*pi/180)^2
//float BA_VARIANCE_PRIOR_GRAVITY_FIRST               = 3.046174093942f;      // (100.0*pi/180)^2
float BA_VARIANCE_PRIOR_GRAVITY_NEW                   = 0.0f;                 // (0.0*pi/180)^2
//float BA_VARIANCE_PRIOR_GRAVITY_NEW                 = 3.046174093942e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_GRAVITY_RESET                 = 3.046174093942e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_ROTATION_NEW                  = 0.0f;                 // (0.0*pi/180)^2
//float BA_VARIANCE_PRIOR_ROTATION_NEW                = 3.046174093942e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_ROTATION_RESET                = 3.046174093942e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_ROTATION_INFORMATIVE          = 3.046174093942e-6f;   // (0.1*pi/180)^2
//float BA_VARIANCE_PRIOR_ROTATION_INFORMATIVE        = 3.046174093942e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_POSITION_NEW                  = 0.0f;                 // 0.0^2
//float BA_VARIANCE_PRIOR_POSITION_NEW                = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_POSITION_RESET                = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_POSITION_INFORMATIVE          = 1.0e-2f;              // 0.1^2
//float BA_VARIANCE_PRIOR_POSITION_INFORMATIVE        = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_VELOCITY_FIRST                = 1.0e-4f;              // 0.01^2
float BA_VARIANCE_PRIOR_VELOCITY_NEW                  = 0.0f;                 // 0.0^2
//float BA_VARIANCE_PRIOR_VELOCITY_NEW                = 1.0f;                 // 1.0^2
//float BA_VARIANCE_PRIOR_VELOCITY_RESET              = 1.0e-4f;              // 0.01^2
float BA_VARIANCE_PRIOR_VELOCITY_RESET                = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_BIAS_ACCELERATION_FIRST       = 1.0e2f;               // 10.0^2
float BA_VARIANCE_PRIOR_BIAS_ACCELERATION_NEW         = 0.0f;                 // 0.0^2
//float BA_VARIANCE_PRIOR_BIAS_ACCELERATION_NEW       = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_BIAS_ACCELERATION_RESET       = 1.0f;                 // 1.0^2
//float BA_VARIANCE_PRIOR_BIAS_ACCELERATION_RESET     = 1.0e2f;               // 10.0^2
//float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST        = 3.046174198662e-2f;   // (10.0*pi/180)^2
float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST          = 1.2184696794648e-1f;  // (20.0*pi/180)^2
float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_NEW            = 0.0f;                 // (0.0*pi/180)^2
//float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_NEW          = 3.046174198662e-4f;   // (1.0*pi/180)^2
float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_RESET          = 3.046174198662e-4f;   // (1.0*pi/180)^2
//float BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_RESET        = 3.046174198662e-2f;   // (10.0*pi/180)^2
float BA_VARIANCE_PRIOR_DEPTH_NEW                     = 0.0f;                 // 0.0^2
//float BA_VARIANCE_PRIOR_DEPTH_NEW                   = 1.0f;                 // 1.0^2
float BA_VARIANCE_PRIOR_FRAME_DEPTH                   = 1.0f;                 // 1.0^2
//float BA_VARIANCE_MAX_ROTATION                      = 3.046174093942e-2f;   // (10.0*pi/180)^2
float BA_VARIANCE_MAX_ROTATION                        = 3.046174093942f;      // (100.0*pi/180)^2
//float BA_VARIANCE_MAX_POSITION                      = 1.0f;                 // 1.0^2
//float BA_VARIANCE_MAX_POSITION                      = 1.0e2f;               // 10.0^2
float BA_VARIANCE_MAX_POSITION                        = 1.0e4f;               // 100.0^2
//float BA_VARIANCE_MAX_VELOCITY                      = 1.0f;                 // 1.0^2
float BA_VARIANCE_MAX_VELOCITY                        = 1.0e4f;               // 100.0^2
//float BA_VARIANCE_MAX_BIAS_ACCELERATION             = 1.0e2f;               // 10.0^2
float BA_VARIANCE_MAX_BIAS_ACCELERATION               = 1.0e4f;               // 100.0^2
float BA_VARIANCE_MAX_BIAS_GYROSCOPE                  = 3.046174198662f;      // (100.0*pi/180)^2
float BA_VARIANCE_MAX_DEPTH                           = 1.0f;                 // 1.0^2
float BA_VARIANCE_MAX_DEPTH_SLIDING_TRACK             = 1.0f;                 // 1.0^2
float BA_VARIANCE_REGULARIZATION_ROTATION             = 0.0f;                 // 0.0
float BA_VARIANCE_REGULARIZATION_POSITION             = 0.0f;                 // 0.0
float BA_VARIANCE_REGULARIZATION_VELOCITY             = 0.0f;                 // 0.0
float BA_VARIANCE_REGULARIZATION_BIAS_ACCELERATION    = 0.0f;                 // 0.0
float BA_VARIANCE_REGULARIZATION_BIAS_GYROSCOPE       = 0.0f;                 // 0.0
//float BA_VARIANCE_REGULARIZATION_ROTATION           = 3.046174093942e-2f;   // (10.0*pi/180)^2
//float BA_VARIANCE_REGULARIZATION_POSITION           = 1.0f;                 // 1.0^2
//float BA_VARIANCE_REGULARIZATION_VELOCITY           = 1.0f;                 // 1.0^2
//float BA_VARIANCE_REGULARIZATION_BIAS_ACCELERATION  = 1.0e2f;               // 10.0^2
//float BA_VARIANCE_REGULARIZATION_BIAS_GYROSCOPE     = 3.046174198662f;      // (100.0*pi/180)^2
float BA_VARIANCE_REGULARIZATION_DEPTH                = 1.0e2f;               // 10.0^2
float BA_UPDATE_ROTATION                              = 3.046174198662e-8f;   // (0.01*pi/180)^2
float BA_UPDATE_POSITION                              = 1.0e-6f;              // 0.001^2
float BA_UPDATE_VELOCITY                              = 1.0e-4f;              // 0.01^2
//float BA_UPDATE_VELOCITY                            = 1.0e-4f;              // 0.01^2
float BA_UPDATE_BIAS_ACCELERATION                     = 1.0e-4f;              // 0.01^2
float BA_UPDATE_BIAS_GYROSCOPE                        = 3.046174198662e-8f;   // (0.01*pi/180)^2
float BA_UPDATE_DEPTH                                 = 1.0e-3f;
float BA_UPDATE_FRAME_DEPTH                           = 1.0f;
float BA_UPDATE_FRAME_DEPTH_RATIO                     = 0.5f;
float BA_BACK_SUBSTITUTE_ROTATION                     = 3.046174198662e-8f;  // (0.01*pi/180)^2
float BA_BACK_SUBSTITUTE_POSITION                     = 1.0e-6f;             // 0.001^2
float BA_CONVERGE_ROTATION                            = 7.615435234857e-5f;  // (0.5*pi/180)^2
float BA_CONVERGE_POSITION                            = 2.5e-3f;             // 0.05^2
float BA_CONVERGE_VELOCITY                            = 2.5e-3f;             // 0.05^2
float BA_CONVERGE_BIAS_ACCELERATION                   = 2.5e-3f;             // 0.05^2
float BA_CONVERGE_BIAS_GYROSCOPE                      = 7.615435234857e-7f;  // (0.05*pi/180)^2
float BA_CONVERGE_DEPTH                               = 1.0e-2f;
float BA_PCG_CONDITIONER_MAX                          = 1.0e7f;
//float BA_PCG_CONDITIONER_EPSILON                    = 0.0f;
float BA_PCG_CONDITIONER_EPSILON                      = 1.0e-2f;
  int BA_PCG_MIN_ITERATIONS                           = 10;
float BA_PCG_MIN_CONVERGE_RESIDUAL_RATIO              = 1.0e-6f;
float BA_PCG_MIN_CONVERGE_PROBABILITY                 = 0.9f;
  int BA_PCG_MAX_ITERATIONS                           = 100;
float BA_PCG_MAX_CONVERGE_RESIDUAL_RATIO              = 1.0e3f;
float BA_PCG_MAX_CONVERGE_PROBABILITY                 = 0.99f;
  int BA_DL_MAX_ITERATIONS                            = 10;
//float BA_DL_RADIUS_INITIAL                          = 1.0f;     // 1.0^2
float BA_DL_RADIUS_INITIAL                            = 1.0e4f;   // 100.0^2
float BA_DL_RADIUS_MIN                                = 1.0e-10f; // 0.00001^2
float BA_DL_RADIUS_MAX                                = 1.0e4f;   // 100.0^2
float BA_DL_RADIUS_FACTOR_INCREASE                    = 9.0f;     // 3.0^2
float BA_DL_RADIUS_FACTOR_DECREASE                    = 0.25f;    // 0.5^2
float BA_DL_GAIN_RATIO_MIN                            = 0.25f;
float BA_DL_GAIN_RATIO_MAX                            = 0.75f;
float BA_ANGLE_EPSILON                                = 1.745329252e-7f;  //1.0e-5*pi/180

 int LBA_MAX_SLIDING_TRACK_LENGTH           = 5;
 int LBA_MAX_LOCAL_FRAMES                   = 50;
bool LBA_RESET_DEPTH_INFORMATION            = false;
 int LBA_PROPAGATE_CAMERA                   = 1;
 int LBA_PCG_CONDITIONER_BAND               = 1;
bool LBA_EMBEDDED_MOTION_ITERATION          = false;
bool LBA_EMBEDDED_POINT_ITERATION           = false;
bool LBA_MARGINALIZATION_REFERENCE_NEAREST  = true;
bool LBA_MARGINALIZATION_CHECK_RANK         = true;
bool LBA_MARGINALIZATION_CHECK_INVERTIBLE   = true;

//float GBA_MAX_IMU_TIME_INTERVAL   = 5.0f;
float GBA_MAX_IMU_TIME_INTERVAL     = 1.0e3f;
 bool GBA_RESET_DEPTH_INFORMATION   = false;
  //int GBA_PCG_CONDITIONER_BAND    = 1;
  int GBA_PCG_CONDITIONER_BAND      = 2;
 bool GBA_PROPAGATE_CAMERA          = false;
 bool GBA_EMBEDDED_MOTION_ITERATION = false;
 bool GBA_EMBEDDED_POINT_ITERATION  = false;
 bool GBA_PRIOR_CAMERA_POSE_ROBUST  = true;

//float MAX_ERROR_FEATURE             = 1.0e1f;
//float MAX_ERROR_IMU_ROTATION        = 1.0f;
//float MAX_ERROR_IMU_POSITION        = 1.0e-1f;
//float MAX_ERROR_IMU_VELOCITY        = 1.0e-1f;
float MAX_ERROR_FEATURE               = 1.0e3f;
float MAX_ERROR_IMU_ROTATION          = 1.0e2f;
float MAX_ERROR_IMU_POSITION          = 1.0e2f;
float MAX_ERROR_IMU_VELOCITY          = 1.0e2f;
float MAX_ERROR_IMU_BIAS_ACCELERATION = 0.01f;
float MAX_ERROR_IMU_BIAS_GYROSCOPE    = 0.1f;
//float MAX_ERROR_DRIFT_ROTATION      = 1.0f;
//float MAX_ERROR_DRIFT_POSITION      = 1.0e-1f;
float MAX_ERROR_DRIFT_ROTATION        = 1.0e1f;
float MAX_ERROR_DRIFT_POSITION        = 1.0f;

   int VW_WINDOW_WIDTH                        = 0;
   int VW_WINDOW_HEIGHT                       = 0;
 float VM_WINDOW_ASPECT_RATIO                 = 0.0f;
double VW_WINDOW_DEPTH                        = 0.00001;
 float VW_PROJECTION_MAX_VIEW_ANGLE           = 0.78539815f; // 45.0*pi/180
 float VW_CAMERA_DEPTH_TO_SCENE_DEPTH_RATIO   = 0.1f;
 float VW_CAMERA_SIZE_ACTIVE_RATIO            = 1.0f;
 float VW_ARCBALL_RADIUS_TO_WINDOW_SIZE_RATIO = 0.4f;
 float VW_SCROLL_TRANSLATTON_Z_RATE           = 100.0f;

int TIME_AVERAGING_COUNT = 10;

void LoadParameters(const Configurator &cfgor) {
  KF_FIRST_LOCAL_FRAMES =
    cfgor.GetArgument(
    "param_kf_first_local_frames",
    KF_FIRST_LOCAL_FRAMES);
  KF_MIN_FRAME_STEP =
    cfgor.GetArgument(
    "param_kf_frame_step",
    KF_MIN_FRAME_STEP);
  KF_MIN_FEATURE_SROUCES =
    cfgor.GetArgument(
    "param_kf_min_feature_sources",
    KF_MIN_FEATURE_SROUCES);
  KF_MIN_FEATURE_MEASUREMENTS =
    cfgor.GetArgument(
    "param_kf_min_feature_measurements",
    KF_MIN_FEATURE_MEASUREMENTS);

  FTR_VARIANCE =
    cfgor.GetArgumentSquared(
    "param_feature_variance",
    FTR_VARIANCE);
  FTR_VARIANCE_EPSILON =
    cfgor.GetArgumentSquared(
    "param_feature_variance_epsilon",
    FTR_VARIANCE_EPSILON);
  FTR_UNDIST_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_feature_undistort_max_iterations",
    FTR_UNDIST_MAX_ITERATIONS);
  FTR_UNDIST_CONVERGE =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_converge",
    FTR_UNDIST_CONVERGE);
  FTR_UNDIST_LUT_SIZE =
    cfgor.GetArgument(
    "param_feature_undistort_loop_up_table_size",
    FTR_UNDIST_LUT_SIZE);
#ifdef CFG_DEBUG
  UT_ASSERT(FTR_UNDIST_LUT_SIZE % 2 == 1);
#endif
  FTR_UNDIST_DL_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_feature_undistort_dog_leg_max_iterations",
    FTR_UNDIST_DL_MAX_ITERATIONS);
  FTR_UNDIST_DL_RADIUS_INITIAL =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_dog_leg_radius_initial",
    FTR_UNDIST_DL_RADIUS_INITIAL);
  FTR_UNDIST_DL_RADIUS_MIN  =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_dog_leg_radius_min",
    FTR_UNDIST_DL_RADIUS_MIN);
  FTR_UNDIST_DL_RADIUS_MAX  =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_dog_leg_radius_max",
    FTR_UNDIST_DL_RADIUS_MAX);
  FTR_UNDIST_DL_RADIUS_FACTOR_INCREASE =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_dog_leg_radius_factor_increase",
    FTR_UNDIST_DL_RADIUS_FACTOR_INCREASE);
  FTR_UNDIST_DL_RADIUS_FACTOR_DECREASE =
    cfgor.GetArgumentSquared(
    "param_feature_undistort_dog_leg_radius_factor_decrease",
    FTR_UNDIST_DL_RADIUS_FACTOR_DECREASE);
  FTR_UNDIST_DL_GAIN_RATIO_MIN =
    cfgor.GetArgument(
    "param_feature_undistort_dog_leg_gain_ratio_min",
    FTR_UNDIST_DL_GAIN_RATIO_MIN);
  FTR_UNDIST_DL_GAIN_RATIO_MAX =
    cfgor.GetArgument(
    "param_feature_undistort_dog_leg_gain_ratio_max",
    FTR_UNDIST_DL_GAIN_RATIO_MAX);

  IMU_GRAVITY_MAGNITUDE =
    cfgor.GetArgument(
    "param_imu_gravity_magnitude",
    IMU_GRAVITY_MAGNITUDE);
  IMU_GRAVITY_EXCLUDED =
    cfgor.GetArgument(
    "param_imu_gravity_excluded",
    static_cast<int>(IMU_GRAVITY_EXCLUDED)) != 0;
  IMU_VARIANCE_ACCELERATION_NOISE =
    cfgor.GetArgumentSquared(
    "param_imu_variance_acceleration_noise",
    IMU_VARIANCE_ACCELERATION_NOISE);
  IMU_VARIANCE_ACCELERATION_BIAS_WALK =
    cfgor.GetArgumentSquared(
    "param_imu_variance_acceleration_bias_walk",
    IMU_VARIANCE_ACCELERATION_BIAS_WALK);
  IMU_VARIANCE_GYROSCOPE_NOISE =
    cfgor.GetArgumentRadianSquared(
    "param_imu_variance_gyroscope_noise",
    IMU_VARIANCE_GYROSCOPE_NOISE);
  IMU_VARIANCE_GYROSCOPE_BIAS_WALK =
    cfgor.GetArgumentRadianSquared(
    "param_imu_variance_gyroscope_bias_walk",
    IMU_VARIANCE_GYROSCOPE_BIAS_WALK);
  IMU_VARIANCE_EPSILON_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_imu_variance_epsilon_rotation",
    IMU_VARIANCE_EPSILON_ROTATION);
  IMU_VARIANCE_EPSILON_VELOCITY =
    cfgor.GetArgumentSquared(
    "param_imu_variance_epsilon_velocity",
    IMU_VARIANCE_EPSILON_VELOCITY);
  IMU_VARIANCE_EPSILON_POSITION =
    cfgor.GetArgumentSquared(
    "param_imu_variance_epsilon_position",
    IMU_VARIANCE_EPSILON_POSITION);
  IMU_VARIANCE_EPSILON_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_imu_variance_epsilon_bias_acceleration",
    IMU_VARIANCE_EPSILON_BIAS_ACCELERATION);
  IMU_VARIANCE_EPSILON_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_imu_variance_epsilon_bias_gyroscope",
    IMU_VARIANCE_EPSILON_BIAS_GYROSCOPE);
  IMU_WEIGHT_ROTATION =
    cfgor.GetArgument(
    "param_imu_weight_rotation",
    IMU_WEIGHT_ROTATION);
  IMU_WEIGHT_VELOCITY =
    cfgor.GetArgument(
    "param_imu_weight_velocity",
    IMU_WEIGHT_VELOCITY);
  IMU_WEIGHT_POSITION =
    cfgor.GetArgument(
    "param_imu_weight_position",
    IMU_WEIGHT_POSITION);
  IMU_WEIGHT_BIAS_ACCELERATION =
    cfgor.GetArgument(
    "param_imu_weight_bias_acceleration",
    IMU_WEIGHT_BIAS_ACCELERATION);
  IMU_WEIGHT_BIAS_GYROSCOPE =
    cfgor.GetArgument(
    "param_imu_weight_bias_gyroscope",
    IMU_WEIGHT_BIAS_GYROSCOPE);

  DEPTH_MIN =
    cfgor.GetArgumentInverse(
    "param_depth_max",
    DEPTH_MIN);
  DEPTH_MAX =
    cfgor.GetArgumentInverse(
    "param_depth_min",
    DEPTH_MAX);
  DEPTH_INITIAL =
    cfgor.GetArgumentInverse(
    "param_depth_initial",
    DEPTH_INITIAL);
  DEPTH_VARIANCE_INITIAL =
    cfgor.GetArgumentSquared(
    "param_depth_variance_initial",
    DEPTH_VARIANCE_INITIAL);
  DEPTH_VARIANCE_WALK =
    cfgor.GetArgumentSquared(
    "param_depth_variance_walk",
    DEPTH_VARIANCE_WALK);
  DEPTH_VARIANCE_CONVERGE =
    cfgor.GetArgumentSquared(
    "param_depth_variance_converge",
    DEPTH_VARIANCE_CONVERGE);
  DEPTH_VARIANCE_EPSILON =
    cfgor.GetArgument(
    "param_depth_variance_epsilon",
    DEPTH_VARIANCE_EPSILON);
  DEPTH_MIN_INLIER_RATIO =
    cfgor.GetArgument(
    "param_depth_min_inlier_ratio",
    DEPTH_MIN_INLIER_RATIO);
  //DEPTH_EPSILON =
  //  cfgor.GetArgument(
  //  "param_depth_epsilon",
  //  DEPTH_EPSILON);
  DEPTH_PROJECTION_MIN =
    cfgor.GetArgument(
    "param_depth_projection_min",
    DEPTH_PROJECTION_MIN);
  DEPTH_PROJECTION_MAX =
    cfgor.GetArgument(
    "param_depth_projection_max",
    DEPTH_PROJECTION_MAX);

  DEPTH_TRI_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_depth_tri_max_iterations",
    DEPTH_TRI_MAX_ITERATIONS);
  DEPTH_TRI_MAX_ERROR =
    cfgor.GetArgument(
    "param_depth_tri_max_error",
    DEPTH_TRI_MAX_ERROR);
  DEPTH_TRI_ROBUST =
    cfgor.GetArgument(
    "param_depth_tri_robust",
    static_cast<int>(DEPTH_TRI_ROBUST)) != 0;
  DEPTH_TRI_CONVERGE =
    cfgor.GetArgument(
    "param_depth_tri_converge",
    DEPTH_TRI_CONVERGE);
  DEPTH_TRI_DL_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_max_iterations",
    DEPTH_TRI_DL_MAX_ITERATIONS);
  DEPTH_TRI_DL_RADIUS_INITIAL =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_radius_initial",
    DEPTH_TRI_DL_RADIUS_INITIAL);
  DEPTH_TRI_DL_RADIUS_MIN  =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_radius_min",
    DEPTH_TRI_DL_RADIUS_MIN);
  DEPTH_TRI_DL_RADIUS_MAX  =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_radius_max",
    DEPTH_TRI_DL_RADIUS_MAX);
  DEPTH_TRI_DL_RADIUS_FACTOR_INCREASE =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_radius_factor_increase",
    DEPTH_TRI_DL_RADIUS_FACTOR_INCREASE);
  DEPTH_TRI_DL_RADIUS_FACTOR_DECREASE =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_radius_factor_decrease",
    DEPTH_TRI_DL_RADIUS_FACTOR_DECREASE);
  DEPTH_TRI_DL_GAIN_RATIO_MIN =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_gain_ratio_min",
    DEPTH_TRI_DL_GAIN_RATIO_MIN);
  DEPTH_TRI_DL_GAIN_RATIO_MAX =
    cfgor.GetArgument(
    "param_depth_tri_dog_leg_gain_ratio_max",
    DEPTH_TRI_DL_GAIN_RATIO_MAX);
  DEPTH_RANGE = DEPTH_MAX - DEPTH_MIN;
  if (DEPTH_MIN == 0.0f) {
    DEPTH_MIN = -FLT_MAX;
  }
#ifdef CFG_DEBUG
  UT_ASSERT(DEPTH_VARIANCE_CONVERGE > FLT_EPSILON);
#endif

  BA_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_ba_max_iterations",
    BA_MAX_ITERATIONS);
  BA_WEIGHT_FEATURE =
    cfgor.GetArgument(
    "param_ba_weight_feature",
    BA_WEIGHT_FEATURE);
  BA_WEIGHT_FEATURE_KEY_FRAME =
    cfgor.GetArgument(
    "param_ba_weight_feature_key_frame",
    BA_WEIGHT_FEATURE_KEY_FRAME);
  BA_WEIGHT_PRIOR_CAMERA_INITIAL =
    cfgor.GetArgument(
    "param_ba_weight_prior_camera_initial",
     BA_WEIGHT_PRIOR_CAMERA_INITIAL);
  BA_WEIGHT_PRIOR_CAMERA_RELATIVE_CONSTRAINT =
    cfgor.GetArgument(
    "param_ba_weight_prior_camera_relative_constraint",
     BA_WEIGHT_PRIOR_CAMERA_RELATIVE_CONSTRAINT);
  BA_WEIGHT_PRIOR_CAMERA_POSE =
    cfgor.GetArgument(
    "param_ba_weight_prior_camera_pose",
     BA_WEIGHT_PRIOR_CAMERA_POSE);
  BA_WEIGHT_PRIOR_CAMERA_MOTION =
    cfgor.GetArgument(
    "param_ba_weight_prior_camera_motion",
     BA_WEIGHT_PRIOR_CAMERA_MOTION);
  BA_WEIGHT_PRIOR_DEPTH =
    cfgor.GetArgument(
    "param_ba_weight_prior_depth",
     BA_WEIGHT_PRIOR_DEPTH);
  BA_WEIGHT_IMU =
    cfgor.GetArgument(
    "param_ba_weight_imu",
     BA_WEIGHT_IMU);
  BA_WEIGHT_FIX_ORIGIN =
    cfgor.GetArgument(
    "param_ba_weight_fix_origin",
    BA_WEIGHT_FIX_ORIGIN);
  BA_WEIGHT_FIX_POSITION_Z =
    cfgor.GetArgument(
    "param_ba_weight_fix_position_z",
    BA_WEIGHT_FIX_POSITION_Z);
  BA_WEIGHT_FIX_MOTION =
    cfgor.GetArgument(
    "param_ba_weight_fix_motion",
    BA_WEIGHT_FIX_MOTION);
  BA_VARIANCE_FIX_ORIGIN_ROTATION_X =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_origin_rotation_x",
    BA_VARIANCE_FIX_ORIGIN_ROTATION_X);
  BA_VARIANCE_FIX_ORIGIN_ROTATION_Y =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_origin_rotation_y",
    BA_VARIANCE_FIX_ORIGIN_ROTATION_Y);
  BA_VARIANCE_FIX_ORIGIN_ROTATION_Z =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_origin_rotation_z",
    BA_VARIANCE_FIX_ORIGIN_ROTATION_Z);
  BA_VARIANCE_FIX_ORIGIN_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_fix_origin_position",
    BA_VARIANCE_FIX_ORIGIN_POSITION);
  BA_VARIANCE_FIX_POSITION_Z =
    cfgor.GetArgumentSquared(
    "param_ba_variance_fix_position_z",
    BA_VARIANCE_FIX_POSITION_Z);
  BA_VARIANCE_FIX_VELOCITY =
    cfgor.GetArgument(
    "param_ba_variance_fix_velocity",
     BA_VARIANCE_FIX_VELOCITY);
  BA_VARIANCE_FIX_VELOCITY_INITIAL =
    cfgor.GetArgument(
    "param_ba_variance_fix_velocity_initial",
     BA_VARIANCE_FIX_VELOCITY_INITIAL);
  BA_VARIANCE_FIX_VELOCITY_INVALID =
    cfgor.GetArgument(
    "param_ba_variance_fix_velocity_invalid",
    BA_VARIANCE_FIX_VELOCITY_INVALID);
  BA_VARIANCE_FIX_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_fix_bias_acceleration",
    BA_VARIANCE_FIX_BIAS_ACCELERATION);
  BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL =
    cfgor.GetArgumentSquared(
    "param_ba_variance_fix_bias_acceleration_initial",
    BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL);
  BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID =
    cfgor.GetArgumentSquared(
    "param_ba_variance_fix_bias_acceleration_invalid",
    BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID);
  BA_VARIANCE_FIX_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_bias_gyroscope",
    BA_VARIANCE_FIX_BIAS_GYROSCOPE);
  BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_bias_gyroscope_initial",
    BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL);
  BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_fix_bias_gyroscope_invalid",
    BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID);
  BA_VARIANCE_PRIOR_GRAVITY_FIRST =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_gravity_first",
    BA_VARIANCE_PRIOR_GRAVITY_FIRST);
  BA_VARIANCE_PRIOR_GRAVITY_NEW =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_gravity_new",
    BA_VARIANCE_PRIOR_GRAVITY_NEW);
  BA_VARIANCE_PRIOR_GRAVITY_RESET =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_gravity_reset",
    BA_VARIANCE_PRIOR_GRAVITY_RESET);
  BA_VARIANCE_PRIOR_ROTATION_NEW =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_rotation_new",
    BA_VARIANCE_PRIOR_ROTATION_NEW);
  BA_VARIANCE_PRIOR_ROTATION_RESET =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_rotation_reset",
    BA_VARIANCE_PRIOR_ROTATION_RESET);
  BA_VARIANCE_PRIOR_ROTATION_INFORMATIVE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_rotation_informative",
    BA_VARIANCE_PRIOR_ROTATION_INFORMATIVE);
  BA_VARIANCE_PRIOR_POSITION_NEW =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_position_new",
    BA_VARIANCE_PRIOR_POSITION_NEW);
  BA_VARIANCE_PRIOR_POSITION_RESET =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_position_reset",
    BA_VARIANCE_PRIOR_POSITION_RESET);
  BA_VARIANCE_PRIOR_POSITION_INFORMATIVE =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_position_informative",
    BA_VARIANCE_PRIOR_POSITION_INFORMATIVE);
  BA_VARIANCE_PRIOR_VELOCITY_FIRST =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_velocity_first",
    BA_VARIANCE_PRIOR_VELOCITY_FIRST);
  BA_VARIANCE_PRIOR_VELOCITY_NEW =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_velocity_new",
    BA_VARIANCE_PRIOR_VELOCITY_NEW);
  BA_VARIANCE_PRIOR_VELOCITY_RESET =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_velocity_reset",
    BA_VARIANCE_PRIOR_VELOCITY_RESET);
  BA_VARIANCE_PRIOR_BIAS_ACCELERATION_FIRST =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_bias_acceleration_first",
    BA_VARIANCE_PRIOR_BIAS_ACCELERATION_FIRST);
  BA_VARIANCE_PRIOR_BIAS_ACCELERATION_NEW =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_bias_acceleration_new",
    BA_VARIANCE_PRIOR_BIAS_ACCELERATION_NEW);
  BA_VARIANCE_PRIOR_BIAS_ACCELERATION_RESET =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_bias_acceleration_reset",
    BA_VARIANCE_PRIOR_BIAS_ACCELERATION_RESET);
  BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_bias_gyroscope_first",
    BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST);
  BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_NEW =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_bias_gyroscope_new",
    BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_NEW);
  BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_RESET =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_prior_bias_gyroscope_reset",
    BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_RESET);
  BA_VARIANCE_PRIOR_DEPTH_NEW =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_depth_new",
     BA_VARIANCE_PRIOR_DEPTH_NEW);
  BA_VARIANCE_PRIOR_FRAME_DEPTH =
    cfgor.GetArgumentSquared(
    "param_ba_variance_prior_frame_depth",
     BA_VARIANCE_PRIOR_FRAME_DEPTH);
  BA_VARIANCE_MAX_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_max_rotation",
    BA_VARIANCE_MAX_ROTATION);
  BA_VARIANCE_MAX_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_max_position",
    BA_VARIANCE_MAX_POSITION);
  BA_VARIANCE_MAX_VELOCITY =
    cfgor.GetArgumentSquared(
    "param_ba_variance_max_velocity",
    BA_VARIANCE_MAX_VELOCITY);
  BA_VARIANCE_MAX_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_max_bias_acceleration",
    BA_VARIANCE_MAX_BIAS_ACCELERATION);
  BA_VARIANCE_MAX_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_max_bias_gyroscope",
    BA_VARIANCE_MAX_BIAS_GYROSCOPE);
  BA_VARIANCE_MAX_DEPTH =
    cfgor.GetArgumentSquared(
    "param_ba_variance_max_depth",
    BA_VARIANCE_MAX_DEPTH);
  BA_VARIANCE_MAX_DEPTH_SLIDING_TRACK =
    cfgor.GetArgumentSquared(
    "param_ba_variance_max_depth_sliding_track",
    BA_VARIANCE_MAX_DEPTH_SLIDING_TRACK);
  BA_VARIANCE_REGULARIZATION_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_regularization_rotation",
    BA_VARIANCE_REGULARIZATION_ROTATION);
  BA_VARIANCE_REGULARIZATION_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_regularization_position",
    BA_VARIANCE_REGULARIZATION_POSITION);
  BA_VARIANCE_REGULARIZATION_VELOCITY =
    cfgor.GetArgumentSquared(
    "param_ba_variance_regularization_velocity",
    BA_VARIANCE_REGULARIZATION_VELOCITY);
  BA_VARIANCE_REGULARIZATION_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_ba_variance_regularization_bias_acceleration",
    BA_VARIANCE_REGULARIZATION_BIAS_ACCELERATION);
  BA_VARIANCE_REGULARIZATION_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_variance_regularization_bias_gyroscope",
    BA_VARIANCE_REGULARIZATION_BIAS_GYROSCOPE);
  BA_VARIANCE_REGULARIZATION_DEPTH =
    cfgor.GetArgumentSquared(
    "param_ba_variance_regularization_depth",
    BA_VARIANCE_REGULARIZATION_DEPTH);
  BA_UPDATE_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_ba_update_rotation",
    BA_UPDATE_ROTATION);
  BA_UPDATE_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_update_position",
    BA_UPDATE_POSITION);
  BA_UPDATE_DEPTH =
    cfgor.GetArgument(
    "param_ba_update_depth",
    BA_UPDATE_DEPTH);
  BA_UPDATE_FRAME_DEPTH =
    cfgor.GetArgument(
    "param_ba_update_frame_depth",
    BA_UPDATE_FRAME_DEPTH);
  BA_UPDATE_FRAME_DEPTH_RATIO =
    cfgor.GetArgument(
    "param_ba_update_frame_depth_ratio",
    BA_UPDATE_FRAME_DEPTH_RATIO);
  BA_UPDATE_VELOCITY =
    cfgor.GetArgumentSquared(
    "param_ba_update_velocity",
    BA_UPDATE_VELOCITY);
  BA_UPDATE_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_ba_update_bias_acceleration",
    BA_UPDATE_BIAS_ACCELERATION);
  BA_UPDATE_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_update_bias_gyroscope",
    BA_UPDATE_BIAS_GYROSCOPE);
  BA_BACK_SUBSTITUTE_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_ba_back_substitute_rotation",
    BA_BACK_SUBSTITUTE_ROTATION);
  BA_BACK_SUBSTITUTE_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_back_substitute_position",
    BA_BACK_SUBSTITUTE_POSITION);
  BA_CONVERGE_ROTATION =
    cfgor.GetArgumentRadianSquared(
    "param_ba_converge_rotation",
    BA_CONVERGE_ROTATION);
  BA_CONVERGE_POSITION =
    cfgor.GetArgumentSquared(
    "param_ba_converge_position",
    BA_CONVERGE_POSITION);
  BA_CONVERGE_VELOCITY =
    cfgor.GetArgumentSquared(
    "param_ba_converge_velocity",
    BA_CONVERGE_VELOCITY);
  BA_CONVERGE_BIAS_ACCELERATION =
    cfgor.GetArgumentSquared(
    "param_ba_converge_bias_acceleration",
    BA_CONVERGE_BIAS_ACCELERATION);
  BA_CONVERGE_BIAS_GYROSCOPE =
    cfgor.GetArgumentRadianSquared(
    "param_ba_converge_bias_gyroscope",
    BA_CONVERGE_BIAS_GYROSCOPE);
  BA_CONVERGE_DEPTH =
    cfgor.GetArgument(
    "param_ba_converge_depth",
    BA_CONVERGE_DEPTH);
  BA_PCG_CONDITIONER_MAX =
    cfgor.GetArgument(
    "param_ba_pcg_conditioner_max",
    BA_PCG_CONDITIONER_MAX);
  BA_PCG_CONDITIONER_EPSILON =
    cfgor.GetArgument(
    "param_ba_pcg_conditioner_epsilon",
    BA_PCG_CONDITIONER_EPSILON);
  BA_PCG_MIN_ITERATIONS =
    cfgor.GetArgument(
    "param_ba_pcg_min_iterations",
    BA_PCG_MIN_ITERATIONS);
  BA_PCG_MIN_CONVERGE_RESIDUAL_RATIO =
    cfgor.GetArgument(
    "param_ba_pcg_min_converge_residual_ratio",
    BA_PCG_MIN_CONVERGE_RESIDUAL_RATIO);
  BA_PCG_MIN_CONVERGE_PROBABILITY =
    cfgor.GetArgument(
    "param_ba_pcg_min_converge_probability",
    BA_PCG_MIN_CONVERGE_PROBABILITY);
  BA_PCG_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_ba_pcg_max_iterations",
    BA_PCG_MAX_ITERATIONS);
  BA_PCG_MAX_CONVERGE_RESIDUAL_RATIO =
    cfgor.GetArgument(
    "param_ba_pcg_max_converge_residual_ratio",
    BA_PCG_MAX_CONVERGE_RESIDUAL_RATIO);
  BA_PCG_MAX_CONVERGE_PROBABILITY =
    cfgor.GetArgument(
    "param_ba_pcg_max_converge_probability",
    BA_PCG_MAX_CONVERGE_PROBABILITY);
  BA_DL_MAX_ITERATIONS =
    cfgor.GetArgument(
    "param_ba_dog_leg_max_iterations",
     BA_DL_MAX_ITERATIONS);
  BA_DL_RADIUS_INITIAL =
    cfgor.GetArgumentSquared(
    "param_ba_dog_leg_radius_initial",
    BA_DL_RADIUS_INITIAL);
  BA_DL_RADIUS_MIN =
    cfgor.GetArgumentSquared(
    "param_ba_dog_leg_radius_min",
    BA_DL_RADIUS_MIN);
  BA_DL_RADIUS_MAX =
    cfgor.GetArgumentSquared(
    "param_ba_dog_leg_radius_max",
    BA_DL_RADIUS_MAX);
  BA_DL_RADIUS_FACTOR_INCREASE =
    cfgor.GetArgumentSquared(
    "param_ba_dog_leg_radius_factor_increase",
    BA_DL_RADIUS_FACTOR_INCREASE);
  BA_DL_RADIUS_FACTOR_DECREASE =
    cfgor.GetArgumentSquared(
    "param_ba_dog_leg_radius_factor_decrease",
    BA_DL_RADIUS_FACTOR_DECREASE);
  BA_DL_GAIN_RATIO_MIN =
    cfgor.GetArgument(
    "param_ba_dog_leg_gain_ratio_min",
     BA_DL_GAIN_RATIO_MIN);
  BA_DL_GAIN_RATIO_MAX =
    cfgor.GetArgument(
    "param_ba_dog_leg_gain_ratio_max",
     BA_DL_GAIN_RATIO_MAX);
  BA_ANGLE_EPSILON =
    cfgor.GetArgumentRadian(
    "param_ba_angle_epsilon",
    BA_ANGLE_EPSILON);
#ifdef CFG_DEBUG
  UT_ASSERT(BA_UPDATE_ROTATION          >= BA_BACK_SUBSTITUTE_ROTATION);
  UT_ASSERT(BA_UPDATE_POSITION          >= BA_BACK_SUBSTITUTE_POSITION);
  UT_ASSERT(BA_UPDATE_ROTATION          <= BA_CONVERGE_ROTATION);
  UT_ASSERT(BA_UPDATE_POSITION          <= BA_CONVERGE_POSITION);
  UT_ASSERT(BA_UPDATE_VELOCITY          <= BA_CONVERGE_VELOCITY);
  UT_ASSERT(BA_UPDATE_BIAS_ACCELERATION <= BA_CONVERGE_BIAS_ACCELERATION);
  UT_ASSERT(BA_UPDATE_BIAS_GYROSCOPE    <= BA_CONVERGE_BIAS_GYROSCOPE);
  UT_ASSERT(BA_UPDATE_DEPTH             <= BA_CONVERGE_DEPTH);
#endif

  LBA_MAX_SLIDING_TRACK_LENGTH =
    cfgor.GetArgument(
    "param_lba_max_sliding_track_length",
    LBA_MAX_SLIDING_TRACK_LENGTH);
  LBA_MAX_LOCAL_FRAMES =
    cfgor.GetArgument(
    "param_lba_max_local_frames",
    LBA_MAX_LOCAL_FRAMES);
  LBA_RESET_DEPTH_INFORMATION =
    cfgor.GetArgument(
    "param_lba_reset_depth_information",
    static_cast<int>(LBA_RESET_DEPTH_INFORMATION)) != 0;
  LBA_PROPAGATE_CAMERA =
    cfgor.GetArgument(
    "param_lba_propagate_camera",
    LBA_PROPAGATE_CAMERA);
  LBA_PCG_CONDITIONER_BAND =
    cfgor.GetArgument(
    "param_lba_pcg_conditioner_band",
    LBA_PCG_CONDITIONER_BAND);
#ifdef CFG_DEBUG
  UT_ASSERT(LBA_MAX_SLIDING_TRACK_LENGTH >= 2 && LBA_MAX_SLIDING_TRACK_LENGTH <= LBA_MAX_LOCAL_FRAMES);
#endif
  LBA_EMBEDDED_MOTION_ITERATION =
    cfgor.GetArgument(
    "param_lba_embedded_motion_iteration",
    static_cast<int>(LBA_EMBEDDED_MOTION_ITERATION)) != 0;
  LBA_EMBEDDED_POINT_ITERATION =
    cfgor.GetArgument(
    "param_lba_embedded_point_iteration",
    static_cast<int>(LBA_EMBEDDED_POINT_ITERATION)) != 0;
  LBA_MARGINALIZATION_REFERENCE_NEAREST =
    cfgor.GetArgument(
    "param_lba_marginalization_reference_nearest",
    static_cast<int>(LBA_MARGINALIZATION_REFERENCE_NEAREST)) != 0;
  LBA_MARGINALIZATION_CHECK_INVERTIBLE =
    cfgor.GetArgument(
    "param_lba_marginalization_check_invertible",
    static_cast<int>(LBA_MARGINALIZATION_CHECK_INVERTIBLE)) != 0;
  LBA_MARGINALIZATION_CHECK_RANK =
    cfgor.GetArgument(
    "param_lba_marginalization_check_rank",
    static_cast<int>(LBA_MARGINALIZATION_CHECK_RANK)) != 0;

  GBA_MAX_IMU_TIME_INTERVAL =
    cfgor.GetArgument(
    "param_gba_max_imu_time_interval",
    GBA_MAX_IMU_TIME_INTERVAL);
  GBA_RESET_DEPTH_INFORMATION =
    cfgor.GetArgument(
    "param_gba_reset_depth_information",
    static_cast<int>(GBA_RESET_DEPTH_INFORMATION)) != 0;
  GBA_PROPAGATE_CAMERA =
    cfgor.GetArgument(
    "param_gba_propagate_camera",
    static_cast<int>(GBA_PROPAGATE_CAMERA)) != 0;
  GBA_PCG_CONDITIONER_BAND =
    cfgor.GetArgument(
    "param_gba_pcg_conditioner_band",
    GBA_PCG_CONDITIONER_BAND);
  GBA_EMBEDDED_MOTION_ITERATION =
    cfgor.GetArgument(
    "param_gba_embedded_motion_iteration",
    static_cast<int>(GBA_EMBEDDED_MOTION_ITERATION)) != 0;
  GBA_EMBEDDED_POINT_ITERATION =
    cfgor.GetArgument(
    "param_gba_embedded_point_iteration",
    static_cast<int>(GBA_EMBEDDED_POINT_ITERATION)) != 0;
  GBA_PRIOR_CAMERA_POSE_ROBUST =
    cfgor.GetArgument(
    "param_gba_prior_camera_pose_robust",
    static_cast<int>(GBA_PRIOR_CAMERA_POSE_ROBUST)) != 0;

  MAX_ERROR_FEATURE =
    cfgor.GetArgument(
    "param_max_error_feature",
    MAX_ERROR_FEATURE);
  MAX_ERROR_IMU_ROTATION =
    cfgor.GetArgument(
    "param_max_error_imu_rotation",
    MAX_ERROR_IMU_ROTATION);
  MAX_ERROR_IMU_POSITION =
    cfgor.GetArgument(
    "param_max_error_imu_position",
    MAX_ERROR_IMU_POSITION);
  MAX_ERROR_IMU_VELOCITY =
    cfgor.GetArgument(
    "param_max_error_imu_velocity",
    MAX_ERROR_IMU_VELOCITY);
  MAX_ERROR_IMU_BIAS_ACCELERATION =
    cfgor.GetArgument(
    "param_max_error_imu_bias_acceleration",
    MAX_ERROR_IMU_BIAS_ACCELERATION);
  MAX_ERROR_IMU_BIAS_GYROSCOPE =
    cfgor.GetArgument(
    "param_max_error_imu_bias_gyroscope",
    MAX_ERROR_IMU_BIAS_GYROSCOPE);
  MAX_ERROR_DRIFT_ROTATION =
    cfgor.GetArgument(
    "param_max_error_drift_rotation",
    MAX_ERROR_DRIFT_ROTATION);
  MAX_ERROR_DRIFT_POSITION =
    cfgor.GetArgument(
    "param_max_error_drift_position",
    MAX_ERROR_DRIFT_POSITION);

  VW_WINDOW_WIDTH =
    cfgor.GetArgument(
    "param_vm_window_width",
    VW_WINDOW_WIDTH);
  VW_WINDOW_HEIGHT =
    cfgor.GetArgument(
    "param_vm_window_height",
    VW_WINDOW_HEIGHT);
  VM_WINDOW_ASPECT_RATIO =
    cfgor.GetArgument(
    "param_vm_window_aspect_ratio",
    VM_WINDOW_ASPECT_RATIO);
  VW_WINDOW_DEPTH =
    cfgor.GetArgument(
    "param_vm_window_depth",
    VW_WINDOW_DEPTH);
  VW_PROJECTION_MAX_VIEW_ANGLE =
    cfgor.GetArgumentRadian(
    "param_vw_projection_max_view_angle",
    VW_PROJECTION_MAX_VIEW_ANGLE);
  VW_CAMERA_DEPTH_TO_SCENE_DEPTH_RATIO =
    cfgor.GetArgument(
    "param_vm_camera_depth_to_scene_depth_ratio",
    VW_CAMERA_DEPTH_TO_SCENE_DEPTH_RATIO);
  VW_CAMERA_SIZE_ACTIVE_RATIO =
    cfgor.GetArgument(
    "param_vm_camera_size_active_ratio",
    VW_CAMERA_SIZE_ACTIVE_RATIO);
  VW_ARCBALL_RADIUS_TO_WINDOW_SIZE_RATIO =
    cfgor.GetArgument(
    "param_vm_arcball_radius_to_window_size_ratio",
    VW_ARCBALL_RADIUS_TO_WINDOW_SIZE_RATIO);
  VW_SCROLL_TRANSLATTON_Z_RATE =
    cfgor.GetArgument(
    "param_vm_scroll_translation_z_rate",
    VW_SCROLL_TRANSLATTON_Z_RATE);

  TIME_AVERAGING_COUNT =
    cfgor.GetArgument(
    "param_time_averaging_count",
    TIME_AVERAGING_COUNT);
}
