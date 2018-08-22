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
#ifndef _VIEWER_IBA_KEY_H_
#define _VIEWER_IBA_KEY_H_

#ifdef __linux__
#define VW_KEY_CTRL_KEY_VALUE                         65507  // CTRL
#define VW_KEY_XD_DRAW_VIEW_TYPE                      65289  // Tab
#else
#define VW_KEY_XD_DRAW_VIEW_TYPE                        9   // Tab
#endif

#define VW_KEY_XD_PAUSE                                 19  // Ctrl + s
#define VW_KEY_XD_STEP                                  1   // Ctrl + a
#define VW_KEY_XD_SAVE                                  's'
#define VW_KEY_XD_SCREEN                                'S'
#define VW_KEY_XD_ACTIVATE_NEXT_FRAME                   'f'
#define VW_KEY_XD_ACTIVATE_LAST_FRAME                   'd'
#define VW_KEY_XD_ACTIVATE_KEY_FRAME                    'r'
#define VW_KEY_XD_ACTIVATE_LOCAL_FRAME                  't'
#define VW_KEY_XD_ACTIVATE_CURRENT_FRAME                'y'
#define VW_KEY_XD_DRAW_CAMERA_TYPE_GROUND_TRUTH           7   // Ctrl + g
#define VW_KEY_XD_DRAW_CAMERA_TYPE_NEXT                 'V'
#define VW_KEY_XD_DRAW_CAMERA_TYPE_LAST                 'C'
#define VW_KEY_XD_DRAW_DEPTH_TYPE_NEXT                  'N'
#define VW_KEY_XD_DRAW_DEPTH_TYPE_LAST                  'B'
#define VW_KEY_XD_DRAW_STRING                           12    // Ctrl + l
#define VW_KEY_XD_DRAW_TIME_LINE_NEXT                   'l'
#define VW_KEY_XD_DRAW_TIME_LINE_LAST                   'k'
#define VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_1  ']'
#define VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_1  '['
#define VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_2  125   // Ctrl + ]
#define VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_2  123   // Ctrl + [
#define VM_KEY_XD_PRINT_CALIBRATION                     3     // Ctrl + c
#define VW_KEY_XD_INPUT_ACTIVE_FEATURE                  24    // Ctrl + x

#define VW_KEY_XD_DRAW_AXIS                             'a'
#define VW_KEY_XD_DRAW_AXIS_LENGTH_INCREASE             '.'
#define VW_KEY_XD_DRAW_AXIS_LENGTH_DECREASE             ','
#define VW_KEY_XD_DRAW_DEPTH_PLANE                      'n'
#define VW_KEY_XD_DRAW_DEPTH_VARIANCE_INCREASE          '+'
#define VW_KEY_XD_DRAW_DEPTH_VARIANCE_DECREASE          '-'

#ifdef __linux__
#define VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_INCREASE 65361 // Left
#define VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_DECREASE 65363 // Right
#define VW_KEY_XD_DRAW_COVARIANCE_SCALE_INCREASE       65362 // Up
#define VW_KEY_XD_DRAW_COVARIANCE_SCALE_DECREASE       65364 // Down
#else
#define VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_INCREASE  37  // Left
#define VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_DECREASE  39  // Right
#define VW_KEY_XD_DRAW_COVARIANCE_SCALE_INCREASE        38  // Up
#define VW_KEY_XD_DRAW_COVARIANCE_SCALE_DECREASE        40  // Down
#endif

#define VW_KEY_PROFILE_ACTIVATE                   ' '
#define VM_KEY_PROFILE_LEVEL_1_NEXT               'R'
#define VM_KEY_PROFILE_LEVEL_1_LAST               'E'
#define VM_KEY_PROFILE_LEVEL_2_NEXT               'F'
#define VM_KEY_PROFILE_LEVEL_2_LAST               'D'
#define VM_KEY_PROFILE_LEVEL_3_NEXT               6  // Ctrl + f
#define VM_KEY_PROFILE_LEVEL_3_LAST               4  // Ctrl + d

#define VW_KEY_2D_DRAW_WARP_IMAGE                 23  // Ctrl + w
#define VW_KEY_2D_DRAW_FEATURE_TYPE               'x'
#define VW_KEY_2D_DRAW_PROJECTION_TYPE            'j'
#define VW_KEY_2D_DRAW_PROJECTION_RANGE           10  // Ctrl + j
#define VW_KEY_2D_DRAW_ERROR_TYPE                 'e'

#define VW_KEY_3D_DRAW_MOTION_TYPE                'm'
#define VW_KEY_3D_DRAW_STRUCTURE_TYPE             'M'
#define VW_KEY_3D_DRAW_DEPTH_COMPARE_TYPE         'G'
#define VW_KEY_3D_DRAW_CAMERA_SIZE_INCREASE       '.'
#define VW_KEY_3D_DRAW_CAMERA_SIZE_DECREASE       ','
#define VW_KEY_3D_DRAW_CAMERA_VELOCITY            'v'
#define VW_KEY_3D_DRAW_CAMERA_TEXTURE             'i'
#define VW_KEY_3D_DRAW_CAMERA_GROUND_TRUTH        'g'
#define VW_KEY_3D_DRAW_ERROR_TYPE                 'e'
#define VW_KEY_3D_DRAW_BACKGROUND_COLOR           11  // Ctrl + k
#define VW_KEY_3D_RESET_VIEW_POINT_ACTIVE         18  // Ctrl + r
#define VW_KEY_3D_RESET_VIEW_POINT_VIRTUAL        'R'
#endif
