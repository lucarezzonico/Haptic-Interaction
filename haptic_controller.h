/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __HAPTIC_CONTROLLER_H
#define __HAPTIC_CONTROLLER_H

#include "main.h"

/** @defgroup HapticController Main / Haptic controller
  * @brief Main haptic paddle controller.
  *
  * This module is the high-level controller of the board. Basically, its role
  * is to process the sensors data and then compute a motor torque, in order to
  * achieve a spicific haptic effect.
  *
  * The content of hapt_Update() is typically what the user of the board will
  * modify, depending on the selected sensors and control algorithms.
  *
  * Call hapt_Init() to setup this module. Its interrupt function will be called
  * automatically periodically.
  *
  * @addtogroup HapticController
  * @{
  */

void hapt_Init(void);

// Functions only used temporarily for tuning the current loop PID
void hapt_Set_enable_position_regulator(uint8_t on);
void hapt_SetKp(float32_t Kp);
void hapt_SetKi(float32_t Ki);
void hapt_SetKd(float32_t Kd);
void hapt_SetARW(float32_t ARW);
float32_t hapt_Get_enable_position_regulator(void);
float32_t hapt_GetKp(void);
float32_t hapt_GetKd(void);
float32_t hapt_GetKi(void);
float32_t hapt_GetARW(void);

/**
  * @}
  */

#endif
