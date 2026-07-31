// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FUSION__FUSIONCALIBRATION_H_
#define FUSION__FUSIONCALIBRATION_H_

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.h"

//------------------------------------------------------------------------------
// Inline functions

/**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param misalignment Misalignment matrix.
 * @param sensitivity Sensitivity.
 * @param offset Offset.
 * @return Calibrated measurement.
 */
static inline FusionVector FusionCalibrationInertial(
  const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity,
  const FusionVector offset)
{
  return FusionMatrixMultiplyVector(
    misalignment,
    FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
}

/**
 * @brief Magnetometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param softIronMatrix Soft-iron matrix.
 * @param hardIronOffset Hard-iron offset.
 * @return Calibrated measurement.
 */
static inline FusionVector FusionCalibrationMagnetic(
  const FusionVector uncalibrated, const FusionMatrix softIronMatrix,
  const FusionVector hardIronOffset)
{
  return FusionMatrixMultiplyVector(
    softIronMatrix, FusionVectorSubtract(uncalibrated, hardIronOffset));
}

#endif  // FUSION__FUSIONCALIBRATION_H_
