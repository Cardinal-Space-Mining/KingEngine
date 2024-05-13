// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/motorcontrol/Victor.h"

#include <hal/FRCUsageReporting.h>

using namespace frc;

Victor::Victor(int channel) : PWMMotorController("Victor", channel) {
  m_pwm.SetBounds(2.027_ms, 1.525_ms, 1.507_ms, 1.49_ms, 1.026_ms);
  m_pwm.SetPeriodMultiplier(PWM::kPeriodMultiplier_2X);
  m_pwm.SetSpeed(0.0);
  m_pwm.SetZeroLatch();

  HAL_Report(HALUsageReporting::kResourceType_Victor, GetChannel() + 1);
}
