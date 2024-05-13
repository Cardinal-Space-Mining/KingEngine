// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/motorcontrol/VictorSP.h"

#include <hal/FRCUsageReporting.h>

using namespace frc;

VictorSP::VictorSP(int channel) : PWMMotorController("VictorSP", channel) {
  m_pwm.SetBounds(2.004_ms, 1.52_ms, 1.50_ms, 1.48_ms, 0.997_ms);
  m_pwm.SetPeriodMultiplier(PWM::kPeriodMultiplier_1X);
  m_pwm.SetSpeed(0.0);
  m_pwm.SetZeroLatch();

  HAL_Report(HALUsageReporting::kResourceType_VictorSP, GetChannel() + 1);
}
