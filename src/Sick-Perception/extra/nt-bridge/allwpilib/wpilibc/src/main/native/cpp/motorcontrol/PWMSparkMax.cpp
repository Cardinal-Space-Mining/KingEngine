// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/motorcontrol/PWMSparkMax.h"

#include <hal/FRCUsageReporting.h>

using namespace frc;

PWMSparkMax::PWMSparkMax(int channel)
    : PWMMotorController("PWMSparkMax", channel) {
  m_pwm.SetBounds(2.003_ms, 1.55_ms, 1.50_ms, 1.46_ms, 0.999_ms);
  m_pwm.SetPeriodMultiplier(PWM::kPeriodMultiplier_1X);
  m_pwm.SetSpeed(0.0);
  m_pwm.SetZeroLatch();

  HAL_Report(HALUsageReporting::kResourceType_RevSparkMaxPWM, GetChannel() + 1);
}
