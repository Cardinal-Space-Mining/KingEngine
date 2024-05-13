// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/xrp/XRPGyro.h"

using namespace frc;

XRPGyro::XRPGyro() : m_simDevice("Gyro:XRPGyro") {
  if (m_simDevice) {
    m_simDevice.CreateBoolean("init", hal::SimDevice::kOutput, true);
    m_simRateX =
        m_simDevice.CreateDouble("rate_x", hal::SimDevice::kInput, 0.0);
    m_simRateY =
        m_simDevice.CreateDouble("rate_y", hal::SimDevice::kInput, 0.0);
    m_simRateZ =
        m_simDevice.CreateDouble("rate_z", hal::SimDevice::kInput, 0.0);
    m_simAngleX =
        m_simDevice.CreateDouble("angle_x", hal::SimDevice::kInput, 0.0);
    m_simAngleY =
        m_simDevice.CreateDouble("angle_y", hal::SimDevice::kInput, 0.0);
    m_simAngleZ =
        m_simDevice.CreateDouble("angle_z", hal::SimDevice::kInput, 0.0);
  }
}

double XRPGyro::GetAngle() const {
  return GetAngleZ();
}

double XRPGyro::GetRate() const {
  return GetRateZ();
}

double XRPGyro::GetRateX() const {
  if (m_simRateX) {
    return m_simRateX.Get();
  }

  return 0.0;
}

double XRPGyro::GetRateY() const {
  if (m_simRateY) {
    return m_simRateY.Get();
  }

  return 0.0;
}

double XRPGyro::GetRateZ() const {
  if (m_simRateZ) {
    return m_simRateZ.Get();
  }

  return 0.0;
}

double XRPGyro::GetAngleX() const {
  if (m_simAngleX) {
    return m_simAngleX.Get() - m_angleXOffset;
  }

  return 0.0;
}

double XRPGyro::GetAngleY() const {
  if (m_simAngleY) {
    return m_simAngleY.Get() - m_angleYOffset;
  }

  return 0.0;
}

double XRPGyro::GetAngleZ() const {
  if (m_simAngleZ) {
    return m_simAngleZ.Get() - m_angleZOffset;
  }

  return 0.0;
}

void XRPGyro::Reset() {
  if (m_simAngleX) {
    m_angleXOffset = m_simAngleX.Get();
    m_angleYOffset = m_simAngleY.Get();
    m_angleZOffset = m_simAngleZ.Get();
  }
}
