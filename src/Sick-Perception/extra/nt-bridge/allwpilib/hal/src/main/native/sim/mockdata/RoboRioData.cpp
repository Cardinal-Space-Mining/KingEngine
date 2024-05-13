// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../PortsInternal.h"
#include "RoboRioDataInternal.h"

using namespace hal;

namespace hal::init {
void InitializeRoboRioData() {
  static RoboRioData srrd;
  ::hal::SimRoboRioData = &srrd;
}
}  // namespace hal::init

RoboRioData* hal::SimRoboRioData;
void RoboRioData::ResetData() {
  fpgaButton.Reset(false);
  vInVoltage.Reset(12.0);
  vInCurrent.Reset(0.0);
  userVoltage6V.Reset(6.0);
  userCurrent6V.Reset(0.0);
  userActive6V.Reset(true);
  userVoltage5V.Reset(5.0);
  userCurrent5V.Reset(0.0);
  userActive5V.Reset(true);
  userVoltage3V3.Reset(3.3);
  userCurrent3V3.Reset(0.0);
  userActive3V3.Reset(true);
  userFaults6V.Reset(0);
  userFaults5V.Reset(0);
  userFaults3V3.Reset(0);
  brownoutVoltage.Reset(6.75);
  cpuTemp.Reset(45.0);
  teamNumber.Reset(0);
  m_serialNumber = "";
  m_comments = "";
}

int32_t RoboRioData::RegisterSerialNumberCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  std::scoped_lock lock(m_serialNumberMutex);
  int32_t uid = m_serialNumberCallbacks.Register(callback, param);
  if (initialNotify) {
    callback(GetSerialNumberName(), param, m_serialNumber.c_str(),
             m_serialNumber.size());
  }
  return uid;
}

void RoboRioData::CancelSerialNumberCallback(int32_t uid) {
  m_serialNumberCallbacks.Cancel(uid);
}

size_t RoboRioData::GetSerialNumber(char* buffer, size_t size) {
  std::scoped_lock lock(m_serialNumberMutex);
  size_t copied = m_serialNumber.copy(buffer, size);
  // Null terminate
  if (copied == size) {
    copied -= 1;
  }
  buffer[copied] = '\0';
  return copied;
}

void RoboRioData::SetSerialNumber(const char* serialNumber, size_t size) {
  // Limit serial number to 8 characters internally- serialnum environment
  // variable is always 8 characters
  if (size > 8) {
    size = 8;
  }
  std::scoped_lock lock(m_serialNumberMutex);
  m_serialNumber = std::string(serialNumber, size);
  m_serialNumberCallbacks(m_serialNumber.c_str(), m_serialNumber.size());
}

int32_t RoboRioData::RegisterCommentsCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  std::scoped_lock lock(m_commentsMutex);
  int32_t uid = m_commentsCallbacks.Register(callback, param);
  if (initialNotify) {
    callback(GetCommentsName(), param, m_comments.c_str(),
             m_serialNumber.size());
  }
  return uid;
}

void RoboRioData::CancelCommentsCallback(int32_t uid) {
  m_commentsCallbacks.Cancel(uid);
}

size_t RoboRioData::GetComments(char* buffer, size_t size) {
  std::scoped_lock lock(m_commentsMutex);
  size_t copied = m_comments.copy(buffer, size);
  // Null terminate if there is room
  if (copied < size) {
    buffer[copied] = '\0';
  }
  return copied;
}

void RoboRioData::SetComments(const char* comments, size_t size) {
  if (size > 64) {
    size = 64;
  }
  std::scoped_lock lock(m_commentsMutex);
  m_comments = std::string(comments, size);
  m_commentsCallbacks(m_comments.c_str(), m_comments.size());
}

extern "C" {
void HALSIM_ResetRoboRioData(void) {
  SimRoboRioData->ResetData();
}

#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                          \
  HAL_SIMDATAVALUE_DEFINE_CAPI_NOINDEX(TYPE, HALSIM, RoboRio##CAPINAME, \
                                       SimRoboRioData, LOWERNAME)

DEFINE_CAPI(HAL_Bool, FPGAButton, fpgaButton)
DEFINE_CAPI(double, VInVoltage, vInVoltage)
DEFINE_CAPI(double, VInCurrent, vInCurrent)
DEFINE_CAPI(double, UserVoltage6V, userVoltage6V)
DEFINE_CAPI(double, UserCurrent6V, userCurrent6V)
DEFINE_CAPI(HAL_Bool, UserActive6V, userActive6V)
DEFINE_CAPI(double, UserVoltage5V, userVoltage5V)
DEFINE_CAPI(double, UserCurrent5V, userCurrent5V)
DEFINE_CAPI(HAL_Bool, UserActive5V, userActive5V)
DEFINE_CAPI(double, UserVoltage3V3, userVoltage3V3)
DEFINE_CAPI(double, UserCurrent3V3, userCurrent3V3)
DEFINE_CAPI(HAL_Bool, UserActive3V3, userActive3V3)
DEFINE_CAPI(int32_t, UserFaults6V, userFaults6V)
DEFINE_CAPI(int32_t, UserFaults5V, userFaults5V)
DEFINE_CAPI(int32_t, UserFaults3V3, userFaults3V3)
DEFINE_CAPI(double, BrownoutVoltage, brownoutVoltage)
DEFINE_CAPI(double, CPUTemp, cpuTemp)
DEFINE_CAPI(int32_t, TeamNumber, teamNumber)
DEFINE_CAPI(HAL_RadioLEDState, RadioLEDState, radioLedState)

int32_t HALSIM_RegisterRoboRioSerialNumberCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  return SimRoboRioData->RegisterSerialNumberCallback(callback, param,
                                                      initialNotify);
}
void HALSIM_CancelRoboRioSerialNumberCallback(int32_t uid) {
  return SimRoboRioData->CancelSerialNumberCallback(uid);
}
size_t HALSIM_GetRoboRioSerialNumber(char* buffer, size_t size) {
  return SimRoboRioData->GetSerialNumber(buffer, size);
}
void HALSIM_SetRoboRioSerialNumber(const char* serialNumber, size_t size) {
  SimRoboRioData->SetSerialNumber(serialNumber, size);
}

int32_t HALSIM_RegisterRoboRioCommentsCallback(
    HAL_RoboRioStringCallback callback, void* param, HAL_Bool initialNotify) {
  return SimRoboRioData->RegisterCommentsCallback(callback, param,
                                                  initialNotify);
}
void HALSIM_CancelRoboRioCommentsCallback(int32_t uid) {
  SimRoboRioData->CancelCommentsCallback(uid);
}
size_t HALSIM_GetRoboRioComments(char* buffer, size_t size) {
  return SimRoboRioData->GetComments(buffer, size);
}
void HALSIM_SetRoboRioComments(const char* comments, size_t size) {
  SimRoboRioData->SetComments(comments, size);
}

void HALSIM_RegisterRoboRioAllCallbacks(HAL_NotifyCallback callback,
                                        void* param, HAL_Bool initialNotify);

#define REGISTER(NAME) \
  SimRoboRioData->NAME.RegisterCallback(callback, param, initialNotify)

void HALSIM_RegisterRoboRioAllCallbacks(HAL_NotifyCallback callback,
                                        void* param, HAL_Bool initialNotify) {
  REGISTER(fpgaButton);
  REGISTER(vInVoltage);
  REGISTER(vInCurrent);
  REGISTER(userVoltage6V);
  REGISTER(userCurrent6V);
  REGISTER(userActive6V);
  REGISTER(userVoltage5V);
  REGISTER(userCurrent5V);
  REGISTER(userActive5V);
  REGISTER(userVoltage3V3);
  REGISTER(userCurrent3V3);
  REGISTER(userActive3V3);
  REGISTER(userFaults6V);
  REGISTER(userFaults5V);
  REGISTER(userFaults3V3);
  REGISTER(brownoutVoltage);
  REGISTER(cpuTemp);
  REGISTER(radioLedState);
}
}  // extern "C"
