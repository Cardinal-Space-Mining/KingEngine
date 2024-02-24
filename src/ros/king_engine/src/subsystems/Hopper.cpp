// This class is for the Hopper and processes involved
// Should control Hopper movement while offloading and mining
// Should keep track of belts inside hopper
// Zeros when needed
// Tracks current from motor to monitor high power usage
// Timer to know how long to mine and offload
// Assume that we will have access to correct location

#include "king_engine/subsystems/Hopper.h"
#include "king_engine/subsystems/subsystem_util.h"
#include "king_engine/subsystems/Collection.h"


Hopper::Hopper() {
//    talonInit(hopperMotor);
//    talonSRXInit(actuator);
//    hopperTab.add("Unload Ladder Encoder Value", unloadEncoder);
//    hopperMotor.setInverted(true);
}

void Hopper::periodic() {
//    // This method will be called once per scheduler run
//    unloadEncoder = hopperMotor.getSensorCollection().getIntegratedSensorPosition();
//    unloadCurrent = hopperMotor.getStatorCurrent();
//
//    hopEncoder = unloadEncoder;
//    hopCurrent = unloadCurrent;
//    revLimit = (actuator.isRevLimitSwitchClosed() == 1);
//    actuatorCurrent = actuator.getStatorCurrent();
//    actuatorGet = actuator.get();
//    isAlive = actuator.isAlive();
//    supplyCurrent = actuator.getSupplyCurrent();
//    outVoltage = actuator.getMotorOutputVoltage();
//    outPercent = actuator.getMotorOutputPercent();
}