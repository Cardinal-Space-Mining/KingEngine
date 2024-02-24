//
// Created by conno on 2/17/2024.
//

#pragma once

class Hopper {

public:
    enum HopperStatus {
        RAISED,
        LOWERED,
        UNKNOWN
    };

    Hopper(); // constructor

    void periodic();

    double getHopperMotorPosition();
    void moveActuatorPercentageControl(double percentage);
    void raiseActuator();
    void lowerActuator();
    bool actuatorIsAtTop();
    void moveHopperPercentage(double percentage);
    bool actuatorIsAtBottom();
    void moveHopperVelocity(double encoderTicks);
    void disableHopperMotor();

    void talonSRXInit(/*WPI_TalonSRX*/double _talonSRX);

    void enableActuatorDisabledControl();

    void setHopperStatus(HopperStatus hopperStatus);

    bool isLowered();
    bool isRaised();


    double sectorTarget = 0;
    double sectorEncoder = 0;
    double sectorAngle = 0;
    double sectorCurrent = 0;
    double sectorRelative = 0;

    double unloadCurrent;
    double unloadEncoder = 0;
    HopperStatus hopperStatus = UNKNOWN;

    double hopEncoder = unloadEncoder;
    double hopCurrent = 0;
    double revLimit = false;
    double actuatorCurrent = 0;
    double actuatorGet = 0;
    double isAlive = false;
    double supplyCurrent = 0;
    double outVoltage = 0;
    double outPercent = 0;

private:
    void talonInit(/*WPI_TalonFX*/ int _talon);
    void talonSRXInit(/*WPI_TalonFX*/ int _talon);

//    const WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.LINEAR_ACTUATOR_MOTOR_CONTROLLER_ID);
//    const WPI_TalonFX hopperMotor = new WPI_TalonFX(Constants.HOPPER_MOTOR_ID);

//    EncoderQueue sectorQueue;

};
