#pragma once

#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>

class CTREConfigs {
    public:

        ctre::phoenix::motorcontrol::can::TalonFXConfiguration swerveAngleFXConfig;
        ctre::phoenix::motorcontrol::can::TalonFXConfiguration swerveDriveFXConfig;
        ctre::phoenix::sensors::CANCoderConfiguration swerveCanCoderConfig;

        CTREConfigs();
};