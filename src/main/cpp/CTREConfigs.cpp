#include "../include/CTREConfigs.h"

#include "../include/Constants.h"

#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>

CTREConfigs::CTREConfigs() {

    using namespace ctre::phoenix::motorcontrol;
    using namespace ctre::phoenix::sensors;

    /* Swerve Angle Motor Configuration */
    SupplyCurrentLimitConfiguration angleSupplyLimit {
        constants::swerve::kAngleEnableCurrentLimit,
        constants::swerve::kAngleContinuousCurrentLimit,
        constants::swerve::kAnglePeakCurrentLimit,
        constants::swerve::kAnglePeakCurrentDuration
    };

    swerveAngleFXConfig.slot0.kP = constants::swerve::angleKP;
    swerveAngleFXConfig.slot0.kI = constants::swerve::angleKI;
    swerveAngleFXConfig.slot0.kD = constants::swerve::angleKD;
    swerveAngleFXConfig.slot0.kF = constants::swerve::angleKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit {
        constants::swerve::kDriveEnableCurrentLimit,
        constants::swerve::kDriveContinuousCurrentLimit,
        constants::swerve::kDrivePeakCurrentLimit,
        constants::swerve::kDrivePeakCurrentDuration
    };

    swerveDriveFXConfig.slot0.kP = constants::swerve::driveKP;
    swerveDriveFXConfig.slot0.kI = constants::swerve::driveKI;
    swerveDriveFXConfig.slot0.kD = constants::swerve::driveKD;
    swerveDriveFXConfig.slot0.kF = constants::swerve::driveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    swerveDriveFXConfig.openloopRamp = constants::swerve::kOpenLoopRamp;
    swerveDriveFXConfig.closedloopRamp = constants::swerve::kClosedLoopRamp;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = constants::swerve::canCoderInvert;
    swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
}