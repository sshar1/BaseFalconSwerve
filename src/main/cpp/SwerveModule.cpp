#include "../include/SwerveModule.h"
#include "Robot.h"

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "../include/lib/math/Conversions.h"
#include "../include/lib/util/CTREModuleState.h"
#include "../include/lib/util/SwerveModuleConstants.h"
#include "../include/Constants.h"
#include "../include/CTREConfigs.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/DemandType.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>

using velocity_t = units::velocity::meters_per_second_t;

SwerveModule::SwerveModule(int moduleNumber, SwerveModuleConstants& moduleConstants) 
    : m_moduleNumber {moduleNumber}
    , m_angleOffset {moduleConstants.angleOffset}
    , m_angleEncoder {moduleConstants.cancoderID}
    , m_angleMotor {moduleConstants.driveMotorID}
    , m_lastAngle {getState().angle.Degrees()}

    {
        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();
    }

void SwerveModule::setDesiredState(frc::SwerveModuleState& desiredState, bool isOpenLoop) {
    desiredState = CTREModuleState::optimize(desiredState, getState().angle);

    if (isOpenLoop) {
        double percentOutput {desiredState.speed / constants::swerve::maxSpeed};
        m_driveMotor.Set(ControlMode::PercentOutput, percentOutput);
    } else {
        velocity_t velocity {conversions::MPSToFalcon(desiredState.speed, constants::swerve::kWheelCircumference, constants::swerve::kDriveGearRatio)};
        //m_driveMotor.set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward, feedforward.calculate(desiredState.speed)); // TODO do feedforward
    }

    double angle {(abs(desiredState.speed) <= (constants::swerve::maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle.Degrees()};
    m_lastAngle = angle;
}

void SwerveModule::resetToAbsolute() {
    double absolutePosition {conversions::degreesToFalcon(getCanCoder().Degrees() - m_angleOffset, constants::swerve::kAngleGearRatio)};
    m_angleMotor.SetSelectedSensorPosition(absolutePosition);
}

void SwerveModule::configAngleEncoder() {
    m_angleEncoder.ConfigFactoryDefault();
    m_angleEncoder.ConfigAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
}

void SwerveModule::configAngleMotor() {
    m_angleMotor.ConfigFactoryDefault();
    m_angleMotor.ConfigAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    m_angleMotor.SetInverted(constants::swerve::angleMotorInvert);
    m_angleMotor.SetNeutralMode(constants::swerve::angleNeutralMode);
    resetToAbsolute();
}

void SwerveModule::configDriveMotor() {
    m_driveMotor.ConfigFactoryDefault();
    m_driveMotor.ConfigAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    m_driveMotor.SetInverted(constants::swerve::driveMotorInvert);
    m_driveMotor.SetNeutralMode(constants::swerve::driveNeutralMode);
    m_driveMotor.SetSelectedSensorPosition(0);
}

frc::Rotation2d SwerveModule::getCanCoder() {
    return frc::Rotation2d {m_angleEncoder.GetAbsolutePosition()};
}

frc::SwerveModuleState SwerveModule::getState() {
    double velocity {conversions::falconToMPS(m_driveMotor.GetSelectedSensorPosition(), constants::swerve::kWheelCircumference, constants::swerve::kDriveGearRatio)};
    frc::Rotation2d angle {conversions::falconToDegrees(m_angleMotor.GetSelectedSensorPosition(), constants::swerve::kAngleGearRatio)};
    
    return frc::SwerveModuleState {velocity, angle};
}