#pragma once

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "../include/lib/math/Conversions.h"
#include "../include/lib/util/CTREModuleState.h"
#include "../include/lib/util/SwerveModuleConstants.h"

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/DemandType.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class SwerveModule {
    private:
        double m_angleOffset {};
        TalonFX m_angleMotor;
        TalonFX m_driveMotor;
        CANCoder m_angleEncoder;
        double m_lastAngle {};

        void resetToAbsolute();

        void configAngleEncoder();

        void configAngleMotor();

        void configDriveMotor();

    public:
        int m_moduleNumber {};
        //frc::SimpleMotorFeedforward feedForward {constants::swerve::driveKS, constants::swerve::driveKV, constants::swerve::driveKA};

        SwerveModule(int moduleNumber, SwerveModuleConstants& moduleConstants);

        void setDesiredState(frc::SwerveModuleState& desiredState, bool isOpenLoop);

        frc::Rotation2d getCanCoder();

        frc::SwerveModuleState getState();
};
