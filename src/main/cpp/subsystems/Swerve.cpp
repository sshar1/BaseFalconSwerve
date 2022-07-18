// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../include/subsystems/Swerve.h"

#include <ctre/phoenix/sensors/PigeonIMU.h>

#include "../include/SwerveModule.h"
#include "../include/Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

using swerveModArr_t = std::array<SwerveModule, 4>;

Swerve::Swerve() {
  gyro.ConfigFactoryDefault();
  zeroGyro();
}

void Swerve::drive(frc::Translation2d& translation, units::radians_per_second_t rotation, bool fieldRelative, bool isOpenLoop) {
  wpi::array<frc::SwerveModuleState, 4> swerveModuleStates {
    constants::swerve::swerveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(translation.X(), translation.Y(), rotation, getYaw())
    )
  };

  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&swerveModuleStates, constants::swerve::maxSpeed);

  for (SwerveModule& mod : m_swerveMods) {
    mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
  }
}

void Swerve::setModuleStates(wpi::array<frc::SwerveModuleState, 4>& desiredStates) {
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&desiredStates, constants::swerve::maxSpeed);

  for (SwerveModule& mod : m_swerveMods) {
    mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
  }
}

frc::Pose2d Swerve::getPose() {
  return m_swerveOdometry.GetPose();
}

void Swerve::resetOdometry(frc::Pose2d& pose) {
  m_swerveOdometry.ResetPosition(pose, getYaw());
}

wpi::array<frc::SwerveModuleState, 4> Swerve::getStates() {

  return {
    m_swerveMods[0].getState(),
    m_swerveMods[1].getState(),
    m_swerveMods[2].getState(),
    m_swerveMods[3].getState(),
  };
}

void Swerve::zeroGyro() {
  gyro.SetYaw(0);
}

frc::Rotation2d Swerve::getYaw() {
  double ypr[3];
  gyro.GetYawPitchRoll(ypr);
  return constants::swerve::kInvertGyro ? frc::Rotation2d {static_cast<units::degree_t>(360 - ypr[0])} : frc::Rotation2d {static_cast<units::degree_t>(ypr[0])};
}

void Swerve::Periodic() {
  m_swerveOdometry.Update(getYaw(), getStates());

  for (SwerveModule& mod : m_swerveMods) {
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod.m_moduleNumber) + " Cancoder", mod.getCanCoder().Degrees().value());
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod.m_moduleNumber) + " Integrated", mod.getState().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod.m_moduleNumber) + " Velocity", mod.getState().speed.value());
  }
}