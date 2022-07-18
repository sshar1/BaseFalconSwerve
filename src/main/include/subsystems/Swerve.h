// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

class Swerve : public frc2::SubsystemBase {

  using swerveModArr_t = std::array<SwerveModule, 4> ;

 public:
  
  frc::SwerveDriveOdometry<4> m_swerveOdometry {constants::swerve::swerveKinematics, getYaw()}; // 4 modules
  swerveModArr_t m_swerveMods {
    SwerveModule {0, constants::swerve::mod0::constants},
    SwerveModule {1, constants::swerve::mod1::constants},
    SwerveModule {2, constants::swerve::mod2::constants},
    SwerveModule {3, constants::swerve::mod3::constants}
  };
  PigeonIMU gyro {constants::swerve::kPigeon};

  Swerve();

  void drive(frc::Translation2d& translation, units::radians_per_second_t rotation, bool fieldRelative, bool isOpenLoop);

  void setModuleStates(wpi::array<frc::SwerveModuleState, 4>& desiredStates);

  frc::Pose2d getPose();

  void resetOdometry(frc::Pose2d& pose);

  wpi::array<frc::SwerveModuleState, 4>  getStates();

  void zeroGyro();

  frc::Rotation2d getYaw();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  //void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
