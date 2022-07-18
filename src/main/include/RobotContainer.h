// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "../include/commands/TeleopSwerve.h"
#include "../include/subsystems/Swerve.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
  public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();

  private:
    // The robot's subsystems and commands are defined here...
    frc::Joystick driver {0};

    const int translationAxis {frc::XboxController::Axis::kLeftY};
    const int strafeAxis {frc::XboxController::Axis::kLeftX};
    const int rotationAxis {frc::XboxController::Axis::kRightX};

    frc2::JoystickButton zeroGyro {&frc::GenericHID{0}, frc::XboxController::Button::kY}; // genericHID same port as driver

    Swerve m_swerve;

    void ConfigureButtonBindings();

};
