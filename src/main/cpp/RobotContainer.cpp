// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Command.h>

#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "../include/commands/TeleopSwerve.h"
#include "../include/subsystems/Swerve.h"

RobotContainer::RobotContainer() {
  bool fieldRelative {true};
  bool openLoop {true};

  m_swerve.SetDefaultCommand(TeleopSwerve {&m_swerve, &driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop});

  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {

  auto* tmp {&m_swerve};

  std::function<void()> cmd {
    [tmp] () -> void {
      tmp->zeroGyro();
    }
  };

  zeroGyro.WhenPressed(&frc2::InstantCommand {cmd});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return nullptr;
}
