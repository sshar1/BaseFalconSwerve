// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "../include/Constants.h"
#include "subsystems/Swerve.h"
#include <frc/Joystick.h>
#include <frc/geometry/Translation2d.h>

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopSwerve
    : public frc2::CommandHelper<frc2::CommandBase, TeleopSwerve> {

  private:
    units::radian_t m_rotation;
    frc::Translation2d m_translation;
    bool m_fieldRelative;
    bool m_openLoop;

    Swerve* m_swerve;
    frc::Joystick* m_controller;
    int m_translationAxis;
    int m_strafeAxis;
    int m_rotationAxis;

  public:
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit TeleopSwerve(Swerve* swerve, frc::Joystick* controller, int translationAxis, int strafeAxis, int rotationAxis, bool fieldRelative, bool openLoop);

    void Execute() override;
};
