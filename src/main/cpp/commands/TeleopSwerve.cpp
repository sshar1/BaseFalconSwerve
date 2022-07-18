// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../include/commands/TeleopSwerve.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "../include/Constants.h"
#include "subsystems/Swerve.h"
#include <frc/Joystick.h>
#include <frc/geometry/Translation2d.h>

using meters = units::meter_t;
using radians = units::radian_t;

TeleopSwerve::TeleopSwerve(Swerve* swerve, frc::Joystick* controller, int translationAxis, int strafeAxis, int rotationAxis, bool fieldRelative, bool openLoop)
    : m_swerve {swerve}
    , m_controller {controller}
    , m_translationAxis {translationAxis}
    , m_strafeAxis {strafeAxis}
    , m_rotationAxis {rotationAxis}
    , m_fieldRelative {fieldRelative}
    , m_openLoop {openLoop} 
    
    {
        AddRequirements(m_swerve);
    }

void TeleopSwerve::Execute() {
    double yAxis {-m_controller->GetRawAxis(m_translationAxis)};
    double xAxis {-m_controller->GetRawAxis(m_strafeAxis)};
    double rAxis {-m_controller->GetRawAxis(m_rotationAxis)};

    /* Deadbands */
    yAxis = (abs(yAxis) < constants::kStickDeadband) ? 0 : yAxis;
    xAxis = (abs(xAxis) < constants::kStickDeadband) ? 0 : xAxis;
    rAxis = (abs(rAxis) < constants::kStickDeadband) ? 0 : rAxis;

    m_translation = {static_cast<meters>(yAxis), static_cast<meters>(xAxis)};
    m_translation = m_translation * constants::swerve::maxSpeed.value();
    m_rotation = {static_cast<radians>(rAxis) * constants::swerve::maxAnglularVelocity};

    m_swerve->drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);
}