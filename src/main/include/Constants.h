// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#define _USE_MATH_DEFINES
 
#include <cmath>
#include <units/length.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include "ctre/phoenix.h"
#include "../include/lib/util/SwerveModuleConstants.h"
#include <frc/controller/SimpleMotorFeedforward.h>

namespace constants {

    using t_port = int;
    using meters = units::meter_t;

    inline constexpr double kStickDeadband {0.1};

    namespace swerve {

        using translation = frc::Translation2d;
        using ctre::phoenix::motorcontrol::NeutralMode;

        inline constexpr t_port kPigeon {1};
        inline constexpr bool kInvertGyro {false};

        inline constexpr meters kTrackWidth {21.73_in};
        inline constexpr meters kWheelBase {21.73_in};
        inline constexpr meters kWheelDiameter = {3.94_in};
        inline constexpr meters kWheelCircumference {kWheelDiameter * M_PI};

        inline constexpr double kOpenLoopRamp {0.25};
        inline constexpr double kClosedLoopRamp {0.0};

        inline constexpr double kDriveGearRatio {6.86 / 1.0};
        inline constexpr double kAngleGearRatio {12.8 / 1.0};

        inline frc::SwerveDriveKinematics swerveKinematics {
            translation {kWheelBase / 2.0, kTrackWidth / 2.0},
            translation {kWheelBase / 2.0, -kTrackWidth / 2.0},
            translation {-kWheelBase / 2.0, kTrackWidth / 2.0},
            translation {-kWheelBase / 2.0, -kTrackWidth / 2.0}
        };

        /* Swerve Current Limiting */
        inline constexpr int kAngleContinuousCurrentLimit {25};
        inline constexpr int kAnglePeakCurrentLimit {40};
        inline constexpr double kAnglePeakCurrentDuration {0.1};
        inline constexpr bool kAngleEnableCurrentLimit {true};

        inline constexpr int kDriveContinuousCurrentLimit {35};
        inline constexpr int kDrivePeakCurrentLimit {60};
        inline constexpr double kDrivePeakCurrentDuration {0.1};
        inline constexpr bool kDriveEnableCurrentLimit {true};

        /* Angle Motor PID Values */
        inline constexpr double angleKP {0.6};
        inline constexpr double angleKI {0.0};
        inline constexpr double angleKD {12.0};
        inline constexpr double angleKF {0.0};

        /* Drive Motor PID Values */
        inline constexpr double driveKP {0.1};
        inline constexpr double driveKI {0.0};
        inline constexpr double driveKD {0.0};
        inline constexpr double driveKF {0.0};

        /* Drive Motor Characterization Values */
        // using acceleration = units::compound_unit<Velocity, units::inverse<units::seconds> >;
        // inline constexpr units::volt_t driveKS {0.667 / 12};
        // inline constexpr units::compound_unit<units::volts, units::inverse<acceleration>> driveKV {2.44 / 12};
        // inline constexpr double driveKA {0.27 / 12};

        /* Swerve Profiling Values */
        inline constexpr auto maxSpeed {4.5_mps};
        inline constexpr auto maxAnglularVelocity {11.5_mps};

        /* Neutral Modes */
        inline constexpr NeutralMode angleNeutralMode {NeutralMode::Coast};
        inline constexpr NeutralMode driveNeutralMode {NeutralMode::Brake};

        /* Motor Inverts */
        inline constexpr bool driveMotorInvert {false};
        inline constexpr bool angleMotorInvert {false};

        /* Angle Encoder Invert */
        inline constexpr bool canCoderInvert {false};

        /*Module Specific Constants */
        /* Front Left Module - Module 0 */
        namespace mod0 {
            inline constexpr t_port driveMotorID {1};
            inline constexpr t_port angleMotorID {2};
            inline constexpr t_port canCoderID {1};
            inline constexpr double angleOffset {37.35};

            inline constexpr SwerveModuleConstants constants {
                driveMotorID, angleMotorID, canCoderID, angleOffset
            };
        }

        /* Front Right Module - Module 1 */
        namespace mod1 {
            inline constexpr t_port driveMotorID {3};
            inline constexpr t_port angleMotorID {4};
            inline constexpr t_port canCoderID {2};
            inline constexpr double angleOffset {10.34};

            inline constexpr SwerveModuleConstants constants {
                driveMotorID, angleMotorID, canCoderID, angleOffset
            };
        }

        /* Back Left Module - Module 2 */
        namespace mod2 {
            inline constexpr t_port driveMotorID {5};
            inline constexpr t_port angleMotorID {6};
            inline constexpr t_port canCoderID {3};
            inline constexpr double angleOffset {38.75};

            inline constexpr SwerveModuleConstants constants {
                driveMotorID, angleMotorID, canCoderID, angleOffset
            };
        }

        /* Back Right Module - Module 3 */
        namespace mod3 {
            inline constexpr t_port driveMotorID {7};
            inline constexpr t_port angleMotorID {8};
            inline constexpr t_port canCoderID {4};
            inline constexpr double angleOffset {58.88};

            inline constexpr SwerveModuleConstants constants {
                driveMotorID, angleMotorID, canCoderID, angleOffset
            };
        }
    }
}

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
