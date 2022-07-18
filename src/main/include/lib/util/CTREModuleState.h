#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>

class CTREModuleState {

    public:
        static frc::SwerveModuleState optimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle);

    private:
        static double placeInAppropriate0To360Scope(double scopeReference, double newAngle);
};