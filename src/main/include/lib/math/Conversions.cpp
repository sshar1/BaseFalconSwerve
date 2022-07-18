#include "Conversions.h"
#include "units/velocity.h"

namespace conversions {

    const double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    const double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }


    const double falconToRPM(double velocityCounts, double gearRatio) {
        return (velocityCounts * (600.0 / 2048.0)) / gearRatio;
    }

    const double RPMToFalcon(double RPM, double gearRatio) {
        return (RPM * gearRatio) * (2048.0 / 600.0);
    }


    const double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocityCounts, gearRatio);
        return (wheelRPM * circumference) / 60;
    }

    const double MPSToFalcon(units::velocity::meters_per_second_t velocity, units::meter_t circumference, double gearRatio) {
        double wheelRPM = (velocity.value() * 60) / circumference.value());
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
}