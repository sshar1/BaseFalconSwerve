#pragma once

using velocity_t = units::velocity::meters_per_second_t;
using meters = units::meter_t;

namespace conversions {

    const double falconToDegrees(double counts, double gearRatio);
    const double degreesToFalcon(double degrees, double gearRatio);

    const double falconToRPM(double velocityCounts, double gearRatio);
    const double RPMToFalcon(double RPM, double gearRatio);

    const double falconToMPS(double velocityCounts, double circumference, double gearRatio);
    const double MPSToFalcon(velocity_t velocity, meters circumference, double gearRatio);
}