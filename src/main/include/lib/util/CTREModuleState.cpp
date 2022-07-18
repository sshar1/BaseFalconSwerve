#include "CTREModuleState.h"
#include "units/length.h"
#include <cstdlib>

class CTREModuleState {

    public:
        static frc::SwerveModuleState optimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle) {
            units::degree_t targetAngle = placeInAppropriate0To360Scope(currentAngle.Degrees(), desiredState.angle.Degrees());
            units::meters_per_second_t targetSpeed = desiredState.speed;
            double delta = static_cast<int>(targetAngle) - currentAngle.Degrees();

            if (abs(delta) > 90){
                targetSpeed = -targetSpeed;
                targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
            }

            return frc::SwerveModuleState {targetSpeed, frc::Rotation2d{targetAngle}};
        }

    private:
        static units::degree_t placeInAppropriate0To360Scope(const units::degree_t scopeReference, units::degree_t newAngle) {
            double lowerBound {};
            double upperBound {};

            double lowerOffset {static_cast<int>(scopeReference) % 360};

            if (lowerOffset >= 0) {
                lowerBound = scopeReference - lowerOffset;
                upperBound = scopeReference + (360 - lowerOffset);
            } else {
                upperBound = scopeReference - lowerOffset;
                lowerBound = scopeReference - (360 + lowerOffset);
            }

            while (static_cast<int>(newAngle) < lowerBound) {
                newAngle += 360;
            }
            while (static_cast<int>(newAngle) > upperBound) {
                newAngle -= 360;
            }

            if (static_cast<int>(newAngle - scopeReference) > 180) {
                newAngle -= 360;
            } else if (static_cast<int>(newAngle - scopeReference) < -180) {
                newAngle += 360;
            }

            return newAngle;
        }
};