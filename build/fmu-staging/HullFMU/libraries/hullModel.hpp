#include <array>
#include <cmath>

namespace HullModel {
    std::array<double, 12> simulate(
        double timeStep,

        double inputThrusterBowForce,
        double inputThrusterBowAngle,
        double inputThrusterPortForce,
        double inputThrusterPortAngle,
        double inputThrusterStarboardForce,
        double inputThrusterStarboardAngle,
        double inputWindSpeed,
        double inputWindAngle,
        double inputWaterCurrentSpeed,
        double inputWaterCurrentAngle,
        std::array<double, 5> inputWaveHeight,

        double parameterHullMass,
        double parameterHullLength,
        double parameterHullWidth,
        double parameterHullHeight,
        double parameterThrusterBowPositionX,
        double parameterThrusterPortPositionX,
        double parameterThrusterPortPositionY,
        double parameterThrusterStarboardPositionX,
        double parameterThrusterStarboardPositionY,

        std::array<double, 6> OutputHullPosition,
        std::array<double, 6> OutputHullVelocity
    );
}