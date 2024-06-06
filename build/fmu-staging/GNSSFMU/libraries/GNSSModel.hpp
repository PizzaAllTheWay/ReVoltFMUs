#include <array>
#include <iostream>
#include <random>    // for randomness

namespace GNSSModel {
    std::array<double, 8> simulate(
        double timeStamp,

        std::array<double, 6> inputHullPosition,
        std::array<double, 6> inputHullVelocity,

        double parameterAntennaDistance,
        double parameterTimeBetweenData,
        double parameterStandardDeviation,

        std::array<double, 3> outputAntenna1Position,
        std::array<double, 3> outputAntenna2Position,
        double outputVelocitySpeed,
        double outputVelocityAngle
    );
}