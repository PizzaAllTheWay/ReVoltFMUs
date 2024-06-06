#include <array>
#include <iostream>
#include <random>    // for randomness

namespace IMUModel {
    std::array<double, 6> simulate(
        double timeStamp,

        std::array<double, 6> inputHullPosition,
        std::array<double, 6> inputHullVelocity,

        double parameterTimeBetweenData,
        double parameterAccuracyAccelerometer,
        double parameterAccuracyGyroscope,

        std::array<double, 3> outputAccelerationLinear,
        std::array<double, 3> outputVelocityAngular
    );
}