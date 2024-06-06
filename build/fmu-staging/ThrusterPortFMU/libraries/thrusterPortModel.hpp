#include <array>
#include <iostream>
#include <random>    // for randomness
#include <algorithm> // for std::clamp

namespace ThrusterPortModel {
    std::array<double, 2> simulate(double timeStep, double inputForce, double inputAngle, double parameterThrusterEfficiency, double currentForce, double currentAngle);
}