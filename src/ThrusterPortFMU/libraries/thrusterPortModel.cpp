#include "thrusterPortModel.hpp"

namespace ThrusterPortModel {
    // Variable for simulating time delay as a ramp-up
    // The bigger the value, the more time it takes for thrusters to give out full force or agle
    double forceRampUpDuration = 0.1;
    double angleRampUpDuration = 0.08;

    // Variable for simulating noise
    double forceNoiseMean = 0.0; // Newtons
    double forceNoiseStandardDeviation = 10.0; // Newtons
    double angleNoiseMean = 0.0; // Degrees
    double angleNoiseStandardDeviation = 0.01; // Degrees



    std::array<double, 2> simulate(double timeStep, double inputForce, double inputAngle, double parameterThrusterEfficiency, double currentForce, double currentAngle) {
        /*
         * Arguments:
         * double timeStep (The simulation step size, any REAL positive number) [Seconds]
         * double inputForce (Any REAL negative or positive value) [Newtons]
         * double inputAngle ((-90.0) to 90.0) [Degrees]
         * double parameterThrusterEfficiency (0.0 to 1.0) [Percent]
         * double currentForce (The force thrusters are giving out now, any REAL negative or positive number) [Newtons]
         * 
         * Outputs:
         * std::array<double, 2> {outputForce, outputAngle}
         * double outputForce (any REAL negative or positive value) [Newtons]
         * double outputAngle ((-90.0) to 90.0) [Degrees]
         */



        // Make a random number generator for simulating noise in the system
        std::random_device rd;
        std::mt19937 gen(rd());



        // Simulate output force (START) --------------------------------------------------
        double outputForce = currentForce;

        // Check that the thrust eficiency doesent go over 1.0 OR <0.0 O_O
        if (parameterThrusterEfficiency < 0.0) {
            std::cerr << "Warning: parameterThrusterEfficiency below minimum. Clamping to 0.0." << std::endl;
            parameterThrusterEfficiency = 0.0;
        } else if (parameterThrusterEfficiency > 1.0) {
            std::cerr << "Warning: parameterThrusterEfficiency above maximum. Clamping to 1.0." << std::endl;
            parameterThrusterEfficiency = 1.0;
        }

        // There is a time delay in the input and output in real world
        // Since the desired force canot be always perfect and thrusters have a not perfect 100% efficiency, some of the thrust will be disapated
        double desiredForce = inputForce * parameterThrusterEfficiency;
        // Set up variables for simulating time delay
        double forceIncrement = (std::abs(desiredForce - currentForce) / forceRampUpDuration) * timeStep;
        // Simulate output force with ramp-up to imitate time delay
        // If current force less than what we want, the output force must be higher, ramp up
        // If we have to high force, ramp down
        if (currentForce < desiredForce) {
            outputForce = std::min(currentForce + forceIncrement, desiredForce);
        } else {
            outputForce = std::max(currentForce - forceIncrement, desiredForce);
        }

        // There is also a part of randomness that must be taken into account
        // However if the inputForce is set to 0, we do NOT add noise
        // As the thruster has stopped giving actively force, and this is less prone to random fluctuations in force
        if (inputForce != 0.0) {
            std::normal_distribution<> forceDist(forceNoiseMean, forceNoiseStandardDeviation);
            outputForce += forceDist(gen);
        }
        // Simulate output force (STOP) --------------------------------------------------
        


        // Simulate output angle (START) --------------------------------------------------
        double outputAngle = currentAngle;

        // There is a time delay in the input and output in real world
        // Set up variables for simulating time delay
        double angleIncrement = (std::abs(inputAngle - currentAngle) / angleRampUpDuration) * timeStep;
        // Simulate output angle with ramp-up to imitate time delay
        // If current angle less than what we want, the output angle must be higher, ramp up
        // If we have to high angle, ramp down
        if (currentAngle < inputAngle) {
            outputAngle = std::min(currentAngle + angleIncrement, inputAngle);
        } else {
            outputAngle = std::max(currentAngle - angleIncrement, inputAngle);
        }

        // Angle in real world can't always be perfect, thus there has to be a bit of slow acting noise
        std::normal_distribution<> angleDist(angleNoiseMean, angleNoiseStandardDeviation);
        outputAngle += angleDist(gen);
        // Simulate output angle (STOP) --------------------------------------------------



        // Give out the simulated values
        return {outputForce, outputAngle};
    }
}