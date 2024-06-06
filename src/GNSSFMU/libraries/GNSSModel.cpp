#include "GNSSModel.hpp"

namespace GNSSModel {
    // Variable for tracking time
    double timeNow = 0.0;

    // Variables for noise simulation
    double standardDeviationToWhiteNoiseCoefficient = 0.05; // (1.0 - 0.0) How noisy white noise is compared to standard deviation. The bigger the number, the higher white noise is
    double noiseSuppressionPositionZCoefficient = 10.0; // (1.0 ->) Z-axis need less noise, this coefficient suppress noise
    double whiteNoiseDistributionVelocitySpeed = 0.1; // [m/s]
    double whiteNoiseDistributionVelocityAngle = 0.01; // [rad]


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
    ) {
        /*
         * Arguments:
         * double timeStep (The simulation step size, any REAL positive number) [s]
         * 
         * std::array<double, 6> inputHullPosition {X, Y, Z, Roll, Pitch, Yaw}
         * double X (any REAL negative or positive value) [m]
         * double Y (any REAL negative or positive value) [m]
         * double Z (any REAL negative or positive value) [m]
         * double Roll (any REAL negative or positive value) [rad]
         * double Pitch (any REAL negative or positive value) [rad]
         * double Yaw (any REAL negative or positive value) [rad]
         * std::array<double, 6> inputHullVelocity {X, Y, Z, Roll, Pitch, Yaw}
         * double X (any REAL negative or positive value) [m/s]
         * double Y (any REAL negative or positive value) [m/s]
         * double Z (any REAL negative or positive value) [m/s]
         * double Roll (any REAL negative or positive value) [rad/s]
         * double Pitch (any REAL negative or positive value) [rad/s]
         * double Yaw (any REAL negative or positive value) [rad/s]
         * 
         * double parameterAntennaDistance (any REAL negative or positive value) [cm]
         * double parameterTimeBetweenData (any REAL positive value) [ms]
         * double parameterStandardDeviation (any REAL negative or positive value) [cm]
         * 
         * std::array<double, 3> outputAntenna1Position {X, Y, Z}
         * double X (any REAL negative or positive value) [m]
         * double Y (any REAL negative or positive value) [m]
         * double Z (any REAL negative or positive value) [m]
         * std::array<double, 3> outputAntenna2Position {X, Y, Z}
         * double X (any REAL negative or positive value) [m]
         * double Y (any REAL negative or positive value) [m]
         * double Z (any REAL negative or positive value) [m]
         * double outputVelocitySpeed (any REAL negative or positive value) [m/s]
         * double outputVelocityAngle (any REAL negative or positive value) [rad]
         * 
         * 
         * 
         * Outputs:
         * std::array<double, 8> {antenna1PositionX, antenna1PositionY, antenna1PositionZ, antenna2PositionX, antenna2PositionY, antenna2PositionZ, velocitySpeed, velocityAngle}
         * double antenna1PositionX (any REAL negative or positive value) [m]
         * double antenna1PositionY (any REAL negative or positive value) [m]
         * double antenna1PositionZ (any REAL negative or positive value) [m]
         * double antenna2PositionX (any REAL negative or positive value) [m]
         * double antenna2PositionY (any REAL negative or positive value) [m]
         * double antenna2PositionZ (any REAL negative or positive value) [m]
         * double velocitySpeed (any REAL negative or positive value) [m/s]
         * double velocityAngle (any REAL negative or positive value) [rad]
         */



        // Before doing anything we must check time
        // If the time has not passed yet until we give the next position data
        // Then we return the old output and be done
        // However if the time HAS passed
        // Then we continue to the actual simulation code and calculations
        double timeBetweenData = parameterTimeBetweenData/1000.0; // Convert ms to s
        if (timeNow < timeBetweenData) {
            timeNow += timeStamp;

            return {
                outputAntenna1Position[0],
                outputAntenna1Position[1],
                outputAntenna1Position[2],
                outputAntenna2Position[0],
                outputAntenna2Position[1],
                outputAntenna2Position[2],
                outputVelocitySpeed,
                outputVelocityAngle
            };
        } 
        timeNow = 0;



        // Generate Noise Algorithm (START) --------------------------------------------------
        // Create random number generator and distribution for noise
        std::random_device rd;
        std::mt19937 gen(rd());

        // Convert Standard deviation from cm to m
        double standardDeviation = parameterStandardDeviation/100.0;

        // Create normal distribution
        std::normal_distribution<> normalDist(0, standardDeviation);
        
        // Create white noise
        double whiteNoiseDistributionPosition = std::abs(standardDeviationToWhiteNoiseCoefficient * standardDeviation);
        std::uniform_real_distribution<> whiteNoisePosition((-whiteNoiseDistributionPosition), whiteNoiseDistributionPosition);
        // Generate Noise Algorithm (STOP) --------------------------------------------------



        // Simulate Antenna 1 position (START) --------------------------------------------------
        double antenna1DesiredPositionX = inputHullPosition[0];
        double antenna1DesiredPositionY = inputHullPosition[1];
        double antenna1DesiredPositionZ = inputHullPosition[2];

        double antenna1PositionX = antenna1DesiredPositionX + normalDist(gen) + whiteNoisePosition(gen);
        double antenna1PositionY = antenna1DesiredPositionY + normalDist(gen) + whiteNoisePosition(gen);

        // Z-Axis is less prone to noise and is more accurate because of how GNSS works and the fact that sea has a more uniform shape that ground terrain
        double antenna1PositionZ = antenna1DesiredPositionZ + (normalDist(gen) + whiteNoisePosition(gen))/noiseSuppressionPositionZCoefficient;
        // Simulate Antenna 1 position (STOP) --------------------------------------------------



        // Simulate Antenna 2 position (START) --------------------------------------------------
        // Convert parameter from cm to m
        double antennaDistance = parameterAntennaDistance/100.0;

        double antenna2DesiredPositionX = inputHullPosition[0] + antennaDistance * std::cos(inputHullPosition[5]);
        double antenna2DesiredPositionY = inputHullPosition[1] + antennaDistance * std::sin(inputHullPosition[5]);
        double antenna2DesiredPositionZ = inputHullPosition[2];

        double antenna2PositionX = antenna2DesiredPositionX + normalDist(gen) + whiteNoisePosition(gen);
        double antenna2PositionY = antenna2DesiredPositionY + normalDist(gen) + whiteNoisePosition(gen);

        // Z-Axis is less prone to noise and is more accurate because of how GNSS works and the fact that sea has a more uniform shape than ground terrain
        double antenna2PositionZ = antenna2DesiredPositionZ + (normalDist(gen) + whiteNoisePosition(gen)) / noiseSuppressionPositionZCoefficient;
        // Simulate Antenna 2 position (STOP) --------------------------------------------------



        // Simulate GNSS Velocity Data (START) --------------------------------------------------
        // Absolute speed
        double velocitySpeedDesired = std::sqrt((inputHullVelocity[0] * inputHullVelocity[0]) + (inputHullVelocity[1] * inputHullVelocity[1]));
        std::uniform_real_distribution<> whiteNoiseVelocitySpeed((-whiteNoiseDistributionVelocitySpeed), whiteNoiseDistributionVelocitySpeed);
        double velocitySpeed = velocitySpeedDesired + whiteNoiseVelocitySpeed(gen);

        // Absolute angle
        double velocityAngleDesired = inputHullPosition[5];
        std::uniform_real_distribution<> whiteNoiseVelocityAngle((-whiteNoiseDistributionVelocityAngle), whiteNoiseDistributionVelocityAngle);
        double velocityAngle = velocityAngleDesired + whiteNoiseVelocityAngle(gen);
        // Simulate GNSS Velocity Data (START) --------------------------------------------------

        

        // Return the simulated results out
        return {
            antenna1PositionX,
            antenna1PositionY,
            antenna1PositionZ,
            antenna2PositionX,
            antenna2PositionY,
            antenna2PositionZ,
            velocitySpeed,
            velocityAngle
        };
    }
}