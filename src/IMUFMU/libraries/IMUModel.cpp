#include "IMUModel.hpp"

namespace IMUModel {
    // Variable for tracking time
    double timeNow = 0.0;

    // Variable for tracking velocity
    double velocityLinearXOld = 0.0;
    double velocityLinearYOld = 0.0;
    double velocityLinearZOld = 0.0;

    // Constant variables
    const double g = 9.81;



    std::array<double, 6> simulate(
        double timeStamp,

        std::array<double, 6> inputHullPosition,
        std::array<double, 6> inputHullVelocity,

        double parameterTimeBetweenData,
        double parameterAccuracyAccelerometer,
        double parameterAccuracyGyroscope,

        std::array<double, 3> outputAccelerationLinear,
        std::array<double, 3> outputVelocityAngular
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
         * double parameterTimeBetweenData (any REAL positive value) [ms]
         * double parameterAccuracyAccelerometer (any REAL positive value) [mm/(s**2) (milli meter per square second)]
         * double parameterAccuracyGyroscope (any REAL positive value) [mrad/s (milli rad per second)]
         * 
         * std::array<double, 3> outputAccelerationLinear {X Acceleration, Y Acceleration, Z Acceleration}
         * double X Acceleration (any REAL negative or positive value) [m/s**2]
         * double Y Acceleration (any REAL negative or positive value) [m/s**2]
         * double Z Acceleration (any REAL negative or positive value) [m/s**2]
         * std::array<double, 3> outputVelocityAngular {Roll Velocity, Pitch Velocity, Yaw Velocity}
         * double Roll Velocity (any REAL negative or positive value) [rad/s]
         * double Pitch Velocity (any REAL negative or positive value) [rad/s]
         * double Yaw Velocity (any REAL negative or positive value) [rad/s]
         * 
         * 
         * 
         * Outputs:
         * std::array<double, 6> {antenna1PositionX, antenna1PositionY, antenna1PositionZ, antenna2PositionX, antenna2PositionY, antenna2PositionZ, velocitySpeed, velocityAngle}
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
            // Update time
            timeNow += timeStamp;

            // Update old velocities
            velocityLinearXOld = inputHullVelocity[0];
            velocityLinearYOld = inputHullVelocity[1];
            velocityLinearZOld = inputHullVelocity[2];

            // Return the old output as time for new data hasn't come yet
            return {
                outputAccelerationLinear[0],
                outputAccelerationLinear[1],
                outputAccelerationLinear[2],
                outputVelocityAngular[0],
                outputVelocityAngular[1],
                outputVelocityAngular[2]
            };
        } 
        timeNow = 0;



        // Generate Noise Algorithm (START) --------------------------------------------------
        // Create random number generator and distribution for noise
        std::random_device rd;
        std::mt19937 gen(rd());

        // Create white noise for Acceleration Linear
        double accuracyAccelerometer = std::abs(parameterAccuracyAccelerometer)/1000.0; // Convert from mm/(s**2) (milli meter per square second) to m/s**2
        std::uniform_real_distribution<> whiteNoiseAccelerationLinear((-accuracyAccelerometer), accuracyAccelerometer);

        // Create white noise for Velocity Angular
        double accuracyGyroscope = std::abs(parameterAccuracyGyroscope)/1000.0; // Convert from mrad/s (milli rad per second) to rad/s
        std::uniform_real_distribution<> whiteNoiseVelocityAngular((-accuracyGyroscope), accuracyGyroscope);
        // Generate Noise Algorithm (STOP) --------------------------------------------------



        // Simulate IMU Linear Accelerations Data (START) --------------------------------------------------
        // Calculate the desired Acceleration
        double accelerationLinearXDesired = (inputHullVelocity[0] - velocityLinearXOld)/timeStamp;
        double accelerationLinearYDesired = (inputHullVelocity[1] - velocityLinearYOld)/timeStamp;
        double accelerationLinearZDesired = (inputHullVelocity[2] - velocityLinearZOld)/timeStamp;
        
        // Update old velocities
        velocityLinearXOld = inputHullVelocity[0];
        velocityLinearYOld = inputHullVelocity[1];
        velocityLinearZOld = inputHullVelocity[2];

        // Transform accelerations from body frame to inertial frame considering roll and pitch
        // This transformation is necessary because the IMU's axes are aligned with the vehicle's body frame.
        // When the vehicle tilts or rolls, the IMU's x, y, and z axes no longer align with the inertial frame axes.
        // By considering the roll and pitch angles, we can correctly compute the accelerations in the inertial frame.
        double cosRoll = std::cos(inputHullPosition[3]);
        double sinRoll = std::sin(inputHullPosition[3]);
        double cosPitch = std::cos(inputHullPosition[4]);
        double sinPitch = std::sin(inputHullPosition[4]);

        double accelerationLinearX = cosPitch * accelerationLinearXDesired + sinPitch * sinRoll * accelerationLinearYDesired + sinPitch * cosRoll * (accelerationLinearZDesired - g);
        double accelerationLinearY = cosRoll * accelerationLinearYDesired - sinRoll * (accelerationLinearZDesired - g);
        double accelerationLinearZ = -sinPitch * accelerationLinearXDesired + cosPitch * sinRoll * accelerationLinearYDesired + cosPitch * cosRoll * (accelerationLinearZDesired - g);

        // Add noise to the IMU measurements to make it more realistic
        accelerationLinearX += whiteNoiseAccelerationLinear(gen);
        accelerationLinearY += whiteNoiseAccelerationLinear(gen);
        accelerationLinearZ += whiteNoiseAccelerationLinear(gen);
        // Simulate IMU Linear Accelerations Data (STOP) --------------------------------------------------



        // Simulate IMU Angular Velocity Data (START) --------------------------------------------------
        // Calculate the desired Acceleration
        double velocityAngularRollDesired = inputHullVelocity[3];
        double velocityAngularPitchDesired = inputHullVelocity[4];
        double velocityAngularYawDesired = inputHullVelocity[5];

        // Simulate IMU Angular Velocity
        double velocityAngularRoll = velocityAngularRollDesired;
        double velocityAngularPitch = velocityAngularPitchDesired;
        double velocityAngularYaw = velocityAngularYawDesired;

        // Add noise to the IMU measurements to make it more realistic
        velocityAngularRoll += whiteNoiseVelocityAngular(gen);
        velocityAngularPitch += whiteNoiseVelocityAngular(gen);
        velocityAngularYaw += whiteNoiseVelocityAngular(gen);
        // Simulate IMU Angular Velocity Data (STOP) --------------------------------------------------

        

        // Return the simulated results out
        return {
            accelerationLinearX,
            accelerationLinearY,
            accelerationLinearZ,
            velocityAngularRoll,
            velocityAngularPitch,
            velocityAngularYaw
        };
    }
}