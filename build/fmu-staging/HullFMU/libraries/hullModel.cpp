#include "hullModel.hpp"

namespace HullModel {
    // Variable for converting deg to rad for easier calculations
    const double PI = 3.14159265358979323846;

    // Constant of water used in calculating drag and buoyancy affecting the hull
    const double waterMassDensity = 1000.0; // [kg/m**3]

    // Variables for calculating drag on the hull
    const double dragCoefficientCube = 1.05;
    
    const double waterCurrentDragCoefficientAngular = 1.5; // This is an experimental coefficient tuned manually, the bigger the number, the more drag
    const double waterDragCoefficientRoll = 1.0; // Experimental Coefficient found by manual adjustment
    const double waterDragCoefficientPitch = 2.0; // Experimental Coefficient found by manual adjustment

    const double airDragCoefficientAngular = 0.1; // This is an experimental coefficient tuned manually, the bigger the number, the more drag
    const double airMassDensity = 1.2; // [kg/m**3]

    // Variables to calculate buoyancy and waves affecting the hull
    const double g = 9.81; // [m/s**2]
    


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
    ) {
        /*
         * Arguments:
         * double timeStep (The simulation step size, any REAL positive number) [s]
         * 
         * double inputThrusterBowForce (Any REAL negative or positive value) [N]
         * double InputThrusterBowAngle (Any REAL negative or positive value) [deg]
         * double inputThrusterPortForce (Any REAL negative or positive value) [N]
         * double inputThrusterPortAngle (Any REAL negative or positive value) [deg]
         * double inputThrusterStarboardForce (Any REAL negative or positive value) [N]
         * double inputThrusterStarboardAngle (Any REAL negative or positive value) [deg]
         * double inputWindSpeed (Any REAL negative or positive value) [m/s]
         * double inputWindAngle (Any REAL negative or positive value) [deg]
         * double inputWaterCurrentSpeed (Any REAL negative or positive value) [m/s]
         * double inputWaterCurrentAngle (Any REAL negative or positive value) [deg]
         * std::array<double, 4> inputWaveHeight {Front Water Height, Left Water Height, Center Water Height, Right Water Height, Back Water Height}
         * double Front Water Height (Any REAL negative or positive value) [m]
         * double Left Water Height (Any REAL negative or positive value) [m]
         * double Center Water Height (Any REAL negative or positive value) [m]
         * double Right Water Height (Any REAL negative or positive value) [m]
         * double Back Water Height (Any REAL negative or positive value) [m]
         * 
         * double parameterHullMass (Any REAL positive value) [kg]
         * double parameterHullLength (Any REAL POSITIVE value) [cm]
         * double parameterHullWidth (Any REAL POSITIVE value) [cm]
         * double parameterHullHeight (Any REAL POSITIVE value) [cm]
         * double parameterThrusterBowPositionX (Any REAL negative or positive value) [cm]
         * double parameterThrusterPortPositionX (Any REAL negative or positive value) [cm]
         * double parameterThrusterPortPositionY (Any REAL negative or positive value) [cm]
         * double parameterThrusterStarboardPositionX (Any REAL negative or positive value) [cm]
         * double parameterThrusterStarboardPositionY (Any REAL negative or positive value) [cm]
         * 
         * std::array<double, 6> OutputHullPosition {X Position, Y Position, Z Position, Roll Position, Pitch Position, Yaw Position}
         * double X Position (Any REAL negative or positive values) [m]
         * double Y Position (Any REAL negative or positive values) [m]
         * double Z Position (Any REAL negative or positive values) [m]
         * double Roll Position (Any REAL negative or positive values) [rad]
         * double Pitch Position (Any REAL negative or positive values) [rad]
         * double Yaw Position (Any REAL negative or positive values) [rad]
         * std::array<double, 6> OutputHullVelocity {X Linear Velocity, Y Linear Velocity, Z Linear Velocity, Roll Angular Velocity, Pitch Angular Velocity, Yaw Angular Velocity}
         * double X Linear Velocity (Any REAL negative or positive values) [m/s]
         * double Y Linear Velocity (Any REAL negative or positive values) [m/s]
         * double Z Linear Velocity (Any REAL negative or positive values) [m/s]
         * double Roll Angular Velocity (Any REAL negative or positive values) [rad/s]
         * double Pitch Angular Velocity (Any REAL negative or positive values) [rad/s]
         * double Yaw Angular Velocity (Any REAL negative or positive values) [rad/s]
         * 
         * 
         * 
         * Outputs:
         * std::array<double, 6> {positionX, positionY, positionZ, positionRoll, positionPitch, positionYaw}
         * double positionX (any REAL negative or positive value) [m]
         * double positionY (any REAL negative or positive value) [m]
         * double positionZ (any REAL negative or positive value) [m]
         * double positionRoll (any REAL negative or positive value) [rad]
         * double positionPitch (any REAL negative or positive value) [rad]
         * double positionYaw (any REAL negative or positive value) [rad]
         * 
         * std::array<double, 6> OutputHullVelocity {velocityX, velocityY, velocityZ, velocityRoll, velocityPitch, velocityYaw}
         * double velocityX (any REAL negative or positive value) [m/s]
         * double velocityY (any REAL negative or positive value) [m/s]
         * double velocityZ (any REAL negative or positive value) [m/s]
         * double positionRoll (any REAL negative or positive value) [rad/s]
         * double positionPitch (any REAL negative or positive value) [rad/s]
         * double positionYaw (any REAL negative or positive value) [rad/s]
         */



        // Preparations for simulation (START) --------------------------------------------------
        // Retrieve current positions
        double positionX = OutputHullPosition[0];
        double positionY = OutputHullPosition[1];
        double positionZ = OutputHullPosition[2];
        double positionRoll = OutputHullPosition[3];
        double positionPitch = OutputHullPosition[4];
        double positionYaw = OutputHullPosition[5];

        // Retrieve current velocities
        double velocityLinearX = OutputHullVelocity[0];
        double velocityLinearY = OutputHullVelocity[1];
        double velocityLinearZ = OutputHullVelocity[2];
        double velocityAngularRoll = OutputHullVelocity[3];
        double velocityAngularPitch = OutputHullVelocity[4];
        double velocityAngularYaw = OutputHullVelocity[5];

        // Calculations to help later on with angular acceleration
        // Calculate inertia g*(cm**2) then use "1e-7" to convert to kg*(m**2) as that is the conversion ration between the units
        // Use the simple ship inertia model, ie (1/12) * mass * (L**2 + W**2)
        double momentOfInertiaYaw = ((parameterHullMass/12) * (parameterHullLength * parameterHullLength + parameterHullWidth * parameterHullWidth)) * 1e-7;

        // Calculations to help later on find Drag on the Hull
        // Calculate area of the body in the top and bottom sides of ship approximately, not ideal but good enough
        double hullAreaXY = parameterHullLength * parameterHullWidth; // Approximate Square
        hullAreaXY /= 10000; // Convert from cm**2 to m**2
        // Calculate area of the body in the front and back sides of ship approximately, not ideal but good enough
        double hullAreaYZ = (parameterHullWidth * parameterHullHeight)/2; // Approximate Triangle
        hullAreaYZ /= 10000; // Convert from cm**2 to m**2
        // Calculate area of the body in the left and right sides of ship approximately, not ideal but good enough
        double hullAreaXZ = parameterHullLength * parameterHullHeight; // Approximate Square
        hullAreaXZ /= 10000; // Convert from cm**2 to m**2        
        // Preparations for simulation (STOP) --------------------------------------------------



        // Simulate Thruster Bow contribution (START) --------------------------------------------------
        // Convert input angles from degrees to radians
        double thrusterBowAngleRad = inputThrusterBowAngle * (PI / 180.0);
        
        // Calculate the local force components in the direction of the thruster
        double thrusterBowLocalForceX = inputThrusterBowForce * std::cos(thrusterBowAngleRad);
        double thrusterBowLocalForceY = inputThrusterBowForce * std::sin(thrusterBowAngleRad);

        // Convert local force components to global force components considering the boat's heading (yaw angle)
        double thrusterBowGlobalForceX = thrusterBowLocalForceX * std::cos(positionYaw) - thrusterBowLocalForceY * std::sin(positionYaw);
        double thrusterBowGlobalForceY = thrusterBowLocalForceX * std::sin(positionYaw) + thrusterBowLocalForceY * std::cos(positionYaw);

        // Calculate acceleration components in the global frame
        double thrusterBowAccelerationLinearX = thrusterBowGlobalForceX/parameterHullMass;
        double thrusterBowAccelerationLinearY = thrusterBowGlobalForceY/parameterHullMass;

        // Converting cm to m
        double thrusterBowPositionX = parameterThrusterBowPositionX/100.0;

        // Calculate the torque and angular acceleration (using moment of inertia)
        double thrusterBowTorque = inputThrusterBowForce * thrusterBowPositionX * std::sin(thrusterBowAngleRad);
        double thrusterBowAccelerationAngularYaw = thrusterBowTorque/momentOfInertiaYaw;
        // Simulate Thruster Bow contribution (STOP) --------------------------------------------------



        // Simulate Thruster Port contribution (START) --------------------------------------------------
        // Convert input angles from degrees to radians
        double thrusterPortAngleRad = inputThrusterPortAngle * (PI / 180.0);
        
        // Calculate the local force components in the direction of the thruster
        double thrusterPortLocalForceX = inputThrusterPortForce * std::cos(thrusterPortAngleRad);
        double thrusterPortLocalForceY = inputThrusterPortForce * std::sin(thrusterPortAngleRad);

        // Convert local force components to global force components considering the boat's heading (yaw angle)
        double thrusterPortGlobalForceX = thrusterPortLocalForceX * std::cos(positionYaw) - thrusterPortLocalForceY * std::sin(positionYaw);
        double thrusterPortGlobalForceY = thrusterPortLocalForceX * std::sin(positionYaw) + thrusterPortLocalForceY * std::cos(positionYaw);

        // Calculate acceleration components in the global frame
        double thrusterPortAccelerationLinearX = thrusterPortGlobalForceX/parameterHullMass;
        double thrusterPortAccelerationLinearY = thrusterPortGlobalForceY/parameterHullMass;

        // Converting cm to m
        double thrusterPortPositionX = parameterThrusterPortPositionX/100.0;
        double thrusterPortPositionY = parameterThrusterPortPositionY/100.0;

        // Calculate torque for both X and Y offsets 
        // This is because the thruster is at the back and not centered, meaning when giving full force with no angle, it will still expedience torque to a lesser extent
        double thrusterPortTorqueX = inputThrusterPortForce * thrusterPortPositionX * std::sin(thrusterPortAngleRad);
        double thrusterPortTorqueY = inputThrusterPortForce * thrusterPortPositionY * std::cos(thrusterPortAngleRad);
        double thrusterPortTorque = thrusterPortTorqueX + thrusterPortTorqueY;

        // Calculate angular acceleration (using moment of inertia)
        double thrusterPortAccelerationAngularYaw = thrusterPortTorque/momentOfInertiaYaw;
        /// Simulate Thruster Port contribution (STOP) --------------------------------------------------



        // Simulate Thruster Starboard contribution (START) --------------------------------------------------
        // Convert input angles from degrees to radians
        double thrusterStarboardAngleRad = inputThrusterStarboardAngle * (PI / 180.0);
        
        // Calculate the local force components in the direction of the thruster
        double thrusterStarboardLocalForceX = inputThrusterStarboardForce * std::cos(thrusterStarboardAngleRad);
        double thrusterStarboardLocalForceY = inputThrusterStarboardForce * std::sin(thrusterStarboardAngleRad);

        // Convert local force components to global force components considering the boat's heading (yaw angle)
        double thrusterStarboardGlobalForceX = thrusterStarboardLocalForceX * std::cos(positionYaw) - thrusterStarboardLocalForceY * std::sin(positionYaw);
        double thrusterStarboardGlobalForceY = thrusterStarboardLocalForceX * std::sin(positionYaw) + thrusterStarboardLocalForceY * std::cos(positionYaw);

        // Calculate acceleration components in the global frame
        double thrusterStarboardAccelerationLinearX = thrusterStarboardGlobalForceX/parameterHullMass;
        double thrusterStarboardAccelerationLinearY = thrusterStarboardGlobalForceY/parameterHullMass;

        // Converting cm to m
        double thrusterStarboardPositionX = parameterThrusterStarboardPositionX/100.0;
        double thrusterStarboardPositionY = parameterThrusterStarboardPositionY/100.0;

        // Calculate torque for both X and Y offsets
        // This is because the thruster is at the back and not centered, meaning when giving full force with no angle, it will still expedience torque to a lesser extent
        double thrusterStarboardTorqueX = inputThrusterStarboardForce * thrusterStarboardPositionX * std::sin(thrusterStarboardAngleRad);
        double thrusterStarboardTorqueY = inputThrusterStarboardForce * thrusterStarboardPositionY * std::cos(thrusterStarboardAngleRad);
        double thrusterStarboardTorque = thrusterStarboardTorqueX + thrusterStarboardTorqueY;

        // Calculate angular acceleration (using moment of inertia)
        double thrusterStarboardAccelerationAngularYaw = thrusterStarboardTorque/momentOfInertiaYaw;
        /// Simulate Thruster Starboard contribution (STOP) --------------------------------------------------r



        // Simulate Air Drag + Wind contribution (START) --------------------------------------------------
        // Convert wind angle from deg to rad
        double windAngleRad = inputWindAngle * (PI/180.0);

        // Calculate the relative flow velocity of air relative to the hull
        // Since wind counts as relative reference for velocity we can combine wind here to take effect as well
        double windRelativeVelocityX = inputWindSpeed * std::cos(windAngleRad) - velocityLinearX;
        double windRelativeVelocityY = inputWindSpeed * std::sin(windAngleRad) - velocityLinearY;

        // Calculate the effective areas the wind is affecting based on the yaw angle
        double windDragEffectiveHullAreaXDirection = hullAreaYZ * std::abs(std::cos(positionYaw)) + hullAreaXZ * std::abs(std::sin(positionYaw));
        double windDragEffectiveHullAreaYDirection = hullAreaXZ * std::abs(std::cos(positionYaw)) + hullAreaYZ * std::abs(std::sin(positionYaw));
        
        // Divide the effective area by 2
        // This is because only approximately 1/2 of the hull area is above water at any given time
        windDragEffectiveHullAreaXDirection /= 2;
        windDragEffectiveHullAreaYDirection /= 2;

        // Calculate drag forces
        // Drag formula in gas: 
        // F_drag = (1/2) * drag_coefficient * mass_density * Area * flow_velocity_relative_to_object**2
        double windDragForceX = 0.5 * dragCoefficientCube * airMassDensity * windDragEffectiveHullAreaXDirection * windRelativeVelocityX * std::abs(windRelativeVelocityX);
        double windDragForceY = 0.5 * dragCoefficientCube * airMassDensity * windDragEffectiveHullAreaYDirection * windRelativeVelocityY * std::abs(windRelativeVelocityY);

        // Convert drag forces to accelerations
        double windDragAccelerationLinearX = windDragForceX/parameterHullMass;
        double windDragAccelerationLinearY = windDragForceY/parameterHullMass;

        // Calculate the torque and angular acceleration due to drag
        double windDragAngularTorque = airDragCoefficientAngular * (-velocityAngularYaw);
        double windDragAccelerationAngularYaw = windDragAngularTorque/momentOfInertiaYaw;
        // Simulate Air Drag + Wind contribution (STOP) --------------------------------------------------



        // Simulate Water Drag + Water Current contribution (START) --------------------------------------------------
        // Convert water current angle from deg to rad
        double waterCurrentAngleRad = inputWaterCurrentAngle * (PI/180.0);

        // Calculate the relative flow velocity of water current relative to the hull
        // Since water current counts as relative reference for velocity we can combine water current here to take effect as well
        double waterCurrentRelativeVelocityX = inputWaterCurrentSpeed * std::cos(waterCurrentAngleRad) - velocityLinearX;
        double waterCurrentRelativeVelocityY = inputWaterCurrentSpeed * std::sin(waterCurrentAngleRad) - velocityLinearY;
        double waterCurrentRelativeVelocityZ = (-velocityLinearZ); // Special case as its only up and down velocity of hull

        // Calculate the effective areas the water current is affecting based on the yaw angle
        double waterDragEffectiveHullAreaXDirection = hullAreaYZ * std::abs(std::cos(positionYaw)) + hullAreaXZ * std::abs(std::sin(positionYaw));
        double waterDragEffectiveHullAreaYDirection = hullAreaXZ * std::abs(std::cos(positionYaw)) + hullAreaYZ * std::abs(std::sin(positionYaw));
        double waterDragEffectiveHullAreaZDirection = hullAreaXY; // Z axis Ie the Top and Bottom areas of hull are special as there is no real significant rotation to change the area (It is changing a bit in reality, but very little so we can neglect it)

        // Divide the effective area by 2
        // This is because only approximately 1/2 of the hull area is submerged under water at any given time
        waterDragEffectiveHullAreaXDirection /= 2;
        waterDragEffectiveHullAreaYDirection /= 2;

        // Calculate drag forces
        // Drag formula in fluids: 
        // F_drag = (1/2) * drag_coefficient * mass_density * Area * flow_velocity_relative_to_object**2
        double waterDragForceX = 0.5 * dragCoefficientCube * waterMassDensity * waterDragEffectiveHullAreaXDirection * waterCurrentRelativeVelocityX * std::abs(waterCurrentRelativeVelocityX);
        double waterDragForceY = 0.5 * dragCoefficientCube * waterMassDensity * waterDragEffectiveHullAreaYDirection * waterCurrentRelativeVelocityY * std::abs(waterCurrentRelativeVelocityY);
        double waterDragForceZ = 0.5 * dragCoefficientCube * waterMassDensity * waterDragEffectiveHullAreaZDirection * waterCurrentRelativeVelocityZ * std::abs(waterCurrentRelativeVelocityZ);

        // Convert drag forces to accelerations
        double waterDragAccelerationLinearX = waterDragForceX/parameterHullMass;
        double waterDragAccelerationLinearY = waterDragForceY/parameterHullMass;
        double waterDragAccelerationLinearZ = waterDragForceZ/parameterHullMass;

        // Calculate the torque and angular acceleration due to drag
        double waterDragAngularTorqueYaw = waterCurrentDragCoefficientAngular * (-velocityAngularYaw);
        double waterDragAccelerationAngularYaw = waterDragAngularTorqueYaw/momentOfInertiaYaw;

        // For Roll And Pitch calculating drag in simpler terms as its not necessary to have super high fidelity
        double waterDragAccelerationAngularRoll = waterDragCoefficientRoll * (-velocityAngularRoll);
        double waterDragAccelerationAngularPitch = waterDragCoefficientPitch * (-velocityAngularPitch);
        // Simulate Water Drag + Water Current contribution (STOP) --------------------------------------------------



        // Simulate Wave contribution (START) --------------------------------------------------
        // Get displacement volume/submerged part of the hull
        double waveHeightFront = inputWaveHeight[0];
        double waveHeightLeft = inputWaveHeight[1];
        double waveHeightCenter = inputWaveHeight[2];
        double waveHeightRight = inputWaveHeight[3];
        double waveHeightBack = inputWaveHeight[4];

        // Variable used later down to help calculate desired angle for hull compared to waves 
        double waveHeightDifference = 0.0;

        // Z-Axis Linear Acceleration ----------
        // Calculate buoyancy force ONLY if hull is submerged
        /*
         !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         !!! The real Algorithm, we only have the other for temporary testing, replace temporay algorithm with this when finalized
         !!!
         !!! double submergedVolumeCenter = waveHeightCenter * hullAreaXY; // Simplified Volume Formula, a bit jank but will suffice :P
         !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         */
        double submergedVolumeCenter = (waveHeightCenter - positionZ) * hullAreaXY; // Simplified Volume Formula, a bit jank but will suffice :P

        double buoyancyForce = 0.0;
        if (submergedVolumeCenter > 0) {
            // Buoyancy Force Formula:
            // F_buoyancy = fluid_mass_density * gravity_acceleration * volume_of_displaced_fluid
            buoyancyForce = waterMassDensity * g * submergedVolumeCenter;
        }
        
        // Calculate acceleration in Z direction
        double buoyancyAccelerationLinearZ = buoyancyForce/parameterHullMass;

        // Roll Angular Acceleration ----------
        // Calculate the difference in wave heights
        waveHeightDifference = waveHeightRight - waveHeightLeft;

        // Convert hull width from cm to m
        double hullWidthMeters = parameterHullWidth / 100.0;

        // Calculate the desired roll angle in radians
        double waveDesiredRoll = std::atan(waveHeightDifference/hullWidthMeters);

        // Compare the current roll with the desired roll
        // If real Roll is less than wave angle, set acceleration higher
        // If reverse, set acceleration lower
        double buoyancyAccelerationAngularRoll = 0.0;
        if (positionRoll < waveDesiredRoll) {
            buoyancyAccelerationAngularRoll = 1.0;
        } else {
            buoyancyAccelerationAngularRoll = -1.0;
        }

        // Pitch Angular Acceleration ----------
        // Calculate the difference in wave heights
        waveHeightDifference = waveHeightBack - waveHeightFront;

        // Convert hull length from cm to m
        double hullLengthMeters = parameterHullLength / 100.0;

        // Calculate the desired Pitch angle in radians
        double waveDesiredPitch = std::atan(waveHeightDifference/hullLengthMeters);

        // Compare the current Pitch with the desired Pitch
        // If real Pitch is less than wave angle, set acceleration higher
        // If reverse, set acceleration lower
        double buoyancyAccelerationAngularPitch = 0.0;
        if (positionPitch < waveDesiredPitch) {
            buoyancyAccelerationAngularPitch = 1.0;
        } else {
            buoyancyAccelerationAngularPitch = -1.0;
        }
        // Simulate Wave contribution (START) --------------------------------------------------



        // Add up all contributions (START) --------------------------------------------------
        // Add upp all the acceleration contribution from different sources
        double accelerationLinearX = thrusterBowAccelerationLinearX + thrusterPortAccelerationLinearX + thrusterStarboardAccelerationLinearX // Thruster Contributions
                                    + windDragAccelerationLinearX + waterDragAccelerationLinearX; // Environmental Forces Contribution
        double accelerationLinearY = thrusterBowAccelerationLinearY + thrusterPortAccelerationLinearY + thrusterStarboardAccelerationLinearY // Thruster Contributions
                                    + windDragAccelerationLinearY + waterDragAccelerationLinearY; // Environmental Forces Contribution
        double accelerationLinearZ = waterDragAccelerationLinearZ + buoyancyAccelerationLinearZ - g; // Environmental Forces Contribution
        double accelerationAngularRoll = waterDragAccelerationAngularRoll + buoyancyAccelerationAngularRoll; // Environmental Forces Contribution
        double accelerationAngularPitch = waterDragAccelerationAngularPitch + buoyancyAccelerationAngularPitch; // Environmental Forces Contribution
        double accelerationAngularYaw = thrusterBowAccelerationAngularYaw + thrusterPortAccelerationAngularYaw + thrusterStarboardAccelerationAngularYaw // Thruster Contributions
                                        + windDragAccelerationAngularYaw + waterDragAccelerationAngularYaw; // Environmental Forces Contribution

        // Update velocities based on the sum of all accelerations
        velocityLinearX += accelerationLinearX * timeStep;
        velocityLinearY += accelerationLinearY * timeStep;
        velocityLinearZ += accelerationLinearZ * timeStep;
        velocityAngularRoll += accelerationAngularRoll * timeStep;
        velocityAngularPitch += accelerationAngularPitch * timeStep;
        velocityAngularYaw += accelerationAngularYaw * timeStep;

        // Update positions based on velocities
        positionX += velocityLinearX * timeStep;
        positionY += velocityLinearY * timeStep;
        positionZ += velocityLinearZ * timeStep;
        positionRoll += velocityAngularRoll * timeStep;
        positionPitch += velocityAngularPitch * timeStep;
        positionYaw += velocityAngularYaw * timeStep;
        // Add up all contributions (STOP) --------------------------------------------------

        

        // Give out the simulated values
        return {
            positionX,
            positionY,
            positionZ,
            positionRoll,
            positionPitch,
            positionYaw,

            velocityLinearX,
            velocityLinearY,
            velocityLinearZ,
            velocityAngularRoll,
            velocityAngularPitch,
            velocityAngularYaw
        };
    }
}