#include "fmu-uuid.h"
#include <cstring> // For strcmp
#include <array>
#include <cppfmu_cs.hpp>

// Custom made models
#include "../libraries/hullModel.hpp"



class HullFMU : public cppfmu::SlaveInstance
{
public:
    // Constructor must be present
    HullFMU()
    {
        HullFMU::Reset();
    }

    // Reset function is called at the beginning of each simulation
    void Reset() override
    {
        InputThrusterBowForce = 0.0; // [N]
        InputThrusterBowAngle = 0.0; // [deg]
        InputThrusterPortForce = 0.0; // [N]
        InputThrusterPortAngle = 0.0; // [deg]
        InputThrusterStarboardForce = 0.0; // [N]
        InputThrusterStarboardAngle = 0.0; // [deg]
        InputWindSpeed = 0.0; // [m/s]
        InputWindAngle = 0.0; // [deg]
        InputWaterCurrentSpeed = 0.0; // [m/s]
        InputWaterCurrentAngle = 0.0; // [deg]
        InputWaveHeight = {
            0.0, // Front [m]
            0.0, // Left [m]
            0.0, // Center [m]
            0.0, // Right [m]
            0.0 // Back [m]
        };

        ParameterHullMass = 1000; // [kg]
        ParameterHullLength = 300; // [cm]
        ParameterHullWidth = 80; // [cm]
        ParameterHullHeight = 70; // [cm]
        ParameterThrusterBowPositionX = 130; // [cm]
        ParameterThrusterPortPositionX = -130; // [cm]
        ParameterThrusterPortPositionY = -20; // [cm]
        ParameterThrusterStarboardPositionX = -130; // [cm]
        ParameterThrusterStarboardPositionY = 20; // [cm]
        
        // Don't need to define it outputs, however is nice to set a specific value for start just in case
        OutputHullPosition = {
            0.0, // X Position [m]
            0.0, // Y Position [m]
            0.0, // Z Position [m]
            0.0, // Roll Position [rad]
            0.0, // Pitch Position [rad]
            0.0 // Yaw Position [rad]
        };
        OutputHullVelocity = {
            0.0, // X Linear Velocity [m/s]
            0.0, // Y Linear Velocity [m/s]
            0.0, // Z Linear Velocity [m/s]
            0.0, // Roll Angular Velocity [rad/s]
            0.0, // Pitch Angular Velocity [rad/s]
            0.0 // Yaw Angular Velocity [rad/s]
        };
    }

    // When the simulations tells FMU to set FMUVaraibles to specific values
    // This function loops through all the Real Varaibles in XML file, BUT if it finds differemnt data structure like Integer, then it will call function SetInteger() instead, so we must define this funmction to handle Integer variables later
    // NOTE: We put in OutputHullPosition cases here because in case there is some outside STC simulator stuff that wants to completely overwrite our Outputs they have the ability to do so, not necesarry, but nice to have
    void SetReal(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, const cppfmu::FMIReal values[]) override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            cppfmu::FMIReal value = values[index];
            switch(valueReference) {
                // Value reference coresponds to value references specified in XML
                case 0:
                    InputThrusterBowForce = value;
                    break;
                case 1:
                    InputThrusterBowAngle = value;
                    break;
                case 2:
                    InputThrusterPortForce = value;
                    break;
                case 3:
                    InputThrusterPortAngle = value;
                    break;
                case 4:
                    InputThrusterStarboardForce = value;
                    break;
                case 5:
                    InputThrusterStarboardAngle = value;
                    break;
                case 6:
                    InputWindSpeed = value;
                    break;
                case 7:
                    InputWindAngle = value;
                    break;
                case 8:
                    InputWaterCurrentSpeed = value;
                    break;
                case 9:
                    InputWaterCurrentAngle = value;
                    break;
                case 10:
                    InputWaveHeight[0] = value;
                    break;
                case 11:
                    InputWaveHeight[1] = value;
                    break;
                case 12:
                    InputWaveHeight[2] = value;
                    break;
                case 13:
                    InputWaveHeight[3] = value;
                    break;
                case 14:
                    InputWaveHeight[4] = value;
                    break;

                case 24:
                    OutputHullPosition[0] = value;
                    break;
                case 25:
                    OutputHullPosition[1] = value;
                    break;
                case 26:
                    OutputHullPosition[2] = value;
                    break;
                case 27:
                    OutputHullPosition[3] = value;
                    break;
                case 28:
                    OutputHullPosition[4] = value;
                    break;
                case 29:
                    OutputHullPosition[5] = value;
                    break;

                case 30:
                    OutputHullVelocity[0] = value;
                    break;
                case 31:
                    OutputHullVelocity[1] = value;
                    break;
                case 32:
                    OutputHullVelocity[2] = value;
                    break;
                case 33:
                    OutputHullVelocity[3] = value;
                    break;
                case 34:
                    OutputHullVelocity[4] = value;
                    break;
                case 35:
                    OutputHullVelocity[5] = value;
                    break;
            }
        }
    }

    void SetInteger(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, const cppfmu::FMIInteger values[]) override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            // In this FMU we focus on Parameters, which can only be Integer data types, so all of these switches will just get all the Parameter values from FMU Parameter Values
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            cppfmu::FMIInteger value = values[index];
            switch(valueReference) {
                case 15:
                    ParameterHullMass = value;
                    break;
                case 16:
                    ParameterHullLength = value;
                    break;
                case 17:
                    ParameterHullWidth = value;
                    break;
                case 18:
                    ParameterHullHeight = value;
                    break;
                case 19:
                    ParameterThrusterBowPositionX = value;
                    break;
                case 20:
                    ParameterThrusterPortPositionX = value;
                    break;
                case 21:
                    ParameterThrusterPortPositionY = value;
                    break;
                case 22:
                    ParameterThrusterStarboardPositionX = value;
                    break;
                case 23:
                    ParameterThrusterStarboardPositionY = value;
                    break;
            }
        }
    }

    // These are functions for giving Outputs from the FMU
    // GetReal will give outputs to Real values, whilest getIntegers() wil give output to Integer variables
    // NOTE: We not only set the Outputs here but also Inputs and Parameter, this is not Necesarry, BUT without it, the FMU output would not see what kind of inpout we gave into the FMU, which would be tricky to debug or plot data in case of something (no bueno)
    void GetReal(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, cppfmu::FMIReal values[]) const override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            switch(valueReference) {
                // Value reference coresponds to value references specified in XML
                case 0:
                    values[index] = InputThrusterBowForce;
                    break;
                case 1:
                    values[index] = InputThrusterBowAngle;
                    break;
                case 2:
                    values[index] = InputThrusterPortForce;
                    break;
                case 3:
                    values[index] = InputThrusterPortAngle;
                    break;
                case 4:
                    values[index] = InputThrusterStarboardForce;
                    break;
                case 5:
                    values[index] = InputThrusterStarboardAngle;
                    break;
                case 6:
                    values[index] = InputWindSpeed;
                    break;
                case 7:
                    values[index] = InputWindAngle;
                    break;
                case 8:
                    values[index] = InputWaterCurrentSpeed;
                    break;
                case 9:
                    values[index] = InputWaterCurrentAngle;
                    break;
                case 10:
                    values[index] = InputWaveHeight[0];
                    break;
                case 11:
                    values[index] = InputWaveHeight[1];
                    break;
                case 12:
                    values[index] = InputWaveHeight[2];
                    break;
                case 13:
                    values[index] = InputWaveHeight[3];
                    break;
                case 14:
                    values[index] = InputWaveHeight[4];
                    break;

                case 24:
                    values[index] = OutputHullPosition[0];
                    break;
                case 25:
                    values[index] = OutputHullPosition[1];
                    break;
                case 26:
                    values[index] = OutputHullPosition[2];
                    break;
                case 27:
                    values[index] = OutputHullPosition[3];
                    break;
                case 28:
                    values[index] = OutputHullPosition[4];
                    break;
                case 29:
                    values[index] = OutputHullPosition[5];
                    break;

                case 30:
                    values[index] = OutputHullVelocity[0];
                    break;
                case 31:
                    values[index] = OutputHullVelocity[1];
                    break;
                case 32:
                    values[index] = OutputHullVelocity[2];
                    break;
                case 33:
                    values[index] = OutputHullVelocity[3];
                    break;
                case 34:
                    values[index] = OutputHullVelocity[4];
                    break;
                case 35:
                    values[index] = OutputHullVelocity[5];
                    break;
            }
        }
    }

    void GetInteger(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, cppfmu::FMIInteger values[]) const override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            switch(valueReference) {
                // In this FMU we focus on Parameters, which can only be Integer data types, so all of these switches will just get all the Parameter values from external sources, in case someone under the simulation decided to charne the FMU Parameter values
                case 15:
                    values[index] = ParameterHullMass;
                    break;
                case 16:
                    values[index] = ParameterHullLength;
                    break;
                case 17:
                    values[index] = ParameterHullWidth;
                    break;
                case 18:
                    values[index] = ParameterHullHeight;
                    break;
                case 19:
                    values[index] = ParameterThrusterBowPositionX;
                    break;
                case 20:
                    values[index] = ParameterThrusterPortPositionX;
                    break;
                case 21:
                    values[index] = ParameterThrusterPortPositionY;
                    break;
                case 22:
                    values[index] = ParameterThrusterStarboardPositionX;
                    break;
                case 23:
                    values[index] = ParameterThrusterStarboardPositionY;
                    break;
            }
        }
    }

    // Begin simulation here
    bool DoStep(cppfmu::FMIReal startOfTimeStep, cppfmu::FMIReal timeStep, cppfmu::FMIBoolean isThisANewTimeStep,
        cppfmu::FMIReal& endOfTimeStep) override
    {
        (void)startOfTimeStep; // Mark as unused
        (void)isThisANewTimeStep; // Mark as unused
        (void)endOfTimeStep; // Mark as unused

        // Simulation (START) --------------------------------------------------
        // Simulation itself
        std::array<double, 12> results = HullModel::simulate(
            timeStep,

            InputThrusterBowForce,
            InputThrusterBowAngle,
            InputThrusterPortForce,
            InputThrusterPortAngle,
            InputThrusterStarboardForce,
            InputThrusterStarboardAngle,
            InputWindSpeed,
            InputWindAngle,
            InputWaterCurrentSpeed,
            InputWaterCurrentAngle,
            InputWaveHeight,

            static_cast<cppfmu::FMIReal>(ParameterHullMass),
            static_cast<cppfmu::FMIReal>(ParameterHullLength),
            static_cast<cppfmu::FMIReal>(ParameterHullWidth),
            static_cast<cppfmu::FMIReal>(ParameterHullHeight),
            static_cast<cppfmu::FMIReal>(ParameterThrusterBowPositionX),
            static_cast<cppfmu::FMIReal>(ParameterThrusterPortPositionX),
            static_cast<cppfmu::FMIReal>(ParameterThrusterPortPositionY),
            static_cast<cppfmu::FMIReal>(ParameterThrusterStarboardPositionX),
            static_cast<cppfmu::FMIReal>(ParameterThrusterStarboardPositionY),

            OutputHullPosition,
            OutputHullVelocity
        );

        // Saving simulated results to corresponding outputs
        // First 6 elemnts in the array are position values
        // The last 6 elements in the array are velocity values
        for (int i = 0; i < 6; i++) {
            OutputHullPosition[i] = results[i];
            OutputHullVelocity[i] = results[i + 6];
        }
        // Simulation (STOP) --------------------------------------------------

        // Specify we are done with the simulation step
        return true;
    }

private:
    // Variables from .xml file that binds FMU code with C++ code
    // Inputs from FMU
    cppfmu::FMIReal InputThrusterBowForce;
    cppfmu::FMIReal InputThrusterBowAngle;
    cppfmu::FMIReal InputThrusterPortForce;
    cppfmu::FMIReal InputThrusterPortAngle;
    cppfmu::FMIReal InputThrusterStarboardForce;
    cppfmu::FMIReal InputThrusterStarboardAngle;
    cppfmu::FMIReal InputWindSpeed;
    cppfmu::FMIReal InputWindAngle;
    cppfmu::FMIReal InputWaterCurrentSpeed;
    cppfmu::FMIReal InputWaterCurrentAngle;
    std::array<cppfmu::FMIReal, 5> InputWaveHeight;

    // Parameters from FMU
    cppfmu::FMIInteger ParameterHullMass;
    cppfmu::FMIInteger ParameterHullLength;
    cppfmu::FMIInteger ParameterHullWidth;
    cppfmu::FMIInteger ParameterHullHeight;
    cppfmu::FMIInteger ParameterThrusterBowPositionX;
    cppfmu::FMIInteger ParameterThrusterPortPositionX;
    cppfmu::FMIInteger ParameterThrusterPortPositionY;
    cppfmu::FMIInteger ParameterThrusterStarboardPositionX;
    cppfmu::FMIInteger ParameterThrusterStarboardPositionY;

    // Outputs from FMU
    std::array<cppfmu::FMIReal, 6> OutputHullPosition;
    std::array<cppfmu::FMIReal, 6> OutputHullVelocity;
};



// This MUST be here as it initiates the binding between simple C++ code class and the FMUs world
// This function also allows to pass variables like fmuResourceLocation that is filepath to rescources folder when the fmu is runnjing, this parameter can then be passed to the return cppfmu::AllocateUnique<ThrusterBowFMU>(memory, fmuResourceLocation);
// The same concept for all other parameters displayed in the functiopn, you dont have to use them, but they are here for conveniece
cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString fmuGUID, cppfmu::FMIString /*fmuResourceLocation*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }
    // Class in AllocateUnique is the class that inherits from cppfmu::SlaveInstance and contains the FMU code
    return cppfmu::AllocateUnique<HullFMU>(memory);
}


