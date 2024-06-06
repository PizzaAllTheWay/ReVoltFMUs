#include "fmu-uuid.h"
#include <cstring> // For strcmp
#include <array>
#include <cppfmu_cs.hpp>

// Custom made models
#include "../libraries/GNSSModel.hpp"



class GNSSFMU : public cppfmu::SlaveInstance
{
public:
    // Constructor must be present
    GNSSFMU()
    {
        GNSSFMU::Reset();
    }

    // Reset function is called at the beginning of each simulation
    void Reset() override
    {
        InputHullPosition = {
            0.0, // X Position [m]
            0.0, // Y Position [m]
            0.0, // Z Position [m]
            0.0, // Roll Position [rad]
            0.0, // Pitch Position [rad]
            0.0 // Yaw Position [rad]
        };
        InputHullVelocity = {
            0.0, // X Linear Velocity [m/s]
            0.0, // Y Linear Velocity [m/s]
            0.0, // Z Linear Velocity [m/s]
            0.0, // Roll Angular Velocity [rad/s]
            0.0, // Pitch Angular Velocity [rad/s]
            0.0 // Yaw Angular Velocity [rad/s]
        };

        ParameterAntennaDistance = 300; // [cm]
        ParameterTimeBetweenData = 2000; // [ms]
        ParameterStandardDeviation = 50; // [cm]

        // Don't need to define it perfectly, however is nice to set a specific value for start just in case
        OutputAntenna1Position = {
            0.0, // X [m]
            0.0, // Y [m]
            0.0 // Z [m]
        };
        OutputAntenna2Position = {
            0.0, // X [m]
            0.0, // Y [m]
            0.0 // Z [m]
        };
        OutputVelocitySpeed = 0; // [m/s]
        OutputVelocityAngle = 0; // [rad]
    }

    // When the simulations tells FMU to set FMUVariables to specific values
    // This function loops through all the Real Variables in XML file, BUT if it finds different data structure like Integer, then it will call function SetInteger() instead, so we must define this function to handle Integer variables later
    // NOTE: We put in OutputForce and OutputAngle cases here because in case there is some outside STC simulator stuff that wants to completely overwrite our Outputs they have the ability to do so, not necessary, but nice to have
    void SetReal(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, const cppfmu::FMIReal values[]) override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            cppfmu::FMIReal value = values[index];
            switch(valueReference) {
                // Value reference corresponds to value references specified in XML
                case 0:
                    InputHullPosition[0] = value;
                    break;
                case 1:
                    InputHullPosition[1] = value;
                    break;
                case 2:
                    InputHullPosition[2] = value;
                    break;
                case 3:
                    InputHullPosition[3] = value;
                    break;
                case 4:
                    InputHullPosition[4] = value;
                    break;
                case 5:
                    InputHullPosition[5] = value;
                    break;
                case 6:
                    InputHullVelocity[0] = value;
                    break;
                case 7:
                    InputHullVelocity[1] = value;
                    break;
                case 8:
                    InputHullVelocity[2] = value;
                    break;
                case 9:
                    InputHullVelocity[3] = value;
                    break;
                case 10:
                    InputHullVelocity[4] = value;
                    break;
                case 11:
                    InputHullVelocity[5] = value;
                    break;

                case 15:
                    OutputAntenna1Position[0] = value;
                    break;
                case 16:
                    OutputAntenna1Position[1] = value;
                    break;
                case 17:
                    OutputAntenna1Position[2] = value;
                    break;
                case 18:
                    OutputAntenna2Position[0] = value;
                    break;
                case 19:
                    OutputAntenna2Position[1] = value;
                    break;
                case 20:
                    OutputAntenna2Position[2] = value;
                    break;
                case 21:
                    OutputVelocitySpeed = value;
                    break;
                case 22:
                    OutputVelocityAngle = value;
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
                case 12:
                    ParameterAntennaDistance = value;
                    break;
                case 13:
                    ParameterTimeBetweenData = value;
                    break;
                case 14:
                    ParameterStandardDeviation = value;
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
                    values[index] = InputHullPosition[0];
                    break;
                case 1:
                    values[index] = InputHullPosition[1];
                    break;
                case 2:
                    values[index] = InputHullPosition[2];
                    break;
                case 3:
                    values[index] = InputHullPosition[3];
                    break;
                case 4:
                    values[index] = InputHullPosition[4];
                    break;
                case 5:
                    values[index] = InputHullPosition[5];
                    break;
                case 6:
                    values[index] = InputHullVelocity[0];
                    break;
                case 7:
                    values[index] = InputHullVelocity[1];
                    break;
                case 8:
                    values[index] = InputHullVelocity[2];
                    break;
                case 9:
                    values[index] = InputHullVelocity[3];
                    break;
                case 10:
                    values[index] = InputHullVelocity[4];
                    break;
                case 11:
                    values[index] = InputHullVelocity[5];
                    break;

                case 15:
                    values[index] = OutputAntenna1Position[0];
                    break;
                case 16:
                    values[index] = OutputAntenna1Position[1];
                    break;
                case 17:
                    values[index] = OutputAntenna1Position[2];
                    break;
                case 18:
                    values[index] = OutputAntenna2Position[0];
                    break;
                case 19:
                    values[index] = OutputAntenna2Position[1];
                    break;
                case 20:
                    values[index] = OutputAntenna2Position[2];
                    break;
                case 21:
                    values[index] = OutputVelocitySpeed;
                    break;
                case 22:
                    values[index] = OutputVelocityAngle;
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
                case 12:
                    values[index] = ParameterAntennaDistance;
                    break;
                case 13:
                    values[index] = ParameterTimeBetweenData;
                    break;
                case 14:
                    values[index] = ParameterStandardDeviation;
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
        std::array<double, 8> results = GNSSModel::simulate(
            timeStep,

            InputHullPosition,
            InputHullVelocity,

            ParameterAntennaDistance,
            ParameterTimeBetweenData,
            ParameterStandardDeviation,

            OutputAntenna1Position,
            OutputAntenna2Position,
            OutputVelocitySpeed,
            OutputVelocityAngle
        );

        // Saving simulated results to corresponding outputs
        for (int i = 0; i < 3; i++) {
            OutputAntenna1Position[i] = results[i];
            OutputAntenna2Position[i] = results[i + 3];
        }
        OutputVelocitySpeed = results[6];
        OutputVelocityAngle = results[7];
        // Simulation (STOP) --------------------------------------------------

        // Specify we are done with the simulation step
        return true;
    }

private:
    // Variables from .xml file that binds FMU code with C++ code
    // Inputs from FMU
    std::array<cppfmu::FMIReal, 6> InputHullPosition;
    std::array<cppfmu::FMIReal, 6> InputHullVelocity;

    // Parameters from FMU
    cppfmu::FMIInteger ParameterAntennaDistance;
    cppfmu::FMIInteger ParameterTimeBetweenData;
    cppfmu::FMIInteger ParameterStandardDeviation;

    // Outputs from FMU
    std::array<cppfmu::FMIReal, 3> OutputAntenna1Position;
    std::array<cppfmu::FMIReal, 3> OutputAntenna2Position;
    cppfmu::FMIReal OutputVelocitySpeed;
    cppfmu::FMIReal OutputVelocityAngle;
};


// This MUST be here as it initiates the binding between simple C++ code class and the FMUs world
// This function also allows to pass variables like fmuResourceLocation that is filepath to rescources folder when the fmu is runnjing, this parameter can then be passed to the return cppfmu::AllocateUnique<GNSSFMU>(memory, fmuResourceLocation);
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
    return cppfmu::AllocateUnique<GNSSFMU>(memory);
}


