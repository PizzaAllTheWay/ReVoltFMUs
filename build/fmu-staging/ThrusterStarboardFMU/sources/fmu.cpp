#include "fmu-uuid.h"
#include <cstring> // For strcmp
#include <array>
#include <cppfmu_cs.hpp>

// Custom made models
#include "../libraries/thrusterStarboardModel.hpp"



class ThrusterStarboardFMU : public cppfmu::SlaveInstance
{
public:
    // Constructor must be present
    ThrusterStarboardFMU()
    {
        ThrusterStarboardFMU::Reset();
    }

    // Reset function is called at the beginning of each simulation
    void Reset() override
    {
        InputForce = 0.0;
        InputAngle = 0.0;

        ParameterThrusterEfficiency = 90; // 90 %

        // Don't need to define it perfectly, however is nice to set a specific value for start just in case
        OutputForce = 0.0;
        OutputAngle = 0.0;
    }

    // When the simulations tells FMU to set FMUVaraibles to specific values
    // This function loops through all the Real Varaibles in XML file, BUT if it finds differemnt data structure like Integer, then it will call function SetInteger() instead, so we must define this funmction to handle Integer variables later
    // NOTE: We put in OutputForce and OutputAngle cases here because in case there is some outside STC simulator stuff that wants to completely overwrite our Outputs they have the ability to do so, not necesarry, but nice to have
    void SetReal(const cppfmu::FMIValueReference valueReferences[], std::size_t number_of_value_references, const cppfmu::FMIReal values[]) override
    {
        for (std::size_t index = 0; index < number_of_value_references; ++index) {
            cppfmu::FMIValueReference valueReference = valueReferences[index];
            cppfmu::FMIReal value = values[index];
            switch(valueReference) {
                // Value reference coresponds to value references specified in XML
                case 0:
                    InputForce = value;
                    break;
                case 1:
                    InputAngle = value;
                    break;
                case 3:
                    OutputForce = value;
                    break;
                case 4:
                    OutputAngle = value;
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
                case 2:
                    ParameterThrusterEfficiency = value;
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
                    values[index] = InputForce;
                    break;
                case 1:
                    values[index] = InputAngle;
                    break;
                case 3:
                    values[index] = OutputForce;
                    break;
                case 4:
                    values[index] = OutputAngle;
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
                case 2:
                    values[index] = ParameterThrusterEfficiency;
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
        // Convert from int to double as the simulation function requires parameters to be in int
        // This parameter is percentage, so we need to also divide by 100 to get the correct ratio
        double parameterThrusterEfficiency = static_cast<cppfmu::FMIReal>(ParameterThrusterEfficiency)/100.0;
        
        // Simulation itself
        std::array<double, 2> results = ThrusterStarboardModel::simulate(
            timeStep,
            InputForce,
            InputAngle,
            parameterThrusterEfficiency,
            OutputForce,
            OutputAngle
        );

        // Saving simulated results to corresponding outputs
        OutputForce = static_cast<cppfmu::FMIReal>(results[0]);
        OutputAngle = static_cast<cppfmu::FMIReal>(results[1]);
        // Simulation (STOP) --------------------------------------------------

        // Specify we are done with the simulation step
        return true;
    }

private:
    // Variables from .xml file that binds FMU code with C++ code
    // Inputs from FMU
    cppfmu::FMIReal InputForce;
    cppfmu::FMIReal InputAngle;

    // Parameters from FMU
    cppfmu::FMIInteger ParameterThrusterEfficiency;

    // Outputs from FMU
    cppfmu::FMIReal OutputForce;
    cppfmu::FMIReal OutputAngle;
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
    return cppfmu::AllocateUnique<ThrusterStarboardFMU>(memory);
}


