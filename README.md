# ReVolt FMU Models
Compiles cpp models of ReVolt Vessel to FMU
- Trusters Models (Bow, Stern Port, Stern Starboard)
- Hull Model
- GNSS Model
- IMU Model

# Example
https://github.com/PizzaAllTheWay/ReVoltFMUs/assets/42731091/6a74ad7f-a5d1-4589-8598-5f8fa1f8295d

## Note
I tried my best to represent the ReVolt vessel in simulations, however its in no way perfect, far from it. Its a simplified model of the physical vessel. However it is good enough for most things that require low to medium fidelity of ship dynamics.

## Strucure
All cpp FMU code is contained in [src](./src). 
The name of the folder in src is the same as the name of the FMU. 
To make more FMUs make a new folder with the FMU name, Then add the FMU folder to the CMakeList.txt file. you can copy the formatting for the MySimpleFMU folder.

## Build and testing
For building use the "(operating system)BuildFMUs" which will build all the fmus
to test your fmus you can execute "verifyFMUs.py" which you can edit to verify specific individual or multiple FMUs, as well as setting their parameters and initial conditions for the simulation and plot their response.

## Dependencies
Code dependencies that are taken from other repositories (mostly Open Simulation Platform):
- [cmake](./cmake), [fmi](./fmi) and [CMakeLists.txt](./CMakeLists.txt) is from [this repo](https://github.com/open-simulation-platform/cpp-fmus)
- [cppfmu](./cppfmu/) is from [this repo](https://github.com/viproma/cppfmu)

Dependencies to build and test FMUs
- CMake (v3.0)
- FMPy (v0.3.20) (Python library)
- Tor Inge Fosse X)

## Useful literature for understanding what is going on
Good book to read up on proper simulation and modeling:
https://folk.ntnu.no/oe/Modeling%20and%20Simulation.pdf

Thank you Tor Inge Fosse, this saved my ass:
https://www.researchgate.net/publication/226128788_Kinematics_of_Ship_Motion
