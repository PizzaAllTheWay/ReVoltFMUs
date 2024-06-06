# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build

# Include any dependencies generated for this target.
include CMakeFiles/IMUFMU.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/IMUFMU.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/IMUFMU.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/IMUFMU.dir/flags.make

CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o: CMakeFiles/IMUFMU.dir/flags.make
CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o: ../src/IMUFMU/sources/fmu.cpp
CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o: CMakeFiles/IMUFMU.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o -MF CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o.d -o CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o -c /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/sources/fmu.cpp

CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/sources/fmu.cpp > CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.i

CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/sources/fmu.cpp -o CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.s

CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o: CMakeFiles/IMUFMU.dir/flags.make
CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o: ../src/IMUFMU/libraries/IMUModel.cpp
CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o: CMakeFiles/IMUFMU.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o -MF CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o.d -o CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o -c /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/libraries/IMUModel.cpp

CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/libraries/IMUModel.cpp > CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.i

CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/IMUFMU/libraries/IMUModel.cpp -o CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.s

CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o: CMakeFiles/IMUFMU.dir/flags.make
CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o: ../cppfmu/cppfmu_cs.cpp
CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o: CMakeFiles/IMUFMU.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o -MF CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o.d -o CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o -c /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/cppfmu_cs.cpp

CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/cppfmu_cs.cpp > CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.i

CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/cppfmu_cs.cpp -o CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.s

CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o: CMakeFiles/IMUFMU.dir/flags.make
CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o: ../cppfmu/fmi_functions.cpp
CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o: CMakeFiles/IMUFMU.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o -MF CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o.d -o CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o -c /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/fmi_functions.cpp

CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/fmi_functions.cpp > CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.i

CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cppfmu/fmi_functions.cpp -o CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.s

# Object files for target IMUFMU
IMUFMU_OBJECTS = \
"CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o" \
"CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o" \
"CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o" \
"CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o"

# External object files for target IMUFMU
IMUFMU_EXTERNAL_OBJECTS =

IMUFMU.so: CMakeFiles/IMUFMU.dir/src/IMUFMU/sources/fmu.cpp.o
IMUFMU.so: CMakeFiles/IMUFMU.dir/src/IMUFMU/libraries/IMUModel.cpp.o
IMUFMU.so: CMakeFiles/IMUFMU.dir/cppfmu/cppfmu_cs.cpp.o
IMUFMU.so: CMakeFiles/IMUFMU.dir/cppfmu/fmi_functions.cpp.o
IMUFMU.so: CMakeFiles/IMUFMU.dir/build.make
IMUFMU.so: CMakeFiles/IMUFMU.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared module IMUFMU.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IMUFMU.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/IMUFMU.dir/build: IMUFMU.so
.PHONY : CMakeFiles/IMUFMU.dir/build

CMakeFiles/IMUFMU.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/IMUFMU.dir/cmake_clean.cmake
.PHONY : CMakeFiles/IMUFMU.dir/clean

CMakeFiles/IMUFMU.dir/depend:
	cd /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles/IMUFMU.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/IMUFMU.dir/depend
