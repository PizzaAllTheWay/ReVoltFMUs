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

# Utility rule file for GNSSFMU_fmu.

# Include any custom commands dependencies for this target.
include CMakeFiles/GNSSFMU_fmu.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/GNSSFMU_fmu.dir/progress.make

CMakeFiles/GNSSFMU_fmu: GNSSFMU.fmu

GNSSFMU.fmu: GNSSFMU.so
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating GNSSFMU.fmu"
	/usr/bin/cmake -E copy_directory /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/src/GNSSFMU /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU
	/usr/bin/cmake -E make_directory /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU/binaries/linux64
	/usr/bin/cmake -E copy /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/GNSSFMU.so /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU/binaries/linux64/
	/usr/bin/cmake -E copy /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-uuids/GNSSFMU/modelDescription.xml /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU/modelDescription.xml
	/usr/bin/cmake -E copy /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-uuids/GNSSFMU/fmu-uuid.h /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU/sources/
	/usr/bin/cmake -DBASE_DIR=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/fmu-staging/GNSSFMU -DOUTPUT_FILE=/home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/GNSSFMU.fmu -P /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/cmake/ZipAll.cmake

GNSSFMU_fmu: CMakeFiles/GNSSFMU_fmu
GNSSFMU_fmu: GNSSFMU.fmu
GNSSFMU_fmu: CMakeFiles/GNSSFMU_fmu.dir/build.make
.PHONY : GNSSFMU_fmu

# Rule to build all files generated by this target.
CMakeFiles/GNSSFMU_fmu.dir/build: GNSSFMU_fmu
.PHONY : CMakeFiles/GNSSFMU_fmu.dir/build

CMakeFiles/GNSSFMU_fmu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/GNSSFMU_fmu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/GNSSFMU_fmu.dir/clean

CMakeFiles/GNSSFMU_fmu.dir/depend:
	cd /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build /home/martynas/revolt_model_ws/martynas_revolt_cpp_fmu/build/CMakeFiles/GNSSFMU_fmu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/GNSSFMU_fmu.dir/depend
