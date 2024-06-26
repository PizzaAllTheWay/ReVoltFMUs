cmake_minimum_required(VERSION 3.0)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

project(cpp-fmus)

option(FMU_USE_STATIC_RUNTIME
    "Whether to link the model code against a static C(++) runtime library"
    ON)
option(FMU_CHECK_COMPLIANCE
    "Whether to test each FMU with the FMU Compliance checker"
    OFF)
set(FMU_OUTPUT_DIR "${CMAKE_BINARY_DIR}"
    CACHE PATH "Where to put the generated FMUs")
set(FMU_STAGING_DIR "${CMAKE_BINARY_DIR}/fmu-staging"
    CACHE PATH "Where to put the generated FMU contents before zipping")
set(FMU_UUID_DIR "${CMAKE_BINARY_DIR}/fmu-uuids"
    CACHE PATH "Where to put the headers containing FMU UUIDs")

# IMPORTANT (START) --------------------------------------------------
# The names of the FMUs built in this project
set(fmus
    "GNSSFMU"
    "HullFMU"
    "IMUFMU"
    "ThrusterBowFMU"
    "ThrusterPortFMU"
    "ThrusterStarboardFMU"
)
# IMPORTANT (STOP) --------------------------------------------------

# ==============================================================================
# Compiler/platform specific settings
# ==============================================================================

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-Wall" "-Wextra" "-Wpedantic" "-Werror")
    add_compile_options("-Wno-parentheses")
    if(FMU_USE_STATIC_RUNTIME)
        add_compile_options("-static")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options("/W4" "/WX")
    add_compile_options("/wd4996")
    add_definitions("-D_SCL_SECURE_NO_WARNINGS" "-D_CRT_SECURE_NO_WARNINGS")
    if(FMU_USE_STATIC_RUNTIME)
        foreach(lang "C" "CXX")
            string(REPLACE "/MD" "/MT" "CMAKE_${lang}_FLAGS" "${${CMAKE_${lang}_FLAGS}}")
            foreach(config ${CMAKE_CONFIGURATION_TYPES})
                string(TOUPPER "${config}" configUpper)
                set(flagVar "CMAKE_${lang}_FLAGS_${configUpper}")
                string(REPLACE "/MD" "/MT" ${flagVar} ${${flagVar}})
            endforeach()
        endforeach()
    endif()
endif()

if(UNIX)
    # Remove the "lib" prefix on the generated .so files
    set(CMAKE_SHARED_MODULE_PREFIX)
endif()

# ==============================================================================
# Build
# ==============================================================================

# List CPPFMU sources
file(GLOB cppfmuSourceFiles "${CMAKE_SOURCE_DIR}/cppfmu/*.?pp")

# Find FMU Compliance Checker
if(FMU_CHECK_COMPLIANCE)
    enable_testing()
    find_package(FMUComplianceChecker REQUIRED)
endif()

# Detect platform
if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(os "darwin")
elseif("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
    set(os "linux")
elseif(WIN32)
    set(os "win")
else()
    message(FATAL_ERROR "Unknown or unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()
math(EXPR wordSize 8*${CMAKE_SIZEOF_VOID_P})
set(platformIdentifier "${os}${wordSize}")

# Prepare names for FMU staging directories.
if(CMAKE_CONFIGURATION_TYPES)
    set(configStagingDir "${FMU_STAGING_DIR}/$<CONFIGURATION>")
else()
    set(configStagingDir "${FMU_STAGING_DIR}")
endif()

# Create targets
set(includeDirs
)

foreach(fmuName IN LISTS fmus)
    set(fmuLibraryTarget "${fmuName}")
    set(fmuGuidTarget "${fmuName}_guid")
    set(fmuTarget "${fmuName}_fmu")

    set(fmuSourceDir "${CMAKE_SOURCE_DIR}/src/${fmuName}")
    set(fmuSourceModelDescription "${fmuSourceDir}/modelDescription.xml")

    # IMPORTANT (START) --------------------------------------------------
    # Iclude the custom made model library for the FMUs
    if("${fmuName}" STREQUAL "GNSSFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/GNSSFMU/libraries/GNSSModel.cpp"
        )
    elseif("${fmuName}" STREQUAL "HullFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/HullFMU/libraries/hullModel.cpp"
        )
    elseif("${fmuName}" STREQUAL "IMUFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/IMUFMU/libraries/IMUModel.cpp"
        )
    elseif("${fmuName}" STREQUAL "ThrusterBowFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/ThrusterBowFMU/libraries/thrusterBowModel.cpp"
        )
    elseif("${fmuName}" STREQUAL "ThrusterPortFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/ThrusterPortFMU/libraries/thrusterPortModel.cpp"
        )
    elseif("${fmuName}" STREQUAL "ThrusterStarboardFMU")
        set(fmuSourceFiles
            "${fmuSourceDir}/sources/fmu.cpp"
            "${CMAKE_SOURCE_DIR}/src/ThrusterStarboardFMU/libraries/thrusterStarboardModel.cpp"
        )
    endif()
    # IMPORTANT (STOP) --------------------------------------------------

    set(fmuGuidDir "${FMU_UUID_DIR}/${fmuName}")
    set(fmuGuidModelDescription "${fmuGuidDir}/modelDescription.xml")
    set(fmuGuidHeaderFile "${fmuGuidDir}/fmu-uuid.h")

    set(fmuStagingDir "${configStagingDir}/${fmuName}")
    set(fmuStagedBinariesDir "${fmuStagingDir}/binaries/${platformIdentifier}")

    set(fmuOutputFile "${FMU_OUTPUT_DIR}/${fmuName}.fmu")

    # Target to generate GUID
    add_custom_command(
        OUTPUT "${fmuGuidModelDescription}" "${fmuGuidHeaderFile}"
        COMMAND
        "${CMAKE_COMMAND}" "-E" "make_directory" "${fmuGuidDir}"
        COMMAND
            "${CMAKE_COMMAND}"
            "-DINPUT_MODEL_DESCRIPTION=${fmuSourceModelDescription}"
            "-DINPUT_HEADER=${CMAKE_SOURCE_DIR}/cmake/fmu-uuid.h.in"
            "-DOUTPUT_MODEL_DESCRIPTION=${fmuGuidModelDescription}"
            "-DOUTPUT_HEADER=${fmuGuidHeaderFile}"
            "-DADDITIONAL_INPUT=${fmuSourceFiles}"
            "-P" "${CMAKE_SOURCE_DIR}/cmake/GenerateFmuGuid.cmake"
        DEPENDS "${fmuSourceModelDescription}"
        VERBATIM
    )
    add_custom_target(${fmuGuidTarget}
        DEPENDS "${fmuGuidModelDescription}" "${fmuGuidHeaderFile}"
    )

    # Target to build dynamic library
    add_library(${fmuLibraryTarget} MODULE
        "${fmuSourceModelDescription}"
        ${fmuSourceFiles}
        ${cppfmuSourceFiles}
    )
    add_dependencies(${fmuLibraryTarget} ${fmuGuidTarget})
    target_include_directories(${fmuLibraryTarget} PRIVATE
        "${CMAKE_SOURCE_DIR}/fmi"
        "${CMAKE_SOURCE_DIR}/cppfmu"
        "${fmuGuidDir}"
    )

    # Target to generate FMU contents
    add_custom_command(OUTPUT "${fmuOutputFile}"
        COMMAND "${CMAKE_COMMAND}" "-E" "copy_directory" "${fmuSourceDir}" "${fmuStagingDir}"
        COMMAND "${CMAKE_COMMAND}" "-E" "make_directory" "${fmuStagedBinariesDir}"
        COMMAND "${CMAKE_COMMAND}" "-E" "copy" "$<TARGET_FILE:${fmuLibraryTarget}>" "${fmuStagedBinariesDir}/"
        COMMAND "${CMAKE_COMMAND}" "-E" "copy" "${fmuGuidModelDescription}" "${fmuStagingDir}/modelDescription.xml"
        COMMAND "${CMAKE_COMMAND}" "-E" "copy" "${fmuGuidHeaderFile}" "${fmuStagingDir}/sources/"
        COMMAND "${CMAKE_COMMAND}" "-DBASE_DIR=${fmuStagingDir}" "-DOUTPUT_FILE=${fmuOutputFile}" "-P" "${CMAKE_SOURCE_DIR}/cmake/ZipAll.cmake"
        DEPENDS ${fmuLibraryTarget} ${fmuGuidTarget}
        VERBATIM)
    add_custom_target(${fmuTarget} ALL DEPENDS "${fmuOutputFile}")

    # Test compliance
    if(FMU_CHECK_COMPLIANCE)
        add_fmu_compliance_check("${fmuTarget}_compliance" "${fmuOutputFile}")
    endif()
endforeach()

source_group("Metadata" REGULAR_EXPRESSION "modelDescription.xml")
