cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0091 NEW)

project(external_fmus_project)

include(FetchContent)

# Find OpenCRG first since Chrono::Chrono_vehicle depends on target OpenCRG::OpenCRG
list(APPEND CMAKE_MODULE_PATH "c:/Users/novo/.gemini/antigravity/scratch/chrono/cmake")
set(OpenCRG_INCLUDE_DIR "c:/Users/novo/.gemini/antigravity/scratch/packages/openCRG/include")
set(OpenCRG_LIBRARY "c:/Users/novo/.gemini/antigravity/scratch/packages/openCRG/lib/OpenCRG.lib")
find_package(OpenCRG REQUIRED)

# Find Chrono
find_package(Chrono COMPONENTS FMI Vehicle OPTIONAL_COMPONENTS Irrlicht CONFIG REQUIRED)

if (NOT Chrono_FOUND)
  message(FATAL_ERROR "Could not find Chrono or one of its required modules")
  return()
endif()

message(STATUS "Found Chrono version: ${Chrono_VERSION}")
message(STATUS "Chrono_DIR:           ${Chrono_DIR}")
message(STATUS "CHRONO_TARGETS:       ${CHRONO_TARGETS}")
message(STATUS "CHRONO_FMU_FORGE_DIR: ${CHRONO_FMU_FORGE_DIR}")

# Set FMU forge directory
if(NOT DEFINED FMU_FORGE_DIR)
  set(FMU_FORGE_DIR "${CHRONO_FMU_FORGE_DIR}")
endif()

# We enable testing if requested
set(TESTING_ENABLED OFF)

add_subdirectory(fmu2_wheeled_vehicle_4torques)
add_subdirectory(fmu2_path_follower_driver)
add_subdirectory(demo_VEH_FMI2_WheeledVehicle_4torques)
add_subdirectory(demo_VEH_FMI2_WheeledVehicle_lanechange)
