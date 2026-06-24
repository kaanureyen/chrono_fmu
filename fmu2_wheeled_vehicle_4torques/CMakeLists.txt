set(COMPONENT_NAME "FMU2cs_WheeledVehicle4Torques")
set(COMPONENT_SOURCES FMU_WheeledVehicle4Torques.h FMU_WheeledVehicle4Torques.cpp)
set(COMPONENT_RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)

set(FMU_CS ON)

include_directories(${FMU_FORGE_DIR})

message(STATUS "...add ${COMPONENT_NAME}")

set(FMU_MODEL_IDENTIFIER ${COMPONENT_NAME})
set(USE_CUSTOM_TYPESPLATFORM OFF)
set(FMU_MAIN_DIRECTORY ${CMAKE_BINARY_DIR})
set(FMU_RESOURCES_DIRECTORY ${COMPONENT_RESOURCES_DIR})
set(FMU_MSG_PREFIX "   - ")

# Determine dependencies
if(CH_ENABLE_MODULE_IRRLICHT AND Windows)
  # Copy Irrlicht DLL if needed
  set(FMU_DEPENDENCIES "${CH_IRRLICHT_DLL}")
endif()

FetchContent_Declare(
    ${FMU_MODEL_IDENTIFIER}
    SOURCE_DIR ${FMU_FORGE_DIR}/fmi2
)

FetchContent_MakeAvailable(${FMU_MODEL_IDENTIFIER})

# add FMU source files to target
target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE ${COMPONENT_SOURCES})

# Link libraries
target_link_libraries(${FMU_MODEL_IDENTIFIER} PRIVATE ${CHRONO_TARGETS})

if(MSVC)
  set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()
