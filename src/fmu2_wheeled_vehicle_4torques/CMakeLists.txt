set(COMPONENT_NAME "FMU2cs_WheeledVehicle4Torques")
set(COMPONENT_SOURCES FMU_WheeledVehicle4Torques.h FMU_WheeledVehicle4Torques.cpp)
set(COMPONENT_RESOURCES_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources_staging)
file(MAKE_DIRECTORY "${COMPONENT_RESOURCES_DIR}")

set(FMU_CS ON)

include_directories(${FMU_FORGE_DIR})

message(STATUS "...add ${COMPONENT_NAME}")

set(FMU_MODEL_IDENTIFIER ${COMPONENT_NAME})
set(USE_CUSTOM_TYPESPLATFORM OFF)
set(FMU_MAIN_DIRECTORY ${CMAKE_BINARY_DIR})
set(FMU_RESOURCES_DIRECTORY ${COMPONENT_RESOURCES_DIR})
set(FMU_MSG_PREFIX "   - ")

# Custom target to stage resources at build time
add_custom_target(stage_fmu_resources ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${COMPONENT_RESOURCES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${COMPONENT_RESOURCES_DIR}/vehicle/sedan"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${COMPONENT_RESOURCES_DIR}/sedan"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${COMPONENT_RESOURCES_DIR}/fonts"
    # Copy JSONs and other local resources
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/resources" "${COMPONENT_RESOURCES_DIR}"
    # Copy standard vehicle meshes from Chrono Vehicle data (under both vehicle/sedan/ and sedan/)
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_chassis_vis.obj" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_chassis_vis.obj" "${COMPONENT_RESOURCES_DIR}/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_chassis_vis.mtl" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_chassis_vis.mtl" "${COMPONENT_RESOURCES_DIR}/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_rim.obj" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_rim.obj" "${COMPONENT_RESOURCES_DIR}/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_rim.mtl" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_rim.mtl" "${COMPONENT_RESOURCES_DIR}/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_tire.obj" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_tire.obj" "${COMPONENT_RESOURCES_DIR}/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_tire.mtl" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CHRONO_VEHICLE_DATA_DIR}/sedan/sedan_tire.mtl" "${COMPONENT_RESOURCES_DIR}/sedan/"
    # Copy standard tire specifications from Chrono Vehicle data
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CHRONO_VEHICLE_DATA_DIR}/sedan/tire" "${COMPONENT_RESOURCES_DIR}/vehicle/sedan/tire"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CHRONO_VEHICLE_DATA_DIR}/sedan/tire" "${COMPONENT_RESOURCES_DIR}/sedan/tire"
    # Copy build-time generated road terrain files from the build/generated/ directory
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/generated/default_road.crg" "${COMPONENT_RESOURCES_DIR}/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/generated/default_road.obj" "${COMPONENT_RESOURCES_DIR}/"
    # Copy fonts from Chrono data
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CHRONO_DATA_DIR}/fonts" "${COMPONENT_RESOURCES_DIR}/fonts"
    # Copy colormaps from Chrono data
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CHRONO_DATA_DIR}/colormaps" "${COMPONENT_RESOURCES_DIR}/colormaps"
    COMMENT "Staging FMU resources at build time..."
)

# Determine dependencies
if(CHRONO_IRRLICHT_FOUND AND WIN32)
  # Copy Irrlicht DLL if needed
  set(FMU_DEPENDENCIES "${Irrlicht_DLL}")
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

add_dependencies(${FMU_MODEL_IDENTIFIER} stage_fmu_resources)
