set(COMPONENT_NAME "FMU2cs_PathFollowerDriver")
set(COMPONENT_SOURCES FMU_PathFollowerDriver.h FMU_PathFollowerDriver.cpp)
set(COMPONENT_RESOURCES_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources_staging)
file(MAKE_DIRECTORY "${COMPONENT_RESOURCES_DIR}")

set(FMU_CS ON)

include_directories(${FMU_FORGE_DIR})

message(STATUS "...add ${COMPONENT_NAME}")

set(FMU_MODEL_IDENTIFIER "${COMPONENT_NAME}")
set(USE_CUSTOM_TYPESPLATFORM OFF)
set(FMU_MAIN_DIRECTORY ${CMAKE_BINARY_DIR})
set(FMU_RESOURCES_DIRECTORY ${COMPONENT_RESOURCES_DIR})
set(FMU_MSG_PREFIX "   - ")

# Custom target to stage driver resources at build time
add_custom_target(stage_driver_resources ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${COMPONENT_RESOURCES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/generated/default_lane_change_path.txt" "${COMPONENT_RESOURCES_DIR}/"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CHRONO_DATA_DIR}/fonts" "${COMPONENT_RESOURCES_DIR}/fonts"
    COMMENT "Staging Driver FMU resources at build time..."
)

# Determine dependencies
if(CHRONO_IRRLICHT_FOUND AND WIN32)
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

add_dependencies(${FMU_MODEL_IDENTIFIER} stage_driver_resources)
