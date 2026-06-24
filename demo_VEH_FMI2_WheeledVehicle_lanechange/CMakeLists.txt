set(DEMO demo_VEH_FMI2_WheeledVehicle_lanechange)
set(DEMO_SOURCES demo_VEH_FMI2_WheeledVehicle_lanechange.cpp)

add_executable(${DEMO} ${DEMO_SOURCES})

target_include_directories(${DEMO} PRIVATE ${FMU_FORGE_DIR})
target_compile_definitions(${DEMO} PUBLIC FMU_UNPACK_DIRECTORY="${CMAKE_BINARY_DIR}/tmp_unpack")
target_compile_definitions(${DEMO} PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
target_compile_definitions(${DEMO} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_link_libraries(${DEMO} PRIVATE ${CHRONO_TARGETS})

if(MSVC)
  set_target_properties(${DEMO} PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()

add_dependencies(${DEMO} FMU2cs_WheeledVehicle4Torques)
add_dependencies(${DEMO} FMU2cs_PathFollowerDriver)
