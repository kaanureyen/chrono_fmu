// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antigravity AI
// Repository: https://github.com/kaanureyen/chrono_fmu
// =============================================================================
//
// Co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels.
// The vehicle does not include an engine, transmission, or tires.
// It accepts independent wheel torque inputs directly at the axles.
//
// =============================================================================

#include <cassert>
#include <map>
#include <algorithm>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>

#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

#include "FMU_WheeledVehicle4Torques.h"

#ifdef CHRONO_HAS_OPENCRG
#include "chrono_vehicle/terrain/CRGTerrain.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
fmu_forge::fmi2::FmuComponentBase* fmu_forge::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
                                                                        fmi2Type fmuType,
                                                                        fmi2String fmuGUID,
                                                                        fmi2String fmuResourceLocation,
                                                                        const fmi2CallbackFunctions* functions,
                                                                        fmi2Boolean visible,
                                                                        fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn), render_frame(0), last_render_time(-1.0) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    g_acc = {0, 0, -9.8};
    driver_inputs = {0, 0, 0, 0};
    init_loc = {0, 0, 0};
    init_yaw = 0;
    init_vel = 22.2222; // Default to 80 kph (22.2222 m/s)

    torque_FL = 0;
    torque_FR = 0;
    torque_RL = 0;
    torque_RR = 0;

    act_force_FL = 0;
    act_force_FR = 0;
    act_force_RL = 0;
    act_force_RR = 0;

    active_susp_actuators = {nullptr, nullptr, nullptr, nullptr};

    system_SMC = 1;
    step_size = 1e-3;

    out_path = ".";
    save_img = false;
    fps = 30;
    m_visible = visible;
    fmu_visible = false;
    vis_driver = 0;

    // Get default JSON files (relative to FMU resources at runtime)
    resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    data_path = "";
    vehicle_JSON = "Vehicle.json";
    tire_JSON = "vehicle/sedan/tire/Sedan_Pac02Tire.json";
    terrain_type = 2; // Default to OpenCRG
    tire_coll_type = 2; // Default to 2D Profile Envelope
    terrain_mesh_file = "";
    terrain_crg_file = "default_road.crg";
    terrain_friction = 0.8;

    // Set wheel identifier strings
    wheel_data[0].identifier = "FL";
    wheel_data[1].identifier = "FR";
    wheel_data[2].identifier = "RL";
    wheel_data[3].identifier = "RR";

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&data_path, "data_path", FmuVariable::Type::String, "1", "vehicle data path",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVariable(&vehicle_JSON, "vehicle_JSON", FmuVariable::Type::String, "1", "vehicle JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);    //

    AddFmuVariable(&tire_JSON, "tire_JSON", FmuVariable::Type::String, "1", "tire JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);    //
    AddFmuVariable(&tire_coll_type, "tire_coll_type", FmuVariable::Type::Integer, "1", "tire collision type (0: single, 1: four points, 2: envelope)",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                                       //

    AddFmuVariable(&terrain_type, "terrain_type", FmuVariable::Type::Integer, "1", "terrain type (0: Flat, 1: Mesh OBJ, 2: OpenCRG)",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                                     //

    AddFmuVariable(&terrain_mesh_file, "terrain_mesh_file", FmuVariable::Type::String, "1", "terrain mesh file (.obj)",     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //

    AddFmuVariable(&terrain_crg_file, "terrain_crg_file", FmuVariable::Type::String, "1", "terrain CRG file (.crg)",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //

    AddFmuVariable(&terrain_friction, "terrain_friction", FmuVariable::Type::Real, "1", "terrain friction",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);    //

    AddFmuVariable(&system_SMC, "system_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVariable(&vis_driver, "vis_driver", FmuVariable::Type::Integer, "1", "visual driver (0: default, 1: OpenGL, 2: D3D9, 3: Software, 4: Burning)", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed); //

    AddFmuVecVariable(init_loc, "init_loc", "m", "initial location",                                //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial location Z",     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVecVariable(g_acc, "g_acc", "m/s2", "gravitational acceleration",                         //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVariable(&init_vel, "init_vel", FmuVariable::Type::Real, "m/s", "initial velocity",       //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVariable(&fmu_visible, "visible", FmuVariable::Type::Boolean, "1", "enable visualization window", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set FIXED PARAMETERS for this FMU (I/O)
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&fps, "fps", FmuVariable::Type::Real, "1", "rendering frequency",             //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU (driver inputs - steering & braking only)
    AddFmuVariable(&driver_inputs.m_steering, "steering", FmuVariable::Type::Real, "1", "steering input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_braking, "braking", FmuVariable::Type::Real, "1", "braking input",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    // Set CONTINOUS INPUTS for this FMU (4 wheel motor torques)
    AddFmuVariable(&torque_FL, "torque_FL", FmuVariable::Type::Real, "Nm", "Front Left motor torque",      //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&torque_FR, "torque_FR", FmuVariable::Type::Real, "Nm", "Front Right motor torque",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&torque_RL, "torque_RL", FmuVariable::Type::Real, "Nm", "Rear Left motor torque",       //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&torque_RR, "torque_RR", FmuVariable::Type::Real, "Nm", "Rear Right motor torque",      //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    // Set CONTINOUS INPUTS for this FMU (4 active suspension forces)
    AddFmuVariable(&act_force_FL, "act_force_FL", FmuVariable::Type::Real, "N", "Front Left active suspension force",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&act_force_FR, "act_force_FR", FmuVariable::Type::Real, "N", "Front Right active suspension force", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&act_force_RL, "act_force_RL", FmuVariable::Type::Real, "N", "Rear Left active suspension force",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&act_force_RR, "act_force_RR", FmuVariable::Type::Real, "N", "Rear Right active suspension force", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //


    // Set DISCRETE INPUTS for this FMU (I/O) [rebuilt]
    AddFmuVariable(&save_img, "save_img", FmuVariable::Type::Boolean, "1", "trigger saving images",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::discrete);               //

    // Set CONTINUOUS OUTPUTS for this FMU (vehicle reference frame)
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                          //
                              FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINUOUS OUTPUTS for this FMU (wheel state and study KPIs for monitoring)
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state.lin_vel = VNULL;
        wheel_data[iw].state.ang_vel = VNULL;

        tire_force[iw] = VNULL;
        tire_slip_angle[iw] = 0;
        tire_slip_ratio[iw] = 0;
        susp_travel[iw] = 0;
        susp_velocity[iw] = 0;

        std::string prefix = "wheel_" + wheel_data[iw].identifier;

        AddFmuVecVariable(wheel_data[iw].state.pos, prefix + ".pos", "m", prefix + " position",                      //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
        AddFmuQuatVariable(wheel_data[iw].state.rot, prefix + ".rot", "1", prefix + " rotation",                     //
                           FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);            //
        AddFmuVecVariable(wheel_data[iw].state.lin_vel, prefix + ".lin_vel", "m/s", prefix + " linear velocity",     //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //
        AddFmuVecVariable(wheel_data[iw].state.ang_vel, prefix + ".ang_vel", "rad/s", prefix + " angular velocity",  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //

        AddFmuVecVariable(tire_force[iw], prefix + ".force", "N", prefix + " tire contact force",                  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);            //
        AddFmuVariable(&tire_slip_angle[iw], prefix + ".slip_angle", FmuVariable::Type::Real, "rad",                //
                       prefix + " tire slip angle",                                                                 //
                       FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);               //
        AddFmuVariable(&tire_slip_ratio[iw], prefix + ".slip_ratio", FmuVariable::Type::Real, "1",                  //
                       prefix + " tire slip ratio",                                                                 //
                       FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);               //

        std::string susp_prefix = "susp_" + wheel_data[iw].identifier;
        AddFmuVariable(&susp_travel[iw], susp_prefix + ".travel", FmuVariable::Type::Real, "m",                     //
                       susp_prefix + " travel",                                                                     //
                       FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);               //
        AddFmuVariable(&susp_velocity[iw], susp_prefix + ".velocity", FmuVariable::Type::Real, "m/s",               //
                       susp_prefix + " velocity",                                                                   //
                       FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);               //
    }

    // Specify variable dependencies
    DeclareVariableDependencies("ref_frame", {"init_loc", "init_yaw"});
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(prefix + ".pos", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".rot", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".force", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".slip_angle", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".slip_ratio", {"init_loc", "init_yaw"});

        std::string susp_prefix = "susp_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(susp_prefix + ".travel", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(susp_prefix + ".velocity", {"init_loc", "init_yaw"});
    }

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateVehicleOutputs(); });
}

void FmuComponent::CreateVehicle() {
    // Force data_path to point to the internal FMU resources directory
    data_path = resources_dir + "/";
    vehicle::SetVehicleDataPath(data_path);
    chrono::SetChronoDataPath(data_path);

    // Resolve vehicle_JSON relative to the resources directory if relative/not found
    std::string resolved_vehicle_JSON = vehicle_JSON;
    std::filesystem::path p_vehicle(vehicle_JSON);
    if (p_vehicle.is_relative()) {
        resolved_vehicle_JSON = (std::filesystem::path(data_path) / p_vehicle).string();
    }
    if (!std::filesystem::exists(resolved_vehicle_JSON)) {
        resolved_vehicle_JSON = (std::filesystem::path(data_path) / "Vehicle.json").string();
    }

    std::cout << "Create 4-wheel torque input vehicle FMU" << std::endl;
    std::cout << " Data path:         " << data_path << std::endl;
    std::cout << " Vehicle JSON:      " << resolved_vehicle_JSON << std::endl;
    std::cout << " Initial location:  " << init_loc << std::endl;
    std::cout << " Initial yaw:       " << init_yaw << std::endl;
    std::cout << " Initial velocity:  " << init_vel << std::endl;

    std::cout << "[FMU] Current working directory: " << std::filesystem::current_path().string() << std::endl;
    std::cout << "[FMU] Vehicle JSON file status: " << (std::filesystem::exists(resolved_vehicle_JSON) ? "FOUND" : "NOT FOUND") << std::endl;

    std::cout << "[DEBUG] Creating vehicle..." << std::endl;
    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(resolved_vehicle_JSON, system_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    std::cout << "[DEBUG] Vehicle created. Initializing..." << std::endl;
    vehicle->Initialize(ChCoordsys<>(init_loc + ChVector3d(0, 0, 0.5), QuatFromAngleZ(init_yaw)), init_vel);
    std::cout << "[DEBUG] Vehicle initialized." << std::endl;

    // Initialize the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    // Cache vehicle wheels
    wheel_data[0].wheel = vehicle->GetWheel(0, VehicleSide::LEFT);
    wheel_data[1].wheel = vehicle->GetWheel(0, VehicleSide::RIGHT);
    wheel_data[2].wheel = vehicle->GetWheel(1, VehicleSide::LEFT);
    wheel_data[3].wheel = vehicle->GetWheel(1, VehicleSide::RIGHT);

    // Disconnect the driveline to allow direct manual application of wheel torques
    vehicle->DisconnectDriveline();

    // Create and initialize the terrain
    std::cout << "[DEBUG] Configuring terrain..." << std::endl;
    if (terrain_type == 1 && !terrain_mesh_file.empty()) {
        std::string resolved_mesh_file = terrain_mesh_file;
        std::filesystem::path p(terrain_mesh_file);
        if (p.is_relative()) {
            resolved_mesh_file = (std::filesystem::path(data_path) / p).string();
        }
        std::cout << "[FMU] Attempting to read terrain mesh file (absolute): " << std::filesystem::absolute(resolved_mesh_file).string() << std::endl;
        std::cout << "[FMU] Terrain mesh file status: " << (std::filesystem::exists(resolved_mesh_file) ? "FOUND" : "NOT FOUND") << std::endl;
        auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(vehicle->GetSystem());
        std::shared_ptr<ChContactMaterial> material;
        if (system_SMC) {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(static_cast<float>(terrain_friction));
            material = matSMC;
        } else {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(static_cast<float>(terrain_friction));
            material = matNSC;
        }
        rigid_terrain->AddPatch(material, ChCoordsys<>(), resolved_mesh_file, true, 0.0, true);
        rigid_terrain->Initialize();
        terrain = rigid_terrain;
        std::cout << "Configured RigidTerrain mesh: " << resolved_mesh_file << std::endl;
    }
#ifdef CHRONO_HAS_OPENCRG
    else if (terrain_type == 2 && !terrain_crg_file.empty()) {
        std::string resolved_crg_file = terrain_crg_file;
        std::filesystem::path p(terrain_crg_file);
        if (p.is_relative()) {
            resolved_crg_file = (std::filesystem::path(data_path) / p).string();
        }
        std::cout << "[FMU] Attempting to read terrain CRG file (absolute): " << std::filesystem::absolute(resolved_crg_file).string() << std::endl;
        std::cout << "[FMU] Terrain CRG file status: " << (std::filesystem::exists(resolved_crg_file) ? "FOUND" : "NOT FOUND") << std::endl;
        auto crg_terrain = chrono_types::make_shared<CRGTerrain>(vehicle->GetSystem());
        crg_terrain->SetContactFrictionCoefficient(static_cast<float>(terrain_friction));
        crg_terrain->SimplifyMesh(true);
        crg_terrain->Initialize(resolved_crg_file);
        terrain = crg_terrain;
        std::cout << "Configured CRGTerrain file: " << resolved_crg_file << std::endl;
        std::cout << "  Road length:    " << crg_terrain->GetLength() << " m" << std::endl;
        std::cout << "  Road width:     " << crg_terrain->GetWidth() << " m" << std::endl;
        auto start_pos = crg_terrain->GetStartPosition();
        std::cout << "  Start position: " << start_pos.pos.x() << ", " << start_pos.pos.y() << ", " << start_pos.pos.z() << std::endl;
        std::cout << "  Start heading:  " << crg_terrain->GetStartHeading() << " rad" << std::endl;
    }
#endif
    else {
        terrain = chrono_types::make_shared<FlatTerrain>(0.0, terrain_friction);
        std::cout << "Configured FlatTerrain with friction: " << terrain_friction << std::endl;
    }

    // Create and initialize the tires
    std::cout << "[DEBUG] Configuring tires..." << std::endl;
    std::string resolved_tire_JSON = tire_JSON;
    std::filesystem::path p_tire(tire_JSON);
    if (p_tire.is_relative()) {
        resolved_tire_JSON = (std::filesystem::path(data_path) / p_tire).string();
    }
    if (!std::filesystem::exists(resolved_tire_JSON)) {
        resolved_tire_JSON = (std::filesystem::path(data_path) / "vehicle/sedan/tire/Sedan_Pac02Tire.json").string();
    }
    std::cout << "[FMU] Attempting to read tire JSON file (absolute): " << std::filesystem::absolute(resolved_tire_JSON).string() << std::endl;
    std::cout << "[FMU] Tire JSON file status: " << (std::filesystem::exists(resolved_tire_JSON) ? "FOUND" : "NOT FOUND") << std::endl;
    tires[0] = ReadTireJSON(resolved_tire_JSON);
    tires[1] = ReadTireJSON(resolved_tire_JSON);
    tires[2] = ReadTireJSON(resolved_tire_JSON);
    tires[3] = ReadTireJSON(resolved_tire_JSON);

    auto coll_type = ChTire::CollisionType::SINGLE_POINT;
    if (tire_coll_type == 1) {
        coll_type = ChTire::CollisionType::FOUR_POINTS;
    } else if (tire_coll_type == 2) {
        coll_type = ChTire::CollisionType::ENVELOPE;
    }
    tires[0]->SetCollisionType(coll_type);
    tires[1]->SetCollisionType(coll_type);
    tires[2]->SetCollisionType(coll_type);
    tires[3]->SetCollisionType(coll_type);

    tires[0]->Initialize(wheel_data[0].wheel);
    tires[1]->Initialize(wheel_data[1].wheel);
    tires[2]->Initialize(wheel_data[2].wheel);
    tires[3]->Initialize(wheel_data[3].wheel);

    tires[0]->SetVisualizationType(VisualizationType::MESH);
    tires[1]->SetVisualizationType(VisualizationType::MESH);
    tires[2]->SetVisualizationType(VisualizationType::MESH);
    tires[3]->SetVisualizationType(VisualizationType::MESH);

    // Associate tires with wheels for visualization and state queries
    wheel_data[0].wheel->SetTire(tires[0]);
    wheel_data[1].wheel->SetTire(tires[1]);
    wheel_data[2].wheel->SetTire(tires[2]);
    wheel_data[3].wheel->SetTire(tires[3]);

    // Resize terrain forces vector
    terrain_forces.resize(4);

    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create parallel active suspension actuators
    // Order: FL (0, LEFT), FR (0, RIGHT), RL (1, LEFT), RR (1, RIGHT)
    int actuator_idx = 0;
    for (int axle = 0; axle < 2; axle++) {
        auto susp = std::dynamic_pointer_cast<ChDoubleWishbone>(vehicle->GetSuspension(axle));
        if (susp) {
            for (int side_idx = 0; side_idx < 2; side_idx++) {
                VehicleSide side = (side_idx == 0) ? LEFT : RIGHT;
                auto spring = susp->GetSpring(side);
                if (spring) {
                    auto actuator = chrono_types::make_shared<ChLinkTSDA>();
                    actuator->Initialize(vehicle->GetChassisBody(), susp->GetAntirollBody(side), true,
                                         spring->GetPoint1Rel(), spring->GetPoint2Rel());
                    actuator->SetSpringCoefficient(0.0);
                    actuator->SetDampingCoefficient(0.0);
                    actuator->SetActuatorForce(0.0);
                    vehicle->GetSystem()->Add(actuator);
                    active_susp_actuators[actuator_idx] = actuator;
                }
                actuator_idx++;
            }
        }
    }
}

void FmuComponent::ConfigureSystem() {
    // Containing system
    auto system = vehicle->GetSystem();

    system->SetGravitationalAcceleration(g_acc);

    // Associate a collision system
    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Modify solver settings if the vehicle model contains bushings
    if (vehicle->HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);
        solver->SetVerbose(false);

        step_size = std::min(step_size, 2e-4);
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
}

void FmuComponent::SynchronizeVehicle(double time) {
    // 1. synchronize the vehicle system (steering, brakes, and flat terrain).
    // Chrono's ChWheeledVehicle::Synchronize will internally update the tires if they are attached to wheels.
    vehicle->Synchronize(time, driver_inputs, *terrain);

    // 2. Apply the 4 independent wheel torques directly to the suspension axle shafts
    // Scale by -1 to match Chrono's shaft convention
    vehicle->GetSuspension(0)->ApplyAxleTorque(LEFT, -torque_FL);
    vehicle->GetSuspension(0)->ApplyAxleTorque(RIGHT, -torque_FR);
    vehicle->GetSuspension(1)->ApplyAxleTorque(LEFT, -torque_RL);
    vehicle->GetSuspension(1)->ApplyAxleTorque(RIGHT, -torque_RR);

    // Apply the 4 active suspension forces to the actuators
    if (active_susp_actuators[0]) active_susp_actuators[0]->SetActuatorForce(act_force_FL);
    if (active_susp_actuators[1]) active_susp_actuators[1]->SetActuatorForce(act_force_FR);
    if (active_susp_actuators[2]) active_susp_actuators[2]->SetActuatorForce(act_force_RL);
    if (active_susp_actuators[3]) active_susp_actuators[3]->SetActuatorForce(act_force_RR);



    // 3. Synchronize the run-time visualization (if available and enabled)
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        vis_sys->Synchronize(time, driver_inputs);
    }
#endif
}

void FmuComponent::CalculateVehicleOutputs() {
    // Extract wheel states
    for (int iw = 0; iw < 4; iw++) {
        if (wheel_data[iw].wheel) {
            wheel_data[iw].state = wheel_data[iw].wheel->GetState();
        }
    }

    // Extract tire forces, slips, and suspension travel/velocity for study KPIs
    if (vehicle && tires[0] && tires[1] && tires[2] && tires[3]) {
        for (int iw = 0; iw < 4; iw++) {
            // Tire contact force (reported in global frame)
            tire_force[iw] = tires[iw]->ReportTireForce(terrain.get()).force;

            // Tire slips
            tire_slip_angle[iw] = tires[iw]->GetSlipAngle();
            tire_slip_ratio[iw] = tires[iw]->GetLongitudinalSlip();

            int axle_id = (iw < 2) ? 0 : 1;
            auto side = (iw % 2 == 0) ? LEFT : RIGHT;
            
            auto suspension = vehicle->GetSuspension(axle_id);
            if (suspension) {
                auto forces = suspension->ReportSuspensionForce(side);
                if (!forces.empty()) {
                    susp_travel[iw] = forces[0].length;
                    susp_velocity[iw] = forces[0].velocity;
                } else {
                    susp_travel[iw] = 0;
                    susp_velocity[iw] = 0;
                }
            }
        }
    }

    // Update the vehicle reference frame
    if (vehicle) {
        ref_frame = vehicle->GetRefFrame();
    }
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {
    std::string filename = "modelDescription.xml";
    std::ifstream infile(filename);
    if (!infile.good()) {
        std::cout << "[FMU postModelDescriptionExport] Warning: " << filename << " not found in current directory." << std::endl;
        return;
    }
    std::stringstream buffer;
    buffer << infile.rdbuf();
    infile.close();

    std::string content = buffer.str();
    size_t pos = content.find("<fmiModelDescription");
    if (pos == std::string::npos) {
        std::cout << "[FMU postModelDescriptionExport] Warning: '<fmiModelDescription' tag not found." << std::endl;
        return;
    }

    // Find the end of the opening tag
    size_t end_tag_pos = content.find(">", pos);
    if (end_tag_pos == std::string::npos) {
        return;
    }

    // Check if description attribute already exists
    if (content.find("description=", pos) == std::string::npos || content.find("description=", pos) > end_tag_pos) {
        // Insert the description attribute right after "<fmiModelDescription"
        std::string insertion = " description=\"Repository: https://github.com/kaanureyen/chrono_fmu\"";
        content.insert(pos + 20, insertion);

        std::ofstream outfile(filename);
        outfile << content;
        outfile.close();
        std::cout << "[FMU postModelDescriptionExport] Injected description into " << filename << std::endl;
    }
}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    // Create the vehicle system
    CreateVehicle();

    // Configure Chrono system
    ConfigureSystem();

    // Initialize runtime visualization (if requested and if available)
#ifdef CHRONO_IRRLICHT
    if (m_visible || fmu_visible) {
        if (!vis_sys) {
            vis_sys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        }
    }
    if (vis_sys) {
        std::cout << " Enable run-time visualization" << std::endl;

        // Set visual driver type if configured
        if (vis_driver == 1) {
            vis_sys->SetDriverType(irr::video::EDT_OPENGL);
            std::cout << "  Using video driver: OpenGL" << std::endl;
        } else if (vis_driver == 2) {
            vis_sys->SetDriverType(irr::video::EDT_DIRECT3D9);
            std::cout << "  Using video driver: Direct3D 9" << std::endl;
        } else if (vis_driver == 3) {
            vis_sys->SetDriverType(irr::video::EDT_SOFTWARE);
            std::cout << "  Using video driver: Software Rasterizer" << std::endl;
        } else if (vis_driver == 4) {
            vis_sys->SetDriverType(irr::video::EDT_BURNINGSVIDEO);
            std::cout << "  Using video driver: Burning's Video Software Rasterizer" << std::endl;
        } else {
            std::cout << "  Using video driver: Default Irrlicht Selection" << std::endl;
        }

        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetJPEGQuality(100);
        vis_sys->SetWindowTitle("Wheeled Vehicle 4-Torque Input FMU (FMI 2.0)");
        vis_sys->SetWindowSize(800, 800);
        vis_sys->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
        vis_sys->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
        vis_sys->AddGrid(0.5, 0.5, 2000, 400, ChCoordsys<>(init_loc, QuatFromAngleZ(init_yaw)), ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
        vis_sys->AttachTerrain(terrain.get());
    }
#endif
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time), std::min(communicationStepSize, step_size));
        vehicle->Advance(h);

#ifdef CHRONO_IRRLICHT
        if (vis_sys) {
            auto status = vis_sys->Run();
            if (!status)
                return fmi2Discard;

            if (m_time >= last_render_time + 1.0 / fps) {
                vis_sys->BeginScene(true, true);
                vis_sys->Render();
                vis_sys->RenderFrame(ref_frame);
                vis_sys->EndScene();

                if (save_img) {
                    std::ostringstream filename;
                    filename << out_path << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".bmp";
                    vis_sys->WriteImageToFile(filename.str());
                    render_frame++;
                }
                last_render_time = m_time;
            }

            vis_sys->Advance(h);
        }
#endif

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}
