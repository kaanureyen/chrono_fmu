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

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef CHRONO_HAS_OPENCRG
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/assets/ChVisualMaterial.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

#ifdef CHRONO_IRRLICHT
#include <irrlicht.h>
#include "chrono_irrlicht/ChIrrTools.h"

class FmuComponent::FMUVisualEventReceiver : public irr::IEventReceiver {
public:
    enum class DisplayMode {
        TIRE_FORCES = 0,
        TIRE_SLIPS = 1,
        WHEEL_SPEEDS_TORQUES = 2,
        ALL = 3,
        NONE = 4
    };
    DisplayMode mode = DisplayMode::TIRE_FORCES;

    virtual bool OnEvent(const irr::SEvent& event) override {
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            if (event.KeyInput.Key == irr::KEY_KEY_I) { // Press 'I' to cycle info modes
                mode = static_cast<DisplayMode>((static_cast<int>(mode) + 1) % 5);
                return true;
            }
        }
        return false;
    }
};
#endif

class VehicleFrictionFunctor : public ChTerrain::FrictionFunctor {
  public:
    VehicleFrictionFunctor(FmuComponent* fmu) : m_fmu(fmu) {}
    
    virtual float operator()(const ChVector3d& loc) override {
        if (!m_fmu || !m_fmu->vehicle)
            return 0.8f;
            
        // Get wheel spindle positions
        auto pos_FL = m_fmu->vehicle->GetSpindlePos(0, LEFT);
        auto pos_FR = m_fmu->vehicle->GetSpindlePos(0, RIGHT);
        auto pos_RL = m_fmu->vehicle->GetSpindlePos(1, LEFT);
        auto pos_RR = m_fmu->vehicle->GetSpindlePos(1, RIGHT);
        
        // Calculate distances squared in x-y plane
        double d_FL = (loc.x() - pos_FL.x()) * (loc.x() - pos_FL.x()) + (loc.y() - pos_FL.y()) * (loc.y() - pos_FL.y());
        double d_FR = (loc.x() - pos_FR.x()) * (loc.x() - pos_FR.x()) + (loc.y() - pos_FR.y()) * (loc.y() - pos_FR.y());
        double d_RL = (loc.x() - pos_RL.x()) * (loc.x() - pos_RL.x()) + (loc.y() - pos_RL.y()) * (loc.y() - pos_RL.y());
        double d_RR = (loc.x() - pos_RR.x()) * (loc.x() - pos_RR.x()) + (loc.y() - pos_RR.y()) * (loc.y() - pos_RR.y());
        
        // Find the closest spindle
        double d_min = d_FL;
        float friction = (float)m_fmu->friction_FL;
        
        if (d_FR < d_min) {
            d_min = d_FR;
            friction = (float)m_fmu->friction_FR;
        }
        if (d_RL < d_min) {
            d_min = d_RL;
            friction = (float)m_fmu->friction_RL;
        }
        if (d_RR < d_min) {
            d_min = d_RR;
            friction = (float)m_fmu->friction_RR;
        }
        
        return friction;
    }
    
  private:
    FmuComponent* m_fmu;
};

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
    init_roll = 0;
    init_pitch = 0;
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
    reset = false;

    // Get default JSON files (relative to FMU resources at runtime)
    resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    data_path = "";
    vehicle_JSON = "Vehicle.json";
    tire_JSON = "vehicle/sedan/tire/Sedan_Pac02Tire.json";
    terrain_type = 2; // Default to OpenCRG
    tire_coll_type = 2; // Default to 2D Profile Envelope
    terrain_mesh_file = "";
    terrain_crg_file = "default_road.crg";
    terrain_crg_simplify = true;
    terrain_diffuse_texture = "";
    terrain_normal_texture = "";
    terrain_show_visual_lines = false;
    terrain_friction = 0.8;

    friction_FL = 0.8;
    friction_FR = 0.8;
    friction_RL = 0.8;
    friction_RR = 0.8;

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

    AddFmuVariable(&terrain_crg_simplify, "terrain_crg_simplify", FmuVariable::Type::Boolean, "1", "simplify OpenCRG visualization mesh", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //
    AddFmuVariable(&terrain_diffuse_texture, "terrain_diffuse_texture", FmuVariable::Type::String, "1", "diffuse texture file for CRG terrain", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //
    AddFmuVariable(&terrain_normal_texture, "terrain_normal_texture", FmuVariable::Type::String, "1", "normal texture file for CRG terrain", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //
    AddFmuVariable(&terrain_show_visual_lines, "terrain_show_visual_lines", FmuVariable::Type::Boolean, "1", "show lane lines on CRG terrain", //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                            //

    AddFmuVariable(&terrain_friction, "terrain_friction", FmuVariable::Type::Real, "1", "terrain friction",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);    //

    AddFmuVariable(&system_SMC, "system_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVecVariable(init_loc, "init_loc", "m", "initial location",                                //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial yaw",            //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //
    AddFmuVariable(&init_roll, "init_roll", FmuVariable::Type::Real, "rad", "initial roll",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //
    AddFmuVariable(&init_pitch, "init_pitch", FmuVariable::Type::Real, "rad", "initial pitch",      //
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

    AddFmuVariable(&friction_FL, "friction_FL", FmuVariable::Type::Real, "1", "Front Left wheel friction coefficient", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&friction_FR, "friction_FR", FmuVariable::Type::Real, "1", "Front Right wheel friction coefficient", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&friction_RL, "friction_RL", FmuVariable::Type::Real, "1", "Rear Left wheel friction coefficient", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //
    AddFmuVariable(&friction_RR, "friction_RR", FmuVariable::Type::Real, "1", "Rear Right wheel friction coefficient", //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);               //


    // Set DISCRETE INPUTS for this FMU (I/O) [rebuilt]
    AddFmuVariable(&save_img, "save_img", FmuVariable::Type::Boolean, "1", "trigger saving images",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::discrete);               //
    AddFmuVariable(&reset, "reset", FmuVariable::Type::Boolean, "1", "reset simulation state",              //
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
    DeclareVariableDependencies("ref_frame", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(prefix + ".pos", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
        DeclareVariableDependencies(prefix + ".rot", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
        DeclareVariableDependencies(prefix + ".force", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
        DeclareVariableDependencies(prefix + ".slip_angle", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
        DeclareVariableDependencies(prefix + ".slip_ratio", {"init_loc", "init_yaw", "init_roll", "init_pitch"});

        std::string susp_prefix = "susp_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(susp_prefix + ".travel", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
        DeclareVariableDependencies(susp_prefix + ".velocity", {"init_loc", "init_yaw", "init_roll", "init_pitch"});
    }

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateVehicleOutputs(); });
}

FmuComponent::~FmuComponent() {
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        auto device = vis_sys->GetDevice();
        if (device) {
            device->closeDevice();
        }
        vis_sys.reset();
    }
#ifdef _WIN32
    MSG msg;
    while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
#endif
#endif
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
    std::cout << " Initial roll:      " << init_roll << std::endl;
    std::cout << " Initial pitch:     " << init_pitch << std::endl;
    std::cout << " Initial velocity:  " << init_vel << std::endl;

    std::cout << "[FMU] Current working directory: " << std::filesystem::current_path().string() << std::endl;
    std::cout << "[FMU] Vehicle JSON file status: " << (std::filesystem::exists(resolved_vehicle_JSON) ? "FOUND" : "NOT FOUND") << std::endl;

    std::cout << "[DEBUG] Creating vehicle..." << std::endl;
    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(resolved_vehicle_JSON, system_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    std::cout << "[DEBUG] Vehicle created. Initializing..." << std::endl;
    
    ChQuaterniond init_rot = QuatFromAngleZ(init_yaw) * QuatFromAngleY(init_pitch) * QuatFromAngleX(init_roll);
    ChVector3d init_offset = init_rot.Rotate(ChVector3d(0, 0, 0.5));
    vehicle->Initialize(ChCoordsys<>(init_loc + init_offset, init_rot), init_vel);
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
        auto patch = rigid_terrain->AddPatch(material, ChCoordsys<>(), resolved_mesh_file, false, 0.0, true);
        if (!terrain_diffuse_texture.empty()) {
            std::filesystem::path tex_path(terrain_diffuse_texture);
            std::string resolved_tex = terrain_diffuse_texture;
            if (tex_path.is_relative()) {
                resolved_tex = (std::filesystem::path(data_path) / tex_path).generic_string();
            } else {
                resolved_tex = tex_path.generic_string();
            }
            patch->SetTexture(resolved_tex, 1.0f, 1.0f);
            std::cout << "[FMU] Applied diffuse texture to RigidTerrain patch: " << resolved_tex << std::endl;
        }
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
        crg_terrain->SimplifyMesh(terrain_crg_simplify);
        
        // Resolve and load textures if configured (passing relative filenames directly to avoid double prepending)
        if (!terrain_diffuse_texture.empty()) {
            std::filesystem::path tex_path(terrain_diffuse_texture);
            std::string tex_to_set = tex_path.is_relative() ? terrain_diffuse_texture : tex_path.filename().string();
            crg_terrain->SetRoadDiffuseTextureFile(tex_to_set);
            std::cout << "[FMU] Applied diffuse road texture: " << tex_to_set << std::endl;
        }
        if (!terrain_normal_texture.empty()) {
            std::filesystem::path tex_path(terrain_normal_texture);
            std::string tex_to_set = tex_path.is_relative() ? terrain_normal_texture : tex_path.filename().string();
            crg_terrain->SetRoadNormalTextureFile(tex_to_set);
            std::cout << "[FMU] Applied normal road texture: " << tex_to_set << std::endl;
        }
        
        crg_terrain->Initialize(resolved_crg_file);
        terrain = crg_terrain;
        
        // Draw visual centerline and boundary curves to represent road lanes
        if (terrain_show_visual_lines) {
            auto ground = crg_terrain->GetGround();
            if (ground) {
                auto center_mat = chrono_types::make_shared<ChVisualMaterial>();
                center_mat->SetDiffuseColor({1.0f, 1.0f, 1.0f}); // White
                
                auto side_mat = chrono_types::make_shared<ChVisualMaterial>();
                side_mat->SetDiffuseColor({0.8f, 0.8f, 0.0f}); // Yellow
                
                auto center_curve = crg_terrain->GetRoadCenterLine();
                auto left_curve = crg_terrain->GetRoadBoundaryLeft();
                auto right_curve = crg_terrain->GetRoadBoundaryRight();
                
                if (center_curve && left_curve && right_curve) {
                    std::vector<ChVector3d> pts_center = center_curve->GetPoints();
                    std::vector<ChVector3d> pts_left = left_curve->GetPoints();
                    std::vector<ChVector3d> pts_right = right_curve->GetPoints();
                    
                    size_t np = pts_center.size();
                    if (pts_left.size() == np && pts_right.size() == np) {
                        for (size_t i = 0; i < np; i++) {
                            // Offset Z slightly to avoid z-fighting with the road surface
                            pts_center[i].z() += 0.015;
                            pts_left[i].z() += 0.015;
                            pts_right[i].z() += 0.015;
                        }
                        
                        // Helper to create and add a visual line shape to the ground
                        auto add_visual_line = [&](const std::vector<ChVector3d>& pts, std::shared_ptr<ChVisualMaterial> mat) {
                            auto curve = chrono_types::make_shared<ChBezierCurve>(pts);
                            auto line = chrono_types::make_shared<ChLineBezier>(curve);
                            auto shape = chrono_types::make_shared<ChVisualShapeLine>();
                            shape->SetLineGeometry(line);
                            shape->SetNumRenderPoints(std::max<unsigned int>(static_cast<unsigned int>(3 * pts.size()), 400));
                            shape->AddMaterial(mat);
                            ground->AddVisualShape(shape);
                        };
                        
                        // 1. Road Centerline (White - Center of the Middle Lane)
                        add_visual_line(pts_center, center_mat);
                        
                        // 2. Left Road Boundary (Yellow)
                        add_visual_line(pts_left, side_mat);
                        
                        // 3. Right Road Boundary (Yellow)
                        add_visual_line(pts_right, side_mat);
                        
                        std::cout << "[FMU] Visual lane lines configured (white centerline, yellow boundaries)." << std::endl;
                    }
                }
            }
        }
        
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

    // Register custom wheel-specific friction functor
    friction_functor = chrono_types::make_shared<VehicleFrictionFunctor>(this);
    terrain->RegisterFrictionFunctor(friction_functor);

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
            // Transform wheel linear and angular velocities to vehicle chassis local frame for easy Simulink analysis
            if (vehicle && vehicle->GetChassisBody()) {
                wheel_data[iw].state.lin_vel = vehicle->GetChassisBody()->TransformDirectionParentToLocal(wheel_data[iw].state.lin_vel);
                wheel_data[iw].state.ang_vel = vehicle->GetChassisBody()->TransformDirectionParentToLocal(wheel_data[iw].state.ang_vel);
            }
        }
    }

    // Extract tire forces, slips, and suspension travel/velocity for study KPIs
    if (vehicle && tires[0] && tires[1] && tires[2] && tires[3] && vehicle->GetChassisBody()) {
        for (int iw = 0; iw < 4; iw++) {
            // Tire contact force (transformed from global to vehicle chassis local frame)
            chrono::ChVector3d force_global = tires[iw]->ReportTireForce(terrain.get()).force;
            tire_force[iw] = vehicle->GetChassisBody()->TransformDirectionParentToLocal(force_global);

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

        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetJPEGQuality(100);
        vis_sys->SetWindowTitle("Wheeled Vehicle 4-Torque Input FMU (FMI 2.0)");
        vis_sys->SetWindowSize(800, 800);
        vis_sys->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
        vis_sys->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
        ChQuaterniond init_rot = QuatFromAngleZ(init_yaw) * QuatFromAngleY(init_pitch) * QuatFromAngleX(init_roll);
        vis_sys->AddGrid(0.5, 0.5, 2000, 400, ChCoordsys<>(init_loc, init_rot), ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
        vis_sys->AttachTerrain(terrain.get());

        // Create and register custom visual event receiver
        visual_receiver = std::make_shared<FMUVisualEventReceiver>();
        vis_sys->AddUserEventReceiver(visual_receiver.get());
    }
#endif
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    if (reset) {
        reset = fmi2False;
        m_time = currentCommunicationPoint;
        vehicle->GetSystem()->SetChTime(currentCommunicationPoint);
        ChQuaterniond init_rot = QuatFromAngleZ(init_yaw) * QuatFromAngleY(init_pitch) * QuatFromAngleX(init_roll);
        ChVector3d init_offset = init_rot.Rotate(ChVector3d(0, 0, 0.5));
        vehicle->Initialize(ChCoordsys<>(init_loc + init_offset, init_rot), init_vel);
        for (int i = 0; i < 4; i++) {
            tires[i]->Initialize(wheel_data[i].wheel);
        }
#ifdef CHRONO_IRRLICHT
        render_frame = 0;
        last_render_time = currentCommunicationPoint;
#endif
    }

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

                DrawCustomTelemetry();

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

#ifdef CHRONO_IRRLICHT
void FmuComponent::DrawCustomTelemetry() {
    if (!vis_sys || !visual_receiver) return;

    auto mode = visual_receiver->mode;
    if (mode == FMUVisualEventReceiver::DisplayMode::NONE) return;

    // Force outputs update before drawing to ensure HUD data is fresh
    CalculateVehicleOutputs();

    // 1. Draw 3D tire forces (normal force in Blue, lateral/longitudinal in Red)
    if (mode == FMUVisualEventReceiver::DisplayMode::TIRE_FORCES || mode == FMUVisualEventReceiver::DisplayMode::ALL) {
        for (int i = 0; i < 4; i++) {
            if (wheel_data[i].wheel) {
                auto spindle_pos = wheel_data[i].wheel->GetSpindle()->GetPos();
                
                // Chassis local axes in world coordinates (non-spinning, follows chassis plane)
                auto chassis_rot = vehicle->GetChassisBody()->GetRot();
                ChVector3d c_fwd = chassis_rot.Rotate(ChVector3d(1, 0, 0)); // chassis forward
                ChVector3d c_lat = chassis_rot.Rotate(ChVector3d(0, 1, 0)); // chassis lateral
                ChVector3d c_up  = chassis_rot.Rotate(ChVector3d(0, 0, 1)); // chassis vertical

                // Retrieve pre-calculated local forces in the chassis plane
                double Fx = tire_force[i].x();
                double Fy = tire_force[i].y();
                double Fz = tire_force[i].z();
                
                // Normal force vector points along chassis vertical axis (Blue)
                ChVector3d norm_end = spindle_pos + c_up * (Fz * 0.0002);
                chrono::irrlicht::tools::drawArrow(
                    vis_sys.get(),
                    spindle_pos,
                    norm_end,
                    c_lat,
                    false,
                    chrono::ChColor(0.f, 0.4f, 1.f),
                    false
                );
                
                // In-plane contact force vector points along chassis local horizontal plane (Red)
                ChVector3d plane_force_vec = c_fwd * Fx + c_lat * Fy;
                ChVector3d plane_end = spindle_pos + plane_force_vec * 0.0002;
                chrono::irrlicht::tools::drawArrow(
                    vis_sys.get(),
                    spindle_pos,
                    plane_end,
                    c_up,
                    false,
                    chrono::ChColor(1.f, 0.f, 0.f),
                    false
                );
            }
        }
    }

    // 2. Draw 2D HUD text overlay
    irr::gui::IGUIFont* font = vis_sys->GetMonospaceFont();
    if (!font) return;

    auto screen_size = vis_sys->GetVideoDriver()->getScreenSize();
    int left = 15;
    int top = screen_size.Height - 165;
    int width = 350;
    int height = 145;

    // Draw background panel
    vis_sys->GetVideoDriver()->draw2DRectangle(
        irr::video::SColor(180, 20, 20, 25),
        irr::core::rect<irr::s32>(left, top, left + width, top + height)
    );

    // Draw Mode title
    std::string mode_str = "";
    if (mode == FMUVisualEventReceiver::DisplayMode::TIRE_FORCES) mode_str = "Mode: Tire Forces (Press 'I' to toggle)";
    else if (mode == FMUVisualEventReceiver::DisplayMode::TIRE_SLIPS) mode_str = "Mode: Tire Slips (Press 'I' to toggle)";
    else if (mode == FMUVisualEventReceiver::DisplayMode::WHEEL_SPEEDS_TORQUES) mode_str = "Mode: Wheel Speeds & Torques (Press 'I' to toggle)";
    else if (mode == FMUVisualEventReceiver::DisplayMode::ALL) mode_str = "Mode: All Telemetry (Press 'I' to toggle)";
    
    font->draw(mode_str.c_str(), irr::core::rect<irr::s32>(left + 10, top + 8, left + width - 10, top + 25), irr::video::SColor(255, 255, 255, 255));

    char buf[128];
    int line_y = top + 30;
    const std::array<std::string, 4> wheel_names = {"FL", "FR", "RL", "RR"};

    if (mode == FMUVisualEventReceiver::DisplayMode::TIRE_FORCES || mode == FMUVisualEventReceiver::DisplayMode::ALL) {
        for (int i = 0; i < 4; i++) {
            auto f = tire_force[i]; // already resolved in vehicle chassis local frame
            snprintf(buf, sizeof(buf), "%s Force: Fx=%+5.0f N, Fy=%+5.0f N, Fz=%+5.0f N", wheel_names[i].c_str(), f.x(), f.y(), f.z());
            font->draw(buf, irr::core::rect<irr::s32>(left + 10, line_y, left + width - 10, line_y + 18), irr::video::SColor(255, 250, 150, 50));
            line_y += 16;
        }
    }
    if (mode == FMUVisualEventReceiver::DisplayMode::TIRE_SLIPS) {
        for (int i = 0; i < 4; i++) {
            double slip_angle_deg = tire_slip_angle[i] * 180.0 / CH_PI;
            double slip_ratio_pct = tire_slip_ratio[i] * 100.0;
            snprintf(buf, sizeof(buf), "%s Slip: Angle=%+6.2f deg, Ratio=%+6.1f %%", wheel_names[i].c_str(), slip_angle_deg, slip_ratio_pct);
            font->draw(buf, irr::core::rect<irr::s32>(left + 10, line_y, left + width - 10, line_y + 18), irr::video::SColor(255, 50, 200, 250));
            line_y += 16;
        }
    }
    if (mode == FMUVisualEventReceiver::DisplayMode::WHEEL_SPEEDS_TORQUES) {
        std::array<double, 4> torques = {torque_FL, torque_FR, torque_RL, torque_RR};
        for (int i = 0; i < 4; i++) {
            double spin_speed = wheel_data[i].wheel ? wheel_data[i].state.omega : 0.0;
            snprintf(buf, sizeof(buf), "%s Wheel: Spin=%+6.1f rad/s, Torque=%+6.1f Nm", wheel_names[i].c_str(), spin_speed, torques[i]);
            font->draw(buf, irr::core::rect<irr::s32>(left + 10, line_y, left + width - 10, line_y + 18), irr::video::SColor(255, 100, 250, 100));
            line_y += 16;
        }
    }
    if (mode == FMUVisualEventReceiver::DisplayMode::ALL) {
        std::array<double, 4> torques = {torque_FL, torque_FR, torque_RL, torque_RR};
        snprintf(buf, sizeof(buf), "Torques: FL=%+.0f FR=%+.0f RL=%+.0f RR=%+.0f", torques[0], torques[1], torques[2], torques[3]);
        font->draw(buf, irr::core::rect<irr::s32>(left + 10, line_y, left + width - 10, line_y + 18), irr::video::SColor(255, 255, 255, 255));
        line_y += 16;
        
        snprintf(buf, sizeof(buf), "Slips %%: FL=%+5.1f FR=%+5.1f RL=%+5.1f RR=%+5.1f", tire_slip_ratio[0]*100, tire_slip_ratio[1]*100, tire_slip_ratio[2]*100, tire_slip_ratio[3]*100);
        font->draw(buf, irr::core::rect<irr::s32>(left + 10, line_y, left + width - 10, line_y + 18), irr::video::SColor(255, 255, 255, 255));
    }
}
#endif
