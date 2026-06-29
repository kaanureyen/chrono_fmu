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
// =============================================================================
//
// Demo illustrating the co-simulation of the custom Chrono wheeled vehicle FMU
// (FMU2cs_WheeledVehicle4Torques) with 4 independent wheel torque inputs,
// using an open-loop driver controller to perform:
//   - Straight-line acceleration (Phase 1)
//   - Straight-line braking (Phase 2)
//   - Curve driving (Phase 3)
//
// =============================================================================

#include <array>
#include <iostream>
#include <filesystem>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

void CreateVehicleFMU(FmuChronoUnit& vehicle_fmu,
                      const std::string& instance_name,
                      const std::string& fmu_filename,
                      const std::string& fmu_unpack_dir,
                      double step_size,
                      double start_time,
                      double stop_time,
                      const std::vector<std::string>& logCategories,
                      const std::string& out_path,
                      bool visible,
                      double fps) {
    try {
        vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename, fmu_unpack_dir);
    } catch (std::exception&) {
        throw;
    }
    std::cout << "Vehicle FMU version:  " << vehicle_fmu.GetVersion() << std::endl;
    std::cout << "Vehicle FMU platform: " << vehicle_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    try {
        vehicle_fmu.Instantiate(instance_name, false, visible);
    } catch (std::exception&) {
        throw;
    }

    // Set debug logging
    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    vehicle_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                start_time,             // start time
                                fmi2False, stop_time);  // use stop time

    // Set I/O fixed parameters
    vehicle_fmu.SetVariable("out_path", out_path);
    vehicle_fmu.SetVariable("fps", fps, FmuVariable::Type::Real);

    // Set fixed parameters - the FMU will use its internal defaults (packaged Vehicle.json)
    // to ensure it only loads from inside the FMU.
    vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
}

void SynchronizeDriver(double time, FmuChronoUnit& vehicle_fmu, double& steering, double& braking,
                       double& torque_FL, double& torque_FR, double& torque_RL, double& torque_RR) {
    double act_force_FL = 0;
    double act_force_FR = 0;
    double act_force_RL = 0;
    double act_force_RR = 0;

    if (time < 4.0) {
        // Phase 1: Straight-line Acceleration
        steering = 0.0;
        braking = 0.0;
        torque_FL = 350.0;
        torque_FR = 350.0;
        torque_RL = 350.0;
        torque_RR = 350.0;
        static bool print_once = true;
        if (print_once) {
            std::cout << "\n>>> Starting Phase 1: Straight-line Acceleration (0s - 4s)" << std::endl;
            print_once = false;
        }
    } else if (time < 8.0) {
        // Phase 2: Straight-line Braking (with active heave bouncing)
        steering = 0.0;
        braking = 0.7;
        torque_FL = 0.0;
        torque_FR = 0.0;
        torque_RL = 0.0;
        torque_RR = 0.0;

        // Active heave motion (all wheels bounce in phase)
        double amp = 5000.0;
        double freq = 3.0; // Hz
        double force = amp * std::sin(2.0 * 3.141592653589793 * freq * (time - 4.0));
        act_force_FL = force;
        act_force_FR = force;
        act_force_RL = force;
        act_force_RR = force;

        static bool print_once = true;
        if (print_once) {
            std::cout << "\n>>> Starting Phase 2: Straight-line Braking with Active Heave Bouncing (4s - 8s)" << std::endl;
            print_once = false;
        }
    } else {
        // Phase 3: Curve Driving
        steering = 0.15; // Turn left
        braking = 0.0;
        torque_FL = 150.0;
        torque_FR = 150.0;
        torque_RL = 150.0;
        torque_RR = 150.0;
        static bool print_once = true;
        if (print_once) {
            std::cout << "\n>>> Starting Phase 3: Curve Driving (8s - 12s)" << std::endl;
            print_once = false;
        }
    }

    // Set input variables on the vehicle FMU
    vehicle_fmu.SetVariable("steering", steering, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("braking", braking, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("torque_FL", torque_FL, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("torque_FR", torque_FR, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("torque_RL", torque_RL, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("torque_RR", torque_RR, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("act_force_FL", act_force_FL, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("act_force_FR", act_force_FR, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("act_force_RL", act_force_RL, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("act_force_RR", act_force_RR, FmuVariable::Type::Real);
}

// Helper function to run a simulation for a specific tire model and terrain
void RunSimulation(const std::string& vehicle_fmu_filename, const std::string& vehicle_unpack_dir,
                   const std::string& tire_JSON_rel, const std::string& out_file,
                   double step_size, double start_time, double stop_time, double fps, bool visible,
                   int terrain_type = 0, const std::string& terrain_mesh_file = "", const std::string& terrain_crg_file = "",
                   const ChVector3d& init_loc = ChVector3d(0, 0, 0.2)) {
    FmuChronoUnit vehicle_fmu;
    std::vector<std::string> logCategories = {"logAll"};

    try {
        CreateVehicleFMU(vehicle_fmu, "WheeledVehicle4TorquesFMU", vehicle_fmu_filename, vehicle_unpack_dir,
                         step_size, start_time, stop_time, logCategories, ".", visible, fps);
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return;
    }

    // Verify unpacked resources directory and required files
    std::filesystem::path resources_path = std::filesystem::path(vehicle_unpack_dir) / "resources";
    if (!std::filesystem::exists(resources_path / "Vehicle.json")) {
        std::cerr << "ERROR: Vehicle config file (Vehicle.json) is missing inside the FMU resources!" << std::endl;
        return;
    }

    std::vector<std::string> required_jsons = {
        "Chassis.json",
        "DoubleWishboneFront.json",
        "DoubleWishboneRear.json",
        "Wheel.json",
        "BrakeSimple_Front.json",
        "BrakeSimple_Rear.json",
        "RackPinion.json",
        "Driveline2WD.json"
    };
    for (const auto& json_file : required_jsons) {
        if (!std::filesystem::exists(resources_path / json_file)) {
            std::cerr << "ERROR: Subsystem config file (" << json_file << ") is missing inside the FMU resources!" << std::endl;
            return;
        }
    }

    if (!std::filesystem::exists(resources_path / tire_JSON_rel)) {
        std::cerr << "ERROR: Tire model file (" << tire_JSON_rel << ") is missing inside the FMU resources!" << std::endl;
        return;
    }

    if (terrain_type == 1 && !terrain_mesh_file.empty()) {
        if (!std::filesystem::exists(resources_path / terrain_mesh_file)) {
            std::cerr << "ERROR: Terrain mesh file (" << terrain_mesh_file << ") is missing inside the FMU resources!" << std::endl;
            return;
        }
    }

    if (terrain_type == 2 && !terrain_crg_file.empty()) {
        if (!std::filesystem::exists(resources_path / terrain_crg_file)) {
            std::cerr << "ERROR: Terrain CRG file (" << terrain_crg_file << ") is missing inside the FMU resources!" << std::endl;
            return;
        }
    }

    // Initialize FMU
    vehicle_fmu.EnterInitializationMode();
    {
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", 0.0, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_roll", 0.0, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_pitch", 0.0, FmuVariable::Type::Real);

        // Pass relative path directly; the FMU will resolve it internally relative to its resources directory
        std::cout << "Configuring tire JSON: " << tire_JSON_rel << std::endl;
        vehicle_fmu.SetVariable("tire_JSON", tire_JSON_rel);

        // Set terrain parameters
        vehicle_fmu.SetVariable("terrain_type", terrain_type, FmuVariable::Type::Integer);
        if (terrain_type == 1) {
            std::cout << "Configuring terrain mesh file: " << terrain_mesh_file << std::endl;
            vehicle_fmu.SetVariable("terrain_mesh_file", terrain_mesh_file);
        } else if (terrain_type == 2) {
            std::cout << "Configuring terrain CRG file: " << terrain_crg_file << std::endl;
            vehicle_fmu.SetVariable("terrain_crg_file", terrain_crg_file);
        }
    }
    vehicle_fmu.ExitInitializationMode();

    vehicle_fmu.SetVariable("save_img", false);

    ChWriterCSV csv;
    csv.SetDelimiter(" ");

    double time = 0;
    double steering, braking, torque_FL, torque_FR, torque_RL, torque_RR;
    chrono::ChFrameMoving<> ref_frame;

    std::cout << "\n--- Starting simulation run for tire: " << tire_JSON_rel << " ---" << std::endl;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        // Update driver inputs
        SynchronizeDriver(time, vehicle_fmu, steering, braking, torque_FL, torque_FR, torque_RL, torque_RR);

        // Fetch vehicle reference frame to record trajectory
        vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);
        csv << time << ref_frame.GetPos() << ref_frame.GetRot().GetCardanAnglesXYZ() << std::endl;

        // Advance FMU
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        if (status_vehicle == fmi2Discard)
            break;

        time += step_size;
    }

    timer.stop();
    std::cout << "Simulation completed in " << timer() << "s for " << time << "s of virtual time." << std::endl;

    csv.WriteToFile(out_file);
    std::cout << "Output saved to: " << out_file << std::endl;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << std::filesystem::path(argv[0]).filename() << std::endl;
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n" << std::endl;

    std::string vehicle_fmu_model_identifier = "FMU2cs_WheeledVehicle4Torques";
    std::string vehicle_fmu_filename;
    bool vehicle_visible = true;

    // Check arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            vehicle_visible = false;
        } else if (arg == "--visible") {
            vehicle_visible = true;
        } else if (arg[0] != '-') {
            vehicle_fmu_filename = arg;
        }
    }

    if (vehicle_fmu_filename.empty()) {
        // First check if the FMU file exists in the current working directory
        std::string local_fmu = vehicle_fmu_model_identifier + ".fmu";
        if (std::filesystem::exists(local_fmu)) {
            vehicle_fmu_filename = local_fmu;
        } else {
            // Check if the FMU file exists in the executable's directory or the parent build directory
            std::filesystem::path exe_dir = std::filesystem::absolute(argv[0]).parent_path();
            std::filesystem::path local_fmu_exe = exe_dir / (vehicle_fmu_model_identifier + ".fmu");
            std::filesystem::path parent_fmu = exe_dir.parent_path().parent_path() / vehicle_fmu_model_identifier / (vehicle_fmu_model_identifier + ".fmu");
            if (std::filesystem::exists(local_fmu_exe)) {
                vehicle_fmu_filename = local_fmu_exe.string();
            } else if (std::filesystem::exists(parent_fmu)) {
                vehicle_fmu_filename = parent_fmu.string();
            } else {
                std::cerr << "ERROR: FMU file " << local_fmu << " not found in current directory ("
                          << std::filesystem::current_path().string() << "), executable directory ("
                          << exe_dir.string() << "), or build directory ("
                          << parent_fmu.parent_path().string() << ")." << std::endl;
                std::cerr << "Please build the FMUs first or provide the correct path to the FMU file." << std::endl;
                std::cerr << "Usage: ./demo_VEH_FMI2_WheeledVehicle_4torques [vehicle_FMU_filename] [--headless]" << std::endl;
                return 1;
            }
        }
    }

    if (!std::filesystem::exists(vehicle_fmu_filename)) {
        std::cerr << "ERROR: Specified FMU file does not exist: " << vehicle_fmu_filename << std::endl;
        return 1;
    }

    // FMU unpack directory (portable relative to build directory)
    std::filesystem::path exe_dir = std::filesystem::absolute(argv[0]).parent_path();
    std::filesystem::path build_dir = exe_dir.parent_path().parent_path();
    std::string vehicle_unpack_dir = (build_dir / "tmp_unpack_vehicle_4torques").string() + "/";

    // Create output directory inside the build directory
    std::string out_dir = (build_dir / "DEMO_OUTPUT" / "DEMO_WHEELEDVEHICLE_FMI_COSIM_4TORQUES").string();
    if (!std::filesystem::exists(out_dir) && !std::filesystem::create_directories(out_dir)) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Set simulation parameters
    double start_time = 0;
    double stop_time = 12; // 4s acc, 4s brake, 4s curve
    double step_size = 1e-3;
    double fps = 60;

    // Run 1: Pacejka Tire (Default)
    std::string out_file_pacejka = out_dir + "/vehicle_4torques_pacejka.out";
    RunSimulation(vehicle_fmu_filename, vehicle_unpack_dir,
                  "vehicle/sedan/tire/Sedan_Pac02Tire.json", out_file_pacejka,
                  step_size, start_time, stop_time, fps, vehicle_visible);

    // Run 2: TMeasy Tire
    std::string out_file_tmeasy = out_dir + "/vehicle_4torques_tmeasy.out";
    RunSimulation(vehicle_fmu_filename, vehicle_unpack_dir,
                  "vehicle/sedan/tire/Sedan_TMeasyTire.json", out_file_tmeasy,
                  step_size, start_time, stop_time, fps, vehicle_visible);

    // Run 3: Pacejka Tire with generated .crg Terrain
    std::string out_file_crg = out_dir + "/vehicle_4torques_crg.out";
    RunSimulation(vehicle_fmu_filename, vehicle_unpack_dir,
                  "vehicle/sedan/tire/Sedan_Pac02Tire.json", out_file_crg,
                  step_size, start_time, stop_time, fps, vehicle_visible,
                  2, "", "default_road.crg");

    std::cout << "\nAll runs completed (Flat, CRG). Out files placed in: " << out_dir << std::endl;
    return 0;
}
