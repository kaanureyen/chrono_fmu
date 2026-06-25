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
// Authors: Antigravity AI
// =============================================================================
//
// Demo illustrating the co-simulation of the custom Chrono wheeled vehicle FMU
// and path follower driver FMU performing a lane change on CRG terrain at 80 kph.
// Supports command-line arguments for parameter sweep / tuning.
//
// =============================================================================

#include <array>
#include <iostream>
#include <filesystem>
#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::fmi2;

int main(int argc, char* argv[]) {
    std::cout << "Chrono FMI2 Co-Simulation - Sedan Lane Change Demo\n" << std::endl;

    // Command-line parameters with defaults
    bool visible = true;
    double Kp_steering = 1.0;
    double look_ahead_dist = 6.0;
    double Kp_speed = 0.5;
    double Ki_speed = 0.1;
    double max_torque = 350.0;
    double init_vel = 22.2222; // 80 kph
    std::string out_file = "lane_change_trajectory.csv";
    int tire_coll_type = 0;    // Default: 0 (Single-point contact)
    double stop_time = 10.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            visible = false;
        } else if (arg == "--visible") {
            visible = true;
        } else if (arg == "--Kp_steering" && i + 1 < argc) {
            Kp_steering = std::stod(argv[++i]);
        } else if (arg == "--look_ahead_dist" && i + 1 < argc) {
            look_ahead_dist = std::stod(argv[++i]);
        } else if (arg == "--Kp_speed" && i + 1 < argc) {
            Kp_speed = std::stod(argv[++i]);
        } else if (arg == "--Ki_speed" && i + 1 < argc) {
            Ki_speed = std::stod(argv[++i]);
        } else if (arg == "--max_torque" && i + 1 < argc) {
            max_torque = std::stod(argv[++i]);
        } else if (arg == "--init_vel" && i + 1 < argc) {
            init_vel = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            out_file = argv[++i];
        } else if (arg == "--tire_coll_type" && i + 1 < argc) {
            tire_coll_type = std::stoi(argv[++i]);
        } else if (arg == "--tend" && i + 1 < argc) {
            stop_time = std::stod(argv[++i]);
        }
    }

    std::cout << "Parameters:" << std::endl;
    std::cout << "  visible:         " << (visible ? "true" : "false") << std::endl;
    std::cout << "  Kp_steering:     " << Kp_steering << std::endl;
    std::cout << "  look_ahead_dist: " << look_ahead_dist << std::endl;
    std::cout << "  Kp_speed:        " << Kp_speed << std::endl;
    std::cout << "  Ki_speed:        " << Ki_speed << std::endl;
    std::cout << "  max_torque:      " << max_torque << std::endl;
    std::cout << "  init_vel:        " << init_vel << std::endl;
    std::cout << "  out_file:        " << out_file << std::endl;
    std::cout << "  tire_coll_type:  " << tire_coll_type << std::endl;
    std::cout << "  stop_time:       " << stop_time << std::endl;

    std::string vehicle_fmu_filename = "FMU2cs_WheeledVehicle4Torques.fmu";
    std::string driver_fmu_filename = "FMU2cs_PathFollowerDriver.fmu";

    // Locate FMUs
    std::filesystem::path exe_dir = std::filesystem::path(argv[0]).parent_path();
    if (!std::filesystem::exists(vehicle_fmu_filename)) {
        std::filesystem::path p = exe_dir / "FMU2cs_WheeledVehicle4Torques" / "FMU2cs_WheeledVehicle4Torques.fmu";
        if (std::filesystem::exists(p)) {
            vehicle_fmu_filename = p.string();
        } else {
            p = exe_dir.parent_path() / "FMU2cs_WheeledVehicle4Torques" / "FMU2cs_WheeledVehicle4Torques.fmu";
            if (std::filesystem::exists(p)) {
                vehicle_fmu_filename = p.string();
            } else {
                p = exe_dir / "FMU2cs_WheeledVehicle4Torques.fmu";
                if (std::filesystem::exists(p)) {
                    vehicle_fmu_filename = p.string();
                }
            }
        }
    }
    if (!std::filesystem::exists(driver_fmu_filename)) {
        std::filesystem::path p = exe_dir / "FMU2cs_PathFollowerDriver" / "FMU2cs_PathFollowerDriver.fmu";
        if (std::filesystem::exists(p)) {
            driver_fmu_filename = p.string();
        } else {
            p = exe_dir.parent_path() / "FMU2cs_PathFollowerDriver" / "FMU2cs_PathFollowerDriver.fmu";
            if (std::filesystem::exists(p)) {
                driver_fmu_filename = p.string();
            } else {
                p = exe_dir / "FMU2cs_PathFollowerDriver.fmu";
                if (std::filesystem::exists(p)) {
                    driver_fmu_filename = p.string();
                }
            }
        }
    }

    std::cout << "Vehicle FMU path: " << vehicle_fmu_filename << std::endl;
    std::cout << "Driver FMU path:  " << driver_fmu_filename << std::endl;

    FmuChronoUnit vehicle_fmu;
    FmuChronoUnit driver_fmu;
    std::vector<std::string> logCategories = {"logAll"};

    std::string vehicle_unpack_dir = std::filesystem::absolute("./tmp_unpack_vehicle_lc/").string();
    std::string driver_unpack_dir = std::filesystem::absolute("./tmp_unpack_driver_lc/").string();
    vehicle_fmu_filename = std::filesystem::absolute(vehicle_fmu_filename).string();
    driver_fmu_filename = std::filesystem::absolute(driver_fmu_filename).string();

    try {
        vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, vehicle_fmu_filename, vehicle_unpack_dir);
        driver_fmu.Load(fmi2Type::fmi2CoSimulation, driver_fmu_filename, driver_unpack_dir);
    } catch (std::exception& e) {
        std::cerr << "ERROR: Failed to load FMUs: " << e.what() << std::endl;
        return 1;
    }

    // Instantiate FMUs
    try {
        vehicle_fmu.Instantiate("WheeledVehicle4TorquesFMU", false, visible);
        driver_fmu.Instantiate("PathFollowerDriverFMU", false, visible);
    } catch (std::exception& e) {
        std::cerr << "ERROR: Failed to instantiate FMUs: " << e.what() << std::endl;
        return 1;
    }

    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);
    driver_fmu.SetDebugLogging(fmi2True, logCategories);

    double start_time = 0.0;
    double step_size = 1e-3;

    vehicle_fmu.SetupExperiment(fmi2False, 0.0, start_time, fmi2False, stop_time);
    driver_fmu.SetupExperiment(fmi2False, 0.0, start_time, fmi2False, stop_time);

    // Enter initialization mode
    vehicle_fmu.EnterInitializationMode();
    driver_fmu.EnterInitializationMode();

    chrono::ChVector3d init_loc;
    double init_yaw;

    try {
        // Configure Driver FMU parameters
        driver_fmu.SetVariable("visible", visible, FmuVariable::Type::Boolean);
        driver_fmu.SetVariable("steering_type", 1, FmuVariable::Type::Integer); // Stanley steering
        driver_fmu.SetVariable("Kp_steering", Kp_steering, FmuVariable::Type::Real);
        driver_fmu.SetVariable("look_ahead_dist", look_ahead_dist, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Kp_speed", Kp_speed, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Ki_speed", Ki_speed, FmuVariable::Type::Real);

        // Exit initialization mode for Driver so we can query its path start position/heading
        driver_fmu.ExitInitializationMode();

        driver_fmu.GetVecVariable("init_loc", init_loc);
        driver_fmu.GetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
    } catch (std::exception& e) {
        std::cerr << "ERROR during Driver configuration/initialization: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Start location from driver path: " << init_loc.x() << ", " << init_loc.y() << ", " << init_loc.z() << std::endl;
    std::cout << "Start heading from driver path:  " << init_yaw << " rad" << std::endl;

    try {
        // Configure Vehicle FMU parameters using aligned initial conditions
        vehicle_fmu.SetVariable("visible", visible, FmuVariable::Type::Boolean);
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_vel", init_vel, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("terrain_type", 2, FmuVariable::Type::Integer); // OpenCRG
        vehicle_fmu.SetVariable("terrain_crg_file", "default_road.crg");
        vehicle_fmu.SetVariable("tire_coll_type", tire_coll_type, FmuVariable::Type::Integer);
        vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);

        // Exit initialization mode for Vehicle
        vehicle_fmu.ExitInitializationMode();

        // Disable image saving for speed
        driver_fmu.SetVariable("save_img", false);
    } catch (std::exception& e) {
        std::cerr << "ERROR during Vehicle configuration/initialization: " << e.what() << std::endl;
        return 1;
    }

    ChWriterCSV csv;
    csv.SetDelimiter(" ");

    double time = 0.0;
    chrono::ChFrameMoving<> ref_frame;

    std::cout << "\nStarting co-simulation loop..." << std::endl;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        // 1. Get vehicle reference frame and send to driver
        vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);
        driver_fmu.SetFrameMovingVariable("ref_frame", ref_frame);

        // 2. Set target speed
        driver_fmu.SetVariable("target_speed", init_vel, FmuVariable::Type::Real);

        // 3. Step driver FMU
        auto status_driver = driver_fmu.DoStep(time, step_size, fmi2True);
        if (status_driver == fmi2Discard) {
            std::cerr << "Driver FMU discarded step at time " << time << std::endl;
            break;
        }

        // 4. Retrieve driver controls
        double steering, throttle, braking;
        driver_fmu.GetVariable("steering", steering, FmuVariable::Type::Real);
        driver_fmu.GetVariable("throttle", throttle, FmuVariable::Type::Real);
        driver_fmu.GetVariable("braking", braking, FmuVariable::Type::Real);

        // Map throttle to driving torque
        double torque = throttle * max_torque;

        // 5. Send inputs to vehicle FMU
        vehicle_fmu.SetVariable("steering", steering, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("braking", braking, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("torque_FL", torque, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("torque_FR", torque, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("torque_RL", torque, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("torque_RR", torque, FmuVariable::Type::Real);

        // 6. Step vehicle FMU
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        if (status_vehicle == fmi2Discard) {
            std::cerr << "Vehicle FMU discarded step at time " << time << std::endl;
            break;
        }

        // Retrieve suspension velocities
        double susp_vel_FL = 0, susp_vel_FR = 0, susp_vel_RL = 0, susp_vel_RR = 0;
        vehicle_fmu.GetVariable("susp_FL.velocity", susp_vel_FL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_FR.velocity", susp_vel_FR, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_RL.velocity", susp_vel_RL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_RR.velocity", susp_vel_RR, FmuVariable::Type::Real);

        // Retrieve tire forces
        double tire_f_FL_x = 0, tire_f_FL_y = 0, tire_f_FL_z = 0;
        double tire_f_FR_x = 0, tire_f_FR_y = 0, tire_f_FR_z = 0;
        double tire_f_RL_x = 0, tire_f_RL_y = 0, tire_f_RL_z = 0;
        double tire_f_RR_x = 0, tire_f_RR_y = 0, tire_f_RR_z = 0;
        vehicle_fmu.GetVariable("wheel_FL.force.x", tire_f_FL_x, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FL.force.y", tire_f_FL_y, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FL.force.z", tire_f_FL_z, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.force.x", tire_f_FR_x, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.force.y", tire_f_FR_y, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.force.z", tire_f_FR_z, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.force.x", tire_f_RL_x, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.force.y", tire_f_RL_y, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.force.z", tire_f_RL_z, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.force.x", tire_f_RR_x, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.force.y", tire_f_RR_y, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.force.z", tire_f_RR_z, FmuVariable::Type::Real);

        // 7. Write trajectory, suspension velocities, and tire forces to CSV
        csv << time << ref_frame.GetPos() << ref_frame.GetRot().GetCardanAnglesXYZ()
            << susp_vel_FL << susp_vel_FR << susp_vel_RL << susp_vel_RR
            << tire_f_FL_x << tire_f_FL_y << tire_f_FL_z
            << tire_f_FR_x << tire_f_FR_y << tire_f_FR_z
            << tire_f_RL_x << tire_f_RL_y << tire_f_RL_z
            << tire_f_RR_x << tire_f_RR_y << tire_f_RR_z << std::endl;

        time += step_size;
    }

    timer.stop();
    std::cout << "Co-simulation finished. CPU time: " << timer() << "s for " << time << "s of simulation time." << std::endl;

    csv.WriteToFile(out_file);
    std::cout << "Trajectory written to: " << out_file << std::endl;

    return 0;
}
