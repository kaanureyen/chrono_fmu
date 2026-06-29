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
// Launcher demo illustrating the co-simulation of the custom Chrono wheeled
// vehicle FMU and path follower driver FMU with dynamically loaded trajectories
// and reference speed profiles.
//
// =============================================================================

#include <array>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <cmath>
#include <algorithm>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::fmi2;

struct SpeedPoint {
    double time;
    double speed;
};

// Helper function to read the speed profile lookup table
std::vector<SpeedPoint> load_speed_profile(const std::string& filename) {
    std::vector<SpeedPoint> profile;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Info: Speed profile lookup table file not found: " << filename 
                  << ". Using constant cruise speed instead." << std::endl;
        return profile;
    }
    double t, v;
    while (file >> t >> v) {
        profile.push_back({t, v});
    }
    std::cout << "Loaded speed profile table with " << profile.size() << " points from: " << filename << std::endl;
    return profile;
}

// Linear interpolation for speed profile lookup table
double get_target_speed(const std::vector<SpeedPoint>& profile, double time, double default_speed) {
    if (profile.empty()) return default_speed;
    if (time <= profile.front().time) return profile.front().speed;
    if (time >= profile.back().time) return profile.back().speed;
    
    for (size_t i = 0; i < profile.size() - 1; ++i) {
        if (time >= profile[i].time && time <= profile[i+1].time) {
            double t0 = profile[i].time;
            double t1 = profile[i+1].time;
            double v0 = profile[i].speed;
            double v1 = profile[i+1].speed;
            return v0 + (v1 - v0) * (time - t0) / (t1 - t0);
        }
    }
    return default_speed;
}

struct SimulationParameters {
    double t_end = 10.0;
    double road_friction = 0.8;
};

// Helper function to parse t_end and road_friction_mu from simulation_parameters.m at runtime
SimulationParameters load_simulation_parameters(const std::string& filename) {
    SimulationParameters params;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Info: Matlab simulation parameters file not found: " << filename << std::endl;
        return params;
    }
    std::string line;
    while (std::getline(file, line)) {
        // Strip spaces and comments
        std::string clean_line = "";
        for (char c : line) {
            if (c == '%') break;
            if (!std::isspace(c)) {
                clean_line += c;
            }
        }
        if (clean_line.empty()) continue;
        
        size_t eq_pos = clean_line.find('=');
        if (eq_pos == std::string::npos) continue;
        
        std::string key = clean_line.substr(0, eq_pos);
        std::string val_str = clean_line.substr(eq_pos + 1);
        
        if (!val_str.empty() && val_str.back() == ';') {
            val_str.pop_back();
        }
        
        try {
            if (key == "simulation_end_time") {
                params.t_end = std::stod(val_str);
            } else if (key == "road_friction_mu") {
                params.road_friction = std::stod(val_str);
            }
        } catch (...) {
            // ignore malformed values
        }
    }
    return params;
}

int main(int argc, char* argv[]) {
    std::cout << "Chrono FMI2 Dynamic Co-Simulation - Vehicle Launcher Demo\n" << std::endl;

    // Command-line parameters with defaults
    bool visible = true;
    int steering_type = 1;      // Default to Stanley (1). 0 for PID.
    double Kp_steering = -1.0;
    double Ki_steering = -1.0;
    double Kd_steering = 0.0;
    double look_ahead_dist = -1.0;
    double Kp_speed = 0.868900;
    double Ki_speed = 0.436516;
    double Kd_speed = 0.0;
    double max_torque = 350.0;
    double init_vel = 16.6667; // 60 kph
    double stanley_dead_zone = -1.0;
    std::string out_file = "cosim_launcher_trajectory.csv";
    std::string path_file_arg = "";
    std::string terrain_crg_file_arg = "";
    std::string speed_profile_arg = "";
    std::string sim_params_arg = "";
    int tire_coll_type = 2;    // Default: 2 (2D Profile Envelope)
    double stop_time = 10.0;
    double fps = 30.0;         // Default to 30 FPS rendering frame rate
    int terrain_type = 2;      // Default: 2 (OpenCRG), 1 (OBJ Mesh)
    bool stop_time_overridden = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            visible = false;
        } else if (arg == "--visible") {
            visible = true;
        } else if (arg == "--steering_type" && i + 1 < argc) {
            steering_type = std::stoi(argv[++i]);
        } else if (arg == "--Kp_steering" && i + 1 < argc) {
            Kp_steering = std::stod(argv[++i]);
        } else if (arg == "--Ki_steering" && i + 1 < argc) {
            Ki_steering = std::stod(argv[++i]);
        } else if (arg == "--Kd_steering" && i + 1 < argc) {
            Kd_steering = std::stod(argv[++i]);
        } else if (arg == "--look_ahead_dist" && i + 1 < argc) {
            look_ahead_dist = std::stod(argv[++i]);
        } else if (arg == "--Kp_speed" && i + 1 < argc) {
            Kp_speed = std::stod(argv[++i]);
        } else if (arg == "--Ki_speed" && i + 1 < argc) {
            Ki_speed = std::stod(argv[++i]);
        } else if (arg == "--Kd_speed" && i + 1 < argc) {
            Kd_speed = std::stod(argv[++i]);
        } else if (arg == "--max_torque" && i + 1 < argc) {
            max_torque = std::stod(argv[++i]);
        } else if (arg == "--init_vel" && i + 1 < argc) {
            init_vel = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            out_file = argv[++i];
        } else if (arg == "--path_file" && i + 1 < argc) {
            path_file_arg = argv[++i];
        } else if (arg == "--terrain_crg_file" && i + 1 < argc) {
            terrain_crg_file_arg = argv[++i];
        } else if (arg == "--speed_profile" && i + 1 < argc) {
            speed_profile_arg = argv[++i];
        } else if (arg == "--stanley_dead_zone" && i + 1 < argc) {
            stanley_dead_zone = std::stod(argv[++i]);
        } else if (arg == "--tire_coll_type" && i + 1 < argc) {
            tire_coll_type = std::stoi(argv[++i]);
        } else if (arg == "--tend" && i + 1 < argc) {
            stop_time = std::stod(argv[++i]);
            stop_time_overridden = true;
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = std::stod(argv[++i]);
        } else if (arg == "--terrain" && i + 1 < argc) {
            terrain_type = std::stoi(argv[++i]);
        }
    }

    // Locate FMUs
    std::filesystem::path exe_dir = std::filesystem::absolute(argv[0]).parent_path();
    std::filesystem::path build_dir = exe_dir.parent_path().parent_path();
    std::string vehicle_fmu_filename = "FMU2cs_WheeledVehicle4Torques.fmu";
    std::string driver_fmu_filename = "FMU2cs_PathFollowerDriver.fmu";

    // Setup default fallback files from road generator output
    if (path_file_arg.empty()) {
        std::filesystem::path default_path = build_dir / "generated" / "default_lane_change_path.txt";
        if (std::filesystem::exists(default_path)) {
            path_file_arg = default_path.string();
        }
    }
    if (terrain_crg_file_arg.empty() && terrain_type == 2) {
        std::filesystem::path default_crg = build_dir / "generated" / "default_road.crg";
        if (std::filesystem::exists(default_crg)) {
            terrain_crg_file_arg = default_crg.string();
        }
    }
    if (speed_profile_arg.empty()) {
        std::filesystem::path default_profile = build_dir / "generated" / "speed_profile.txt";
        if (std::filesystem::exists(default_profile)) {
            speed_profile_arg = default_profile.string();
        }
    }
    if (sim_params_arg.empty()) {
        std::filesystem::path default_params = build_dir / "generated" / "simulation_parameters.m";
        if (std::filesystem::exists(default_params)) {
            sim_params_arg = default_params.string();
        }
    }

    // Locate FMU binary targets
    if (!std::filesystem::exists(vehicle_fmu_filename)) {
        std::filesystem::path p = exe_dir / "FMU2cs_WheeledVehicle4Torques" / "FMU2cs_WheeledVehicle4Torques.fmu";
        if (std::filesystem::exists(p)) {
            vehicle_fmu_filename = p.string();
        } else {
            p = exe_dir.parent_path() / "FMU2cs_WheeledVehicle4Torques" / "FMU2cs_WheeledVehicle4Torques.fmu";
            if (std::filesystem::exists(p)) {
                vehicle_fmu_filename = p.string();
            } else {
                p = exe_dir.parent_path().parent_path() / "FMU2cs_WheeledVehicle4Torques" / "FMU2cs_WheeledVehicle4Torques.fmu";
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
                p = exe_dir.parent_path().parent_path() / "FMU2cs_PathFollowerDriver" / "FMU2cs_PathFollowerDriver.fmu";
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
    }

    std::cout << "Loading FMUs..." << std::endl;
    std::cout << "  Vehicle FMU: " << vehicle_fmu_filename << std::endl;
    std::cout << "  Driver FMU:  " << driver_fmu_filename << std::endl;

    // Apply controller gains depending on mode if not overridden
    if (steering_type == 1) { // Stanley
        if (Kp_steering < 0) Kp_steering = 2.398832;
        if (Ki_steering < 0) Ki_steering = 0.0;
        if (look_ahead_dist < 0) look_ahead_dist = 3.615358;
        if (stanley_dead_zone < 0) stanley_dead_zone = 0.010965;
    } else { // PID
        if (Kp_steering < 0) Kp_steering = 1.047129;
        if (Ki_steering < 0) Ki_steering = 0.01;
        if (look_ahead_dist < 0) look_ahead_dist = 4.990583;
        if (stanley_dead_zone < 0) stanley_dead_zone = 0.0;
    }

    std::cout << "Sim Configuration parameters:" << std::endl;
    std::cout << "  Visible rendering: " << (visible ? "ON" : "OFF") << std::endl;
    std::cout << "  Steering type:      " << (steering_type == 1 ? "1 (Stanley)" : "0 (PID)") << std::endl;
    std::cout << "  Steer Gains (P,I,D):" << Kp_steering << ", " << Ki_steering << ", " << Kd_steering << std::endl;
    std::cout << "  Speed Gains (P,I,D):" << Kp_speed << ", " << Ki_speed << ", " << Kd_speed << std::endl;
    std::cout << "  Terrain Mode:       " << (terrain_type == 1 ? "OBJ Mesh" : "OpenCRG") << std::endl;
    std::cout << "  Active Path:        " << (path_file_arg.empty() ? "Unspecified (will fail)" : path_file_arg) << std::endl;
    std::cout << "  Active Speed Prof:  " << (speed_profile_arg.empty() ? "None (constant speed)" : speed_profile_arg) << std::endl;

    FmuChronoUnit vehicle_fmu;
    FmuChronoUnit driver_fmu;
    std::vector<std::string> logCategories = {"logAll"};

    std::filesystem::path out_file_path(out_file);
    if (out_file_path.is_relative()) {
        out_file = std::filesystem::absolute(build_dir / out_file_path).string();
        out_file_path = std::filesystem::path(out_file);
    }

    std::string suffix = out_file_path.stem().string();
    std::string vehicle_unpack_dir = std::filesystem::absolute(build_dir / ("tmp_unpack_vehicle_cosim_" + suffix)).string() + "/";
    std::string driver_unpack_dir = std::filesystem::absolute(build_dir / ("tmp_unpack_driver_cosim_" + suffix)).string() + "/";

    if (!std::filesystem::exists(vehicle_fmu_filename)) {
        std::cerr << "ERROR: Vehicle FMU file not found!" << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(driver_fmu_filename)) {
        std::cerr << "ERROR: Driver FMU file not found!" << std::endl;
        return 1;
    }

    try {
        vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, vehicle_fmu_filename, vehicle_unpack_dir);
        driver_fmu.Load(fmi2Type::fmi2CoSimulation, driver_fmu_filename, driver_unpack_dir);
    } catch (std::exception& e) {
        std::cerr << "ERROR: Failed to load FMUs: " << e.what() << std::endl;
        return 1;
    }

    std::filesystem::path driver_resources_path = std::filesystem::path(driver_unpack_dir) / "resources";
    std::filesystem::path vehicle_resources_path = std::filesystem::path(vehicle_unpack_dir) / "resources";

    // Stage generated path, speed profile, and CRG road into the unpack folders
    if (!path_file_arg.empty()) {
        std::filesystem::path p_src(path_file_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, driver_resources_path / "default_lane_change_path.txt", std::filesystem::copy_options::overwrite_existing);
            std::cout << "Staged active trajectory file to driver resources." << std::endl;
        } else {
            std::cerr << "ERROR: Path file not found: " << path_file_arg << std::endl;
            return 1;
        }
    }
    if (!terrain_crg_file_arg.empty() && terrain_type == 2) {
        std::filesystem::path p_src(terrain_crg_file_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, vehicle_resources_path / "default_road.crg", std::filesystem::copy_options::overwrite_existing);
            std::cout << "Staged OpenCRG terrain file to vehicle resources." << std::endl;
        } else {
            std::cerr << "ERROR: CRG file not found: " << terrain_crg_file_arg << std::endl;
            return 1;
        }
    }
    if (!speed_profile_arg.empty()) {
        std::filesystem::path p_src(speed_profile_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, driver_resources_path / "speed_profile.txt", std::filesystem::copy_options::overwrite_existing);
            std::cout << "Staged speed profile lookup file to driver resources." << std::endl;
        }
    }
    if (!sim_params_arg.empty()) {
        std::filesystem::path p_src(sim_params_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, vehicle_resources_path / "simulation_parameters.m", std::filesystem::copy_options::overwrite_existing);
            std::cout << "Staged simulation parameters file to vehicle resources." << std::endl;
        }
    }

    // Parse simulation parameters from generated simulation_parameters.m
    SimulationParameters sim_params = load_simulation_parameters((vehicle_resources_path / "simulation_parameters.m").string());
    if (!stop_time_overridden) {
        stop_time = sim_params.t_end;
        std::cout << "Dynamic stop time from simulation_parameters.m: " << stop_time << " s" << std::endl;
    }

    // Load reference speed profile table for dynamic speed lookup during the simulation loop
    std::vector<SpeedPoint> speed_table = load_speed_profile((driver_resources_path / "speed_profile.txt").string());

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
    double init_roll;
    double init_pitch;

    try {
        // Configure Driver FMU parameters
        driver_fmu.SetVariable("visible", visible, FmuVariable::Type::Boolean);
        driver_fmu.SetVariable("path_file", std::string("default_lane_change_path.txt"), FmuVariable::Type::String);
        driver_fmu.SetVariable("steering_type", steering_type, FmuVariable::Type::Integer);
        driver_fmu.SetVariable("Kp_steering", Kp_steering, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Ki_steering", Ki_steering, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Kd_steering", Kd_steering, FmuVariable::Type::Real);
        driver_fmu.SetVariable("look_ahead_dist", look_ahead_dist, FmuVariable::Type::Real);
        driver_fmu.SetVariable("stanley_dead_zone", stanley_dead_zone, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Kp_speed", Kp_speed, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Ki_speed", Ki_speed, FmuVariable::Type::Real);
        driver_fmu.SetVariable("Kd_speed", Kd_speed, FmuVariable::Type::Real);
        driver_fmu.SetVariable("fps", fps, FmuVariable::Type::Real);

        // Exit initialization mode for Driver so we can query start position/heading
        driver_fmu.ExitInitializationMode();

        driver_fmu.GetVecVariable("init_loc", init_loc);
        driver_fmu.GetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        driver_fmu.GetVariable("init_roll", init_roll, FmuVariable::Type::Real);
        driver_fmu.GetVariable("init_pitch", init_pitch, FmuVariable::Type::Real);
    } catch (std::exception& e) {
        std::cerr << "ERROR during Driver configuration/initialization: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Aligned starting location: " << init_loc.x() << ", " << init_loc.y() << ", " << init_loc.z() << std::endl;
    std::cout << "Aligned starting heading:  " << init_yaw << " rad" << std::endl;
    std::cout << "Aligned starting roll:     " << init_roll << " rad" << std::endl;
    std::cout << "Aligned starting pitch:    " << init_pitch << " rad" << std::endl;

    double road_friction = sim_params.road_friction;
    std::cout << "Dynamic road friction from simulation_parameters.m: " << road_friction << std::endl;

    try {
        // Configure Vehicle FMU parameters using aligned initial conditions and friction setup
        vehicle_fmu.SetVariable("terrain_friction", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_FL", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_FR", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_RL", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_RR", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("visible", visible, FmuVariable::Type::Boolean);
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_roll", init_roll, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_pitch", init_pitch, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_vel", init_vel, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("terrain_type", terrain_type, FmuVariable::Type::Integer);
        if (terrain_type == 1) {
            vehicle_fmu.SetVariable("terrain_mesh_file", std::string("default_road.obj"));
        } else {
            vehicle_fmu.SetVariable("terrain_crg_file", std::string("default_road.crg"), FmuVariable::Type::String);
        }
        vehicle_fmu.SetVariable("tire_coll_type", tire_coll_type, FmuVariable::Type::Integer);
        vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("fps", fps, FmuVariable::Type::Real);

        // Exit initialization mode for Vehicle
        vehicle_fmu.ExitInitializationMode();
    } catch (std::exception& e) {
        std::cerr << "ERROR during Vehicle configuration/initialization: " << e.what() << std::endl;
        return 1;
    }

    ChWriterCSV csv;
    csv.SetDelimiter(" ");

    double time = 0.0;
    chrono::ChFrameMoving<> ref_frame;

    std::cout << "\nExecuting co-simulation loop..." << std::endl;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        // 1. Get vehicle reference frame and send to driver
        vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);
        driver_fmu.SetFrameMovingVariable("ref_frame", ref_frame);

        // 2. Look up and send dynamic target speed reference
        double target_speed = get_target_speed(speed_table, time, init_vel);
        driver_fmu.SetVariable("target_speed", target_speed, FmuVariable::Type::Real);

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

        // 7. Write outputs to CSV
        csv << time << ref_frame.GetPos() << ref_frame.GetRot().GetCardanAnglesXYZ()
            << target_speed << steering << throttle << braking
            << susp_vel_FL << susp_vel_FR << susp_vel_RL << susp_vel_RR
            << tire_f_FL_x << tire_f_FL_y << tire_f_FL_z
            << tire_f_FR_x << tire_f_FR_y << tire_f_FR_z
            << tire_f_RL_x << tire_f_RL_y << tire_f_RL_z
            << tire_f_RR_x << tire_f_RR_y << tire_f_RR_z << std::endl;

        time += step_size;
    }

    timer.stop();
    std::cout << "Co-simulation finished. CPU time: " << timer() << "s." << std::endl;

    csv.WriteToFile(out_file);
    std::cout << "Co-simulation results successfully written to: " << out_file << std::endl;

    try {
        std::filesystem::remove_all(vehicle_unpack_dir);
        std::filesystem::remove_all(driver_unpack_dir);
    } catch (...) {
        // Ignore file locks on termination
    }

    return 0;
}
