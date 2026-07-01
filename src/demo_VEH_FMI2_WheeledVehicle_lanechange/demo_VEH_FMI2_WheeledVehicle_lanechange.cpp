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

struct SimulationParameters {
    double t_end = 10.0;
    double road_friction = 0.8;
};

struct SpeedProfilePoint {
    double time;
    double speed;
};

std::vector<SpeedProfilePoint> load_speed_profile(const std::string& filename) {
    std::vector<SpeedProfilePoint> profile;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Info: Speed profile file not found: " << filename << std::endl;
        return profile;
    }
    double t, v;
    while (file >> t >> v) {
        profile.push_back({t, v});
    }
    std::cout << "Successfully loaded speed profile with " << profile.size() << " points." << std::endl;
    return profile;
}

double interpolate_speed(double t, const std::vector<SpeedProfilePoint>& profile) {
    if (profile.empty()) return 0.0;
    if (t <= profile.front().time) return profile.front().speed;
    if (t >= profile.back().time) return profile.back().speed;
    
    // Linear interpolation
    for (size_t i = 0; i < profile.size() - 1; ++i) {
        if (t >= profile[i].time && t <= profile[i + 1].time) {
            double t0 = profile[i].time;
            double t1 = profile[i + 1].time;
            double v0 = profile[i].speed;
            double v1 = profile[i + 1].speed;
            double factor = (t - t0) / (t1 - t0);
            return v0 + factor * (v1 - v0);
        }
    }
    return profile.back().speed;
}

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
    std::cout << "Chrono FMI2 Co-Simulation - Sedan Lane Change Demo\n" << std::endl;

    // Command-line parameters with defaults
    bool vehicle_visible = true;
    bool driver_visible = false;
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
    std::string out_file = "lane_change_trajectory.csv";
    std::string path_file_arg = "";
    std::string terrain_crg_file_arg = "";
    std::string sim_params_arg = "";
    std::string speed_profile_file_arg = "";
    int tire_coll_type = 2;    // Default: 2 (2D Profile Envelope)
    double stop_time = 10.0;
    double fps = 30.0;         // Default to 30 FPS rendering frame rate
    int terrain_type = 2;      // Default: 2 (OpenCRG), 1 (OBJ Mesh)
    bool terrain_crg_simplify = true; // Default: true (enabled)
    std::string terrain_diffuse_texture = "";
    std::string terrain_normal_texture = "";
    bool terrain_show_visual_lines = false;
    bool stop_time_overridden = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            vehicle_visible = false;
            driver_visible = false;
        } else if (arg == "--visible") {
            vehicle_visible = true;
            driver_visible = true;
        } else if (arg == "--vehicle_visible" && i + 1 < argc) {
            vehicle_visible = (std::stoi(argv[++i]) != 0);
        } else if (arg == "--driver_visible" && i + 1 < argc) {
            driver_visible = (std::stoi(argv[++i]) != 0);
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
        } else if (arg == "--speed_profile_file" && i + 1 < argc) {
            speed_profile_file_arg = argv[++i];
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
        } else if (arg == "--simplify_crg" && i + 1 < argc) {
            terrain_crg_simplify = (std::stoi(argv[++i]) != 0);
        } else if (arg == "--terrain_diffuse_texture" && i + 1 < argc) {
            terrain_diffuse_texture = argv[++i];
        } else if (arg == "--terrain_normal_texture" && i + 1 < argc) {
            terrain_normal_texture = argv[++i];
        } else if (arg == "--terrain_show_visual_lines" && i + 1 < argc) {
            terrain_show_visual_lines = (std::stoi(argv[++i]) != 0);
        }
    }

    // Apply optimized defaults depending on steering controller type if not overridden
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

    std::cout << "Parameters:" << std::endl;
    std::cout << "  vehicle_visible: " << (vehicle_visible ? "true" : "false") << std::endl;
    std::cout << "  driver_visible:  " << (driver_visible ? "true" : "false") << std::endl;
    std::cout << "  steering_type:   " << (steering_type == 1 ? "1 (Stanley)" : "0 (PID)") << std::endl;
    std::cout << "  Kp_steering:     " << Kp_steering << std::endl;
    std::cout << "  Ki_steering:     " << Ki_steering << std::endl;
    std::cout << "  Kd_steering:     " << Kd_steering << std::endl;
    std::cout << "  look_ahead_dist: " << look_ahead_dist << std::endl;
    std::cout << "  stanley_dead_zone: " << stanley_dead_zone << std::endl;
    std::cout << "  Kp_speed:        " << Kp_speed << std::endl;
    std::cout << "  Ki_speed:        " << Ki_speed << std::endl;
    std::cout << "  Kd_speed:        " << Kd_speed << std::endl;
    std::cout << "  max_torque:      " << max_torque << std::endl;
    std::cout << "  init_vel:        " << init_vel << std::endl;
    std::cout << "  out_file:        " << out_file << std::endl;
    std::cout << "  path_file:       " << path_file_arg << std::endl;
    std::cout << "  terrain_crg:     " << terrain_crg_file_arg << std::endl;
    std::cout << "  tire_coll_type:  " << tire_coll_type << std::endl;
    std::cout << "  stop_time:       " << stop_time << std::endl;
    std::cout << "  fps:             " << fps << std::endl;
    std::cout << "  terrain_type:    " << (terrain_type == 1 ? "1 (OBJ Mesh)" : "2 (OpenCRG)") << std::endl;

    std::string vehicle_fmu_filename = "FMU2cs_WheeledVehicle4Torques.fmu";
    std::string driver_fmu_filename = "FMU2cs_PathFollowerDriver.fmu";

    // Locate FMUs
    std::filesystem::path exe_dir = std::filesystem::absolute(argv[0]).parent_path();
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

    std::cout << "Vehicle FMU path: " << vehicle_fmu_filename << std::endl;
    std::cout << "Driver FMU path:  " << driver_fmu_filename << std::endl;

    std::vector<std::string> logCategories = {"logAll"};

    std::filesystem::path build_dir = exe_dir.parent_path().parent_path();
    vehicle_fmu_filename = std::filesystem::absolute(vehicle_fmu_filename).string();
    driver_fmu_filename = std::filesystem::absolute(driver_fmu_filename).string();

    std::filesystem::path out_file_path(out_file);
    if (out_file_path.is_relative()) {
        out_file = std::filesystem::absolute(build_dir / out_file_path).string();
        out_file_path = std::filesystem::path(out_file);
    }

    std::string suffix = out_file_path.stem().string();
    std::string vehicle_unpack_dir = std::filesystem::absolute(build_dir / ("tmp_unpack_vehicle_lc_" + suffix)).string() + "/";
    std::string driver_unpack_dir = std::filesystem::absolute(build_dir / ("tmp_unpack_driver_lc_" + suffix)).string() + "/";

    if (!std::filesystem::exists(vehicle_fmu_filename)) {
        std::cerr << "ERROR: Vehicle FMU file not found: " << vehicle_fmu_filename << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(driver_fmu_filename)) {
        std::cerr << "ERROR: Driver FMU file not found: " << driver_fmu_filename << std::endl;
        return 1;
    }

    // Start nested scope to control FMU DLL lifecycles and release file locks
    {
        FmuChronoUnit vehicle_fmu;
        FmuChronoUnit driver_fmu;

        try {
            vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, vehicle_fmu_filename, vehicle_unpack_dir);
            driver_fmu.Load(fmi2Type::fmi2CoSimulation, driver_fmu_filename, driver_unpack_dir);
    } catch (std::exception& e) {
        std::cerr << "ERROR: Failed to load FMUs: " << e.what() << std::endl;
        return 1;
    }

    std::filesystem::path driver_resources_path = std::filesystem::path(driver_unpack_dir) / "resources";
    std::filesystem::path vehicle_resources_path = std::filesystem::path(vehicle_unpack_dir) / "resources";

    if (sim_params_arg.empty()) {
        std::filesystem::path default_params = build_dir / "generated" / "simulation_parameters.m";
        if (std::filesystem::exists(default_params)) {
            sim_params_arg = default_params.string();
        }
    }

    std::string speed_profile_file = "";
    if (!speed_profile_file_arg.empty()) {
        speed_profile_file = speed_profile_file_arg;
    } else {
        std::filesystem::path default_speed = build_dir / "generated" / "speed_profile.txt";
        if (std::filesystem::exists(default_speed)) {
            speed_profile_file = default_speed.string();
        }
    }

    std::vector<SpeedProfilePoint> speed_profile;
    if (!speed_profile_file.empty()) {
        std::cout << "Loading speed profile: " << speed_profile_file << std::endl;
        speed_profile = load_speed_profile(speed_profile_file);
    }

    if (!path_file_arg.empty()) {
        std::filesystem::path p_src(path_file_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, driver_resources_path / p_src.filename(), std::filesystem::copy_options::overwrite_existing);
        } else {
            std::cerr << "WARNING: Custom path file not found: " << path_file_arg << std::endl;
        }
    }
    if (!terrain_crg_file_arg.empty()) {
        std::filesystem::path p_src(terrain_crg_file_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, vehicle_resources_path / p_src.filename(), std::filesystem::copy_options::overwrite_existing);
        } else {
            std::cerr << "WARNING: Custom CRG file not found: " << terrain_crg_file_arg << std::endl;
        }
    }
    // Copy the OBJ mesh file to unpacked vehicle resources if it exists
    {
        std::filesystem::path obj_src = "";
        if (!terrain_crg_file_arg.empty()) {
            obj_src = std::filesystem::path(terrain_crg_file_arg).parent_path() / "default_road.obj";
        } else {
            obj_src = build_dir / "generated" / "default_road.obj";
        }
        if (std::filesystem::exists(obj_src)) {
            std::filesystem::copy_file(obj_src, vehicle_resources_path / "default_road.obj", std::filesystem::copy_options::overwrite_existing);
            std::cout << "[FMU] Copied generated OBJ mesh to unpacked resources: " << obj_src.string() << std::endl;
        }
    }
    if (!sim_params_arg.empty()) {
        std::filesystem::path p_src(sim_params_arg);
        if (std::filesystem::exists(p_src)) {
            std::filesystem::copy_file(p_src, vehicle_resources_path / "simulation_parameters.m", std::filesystem::copy_options::overwrite_existing);
        } else {
            std::cerr << "WARNING: Custom simulation parameters file not found: " << sim_params_arg << std::endl;
        }
    }

    // Verify unpacked driver resources
    if (!std::filesystem::exists(driver_resources_path / "default_lane_change_path.txt")) {
        std::cerr << "ERROR: default_lane_change_path.txt is missing in the unpacked Driver FMU resources!" << std::endl;
        return 1;
    }

    // Verify unpacked vehicle resources
    if (!std::filesystem::exists(vehicle_resources_path / "Vehicle.json")) {
        std::cerr << "ERROR: Vehicle.json is missing in the unpacked Vehicle FMU resources!" << std::endl;
        return 1;
    }
    if (terrain_type == 2 && !std::filesystem::exists(vehicle_resources_path / "default_road.crg")) {
        std::cerr << "ERROR: default_road.crg is missing in the unpacked Vehicle FMU resources!" << std::endl;
        return 1;
    }
    if (terrain_type == 1 && !std::filesystem::exists(vehicle_resources_path / "default_road.obj")) {
        std::cerr << "ERROR: default_road.obj is missing in the unpacked Vehicle FMU resources!" << std::endl;
        return 1;
    }

    std::vector<std::string> required_vehicle_jsons = {
        "Chassis.json",
        "DoubleWishboneFront.json",
        "DoubleWishboneRear.json",
        "Wheel.json",
        "BrakeSimple_Front.json",
        "BrakeSimple_Rear.json",
        "RackPinion.json",
        "Driveline2WD.json"
    };
    for (const auto& json_file : required_vehicle_jsons) {
        if (!std::filesystem::exists(vehicle_resources_path / json_file)) {
            std::cerr << "ERROR: Subsystem config file (" << json_file << ") is missing in the unpacked Vehicle FMU resources!" << std::endl;
            return 1;
        }
    }

    // Instantiate FMUs
    try {
        vehicle_fmu.Instantiate("WheeledVehicle4TorquesFMU", false, vehicle_visible);
        driver_fmu.Instantiate("PathFollowerDriverFMU", false, driver_visible);
    } catch (std::exception& e) {
        std::cerr << "ERROR: Failed to instantiate FMUs: " << e.what() << std::endl;
        return 1;
    }

    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);
    driver_fmu.SetDebugLogging(fmi2True, logCategories);

    // Parse simulation parameters from generated simulation_parameters.m
    SimulationParameters sim_params = load_simulation_parameters((vehicle_resources_path / "simulation_parameters.m").string());
    if (!stop_time_overridden) {
        stop_time = sim_params.t_end;
        std::cout << "Dynamic stop time from simulation_parameters.m: " << stop_time << " s" << std::endl;
    }

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
        driver_fmu.SetVariable("visible", driver_visible);
        if (!path_file_arg.empty()) {
            driver_fmu.SetVariable("path_file", std::filesystem::path(path_file_arg).filename().string());
        } else {
            driver_fmu.SetVariable("path_file", std::string("default_lane_change_path.txt"));
        }
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

        // Exit initialization mode for Driver so we can query its path start position/heading
        driver_fmu.ExitInitializationMode();

        driver_fmu.GetVecVariable("init_loc", init_loc);
        driver_fmu.GetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        driver_fmu.GetVariable("init_roll", init_roll, FmuVariable::Type::Real);
        driver_fmu.GetVariable("init_pitch", init_pitch, FmuVariable::Type::Real);
    } catch (std::exception& e) {
        std::cerr << "ERROR during Driver configuration/initialization: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Start location from driver path: " << init_loc.x() << ", " << init_loc.y() << ", " << init_loc.z() << std::endl;
    std::cout << "Start heading from driver path:  " << init_yaw << " rad" << std::endl;
    std::cout << "Start roll from driver path:     " << init_roll << " rad" << std::endl;
    std::cout << "Start pitch from driver path:    " << init_pitch << " rad" << std::endl;

    double road_friction = sim_params.road_friction;
    std::cout << "Dynamic road friction from simulation_parameters.m: " << road_friction << std::endl;

    try {
        // Configure Vehicle FMU parameters using aligned initial conditions and friction setup
        vehicle_fmu.SetVariable("terrain_friction", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_FL", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_FR", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_RL", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("friction_RR", road_friction, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("visible", vehicle_visible);
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_roll", init_roll, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_pitch", init_pitch, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("init_vel", init_vel, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("terrain_type", terrain_type, FmuVariable::Type::Integer);
        if (terrain_type == 1) {
            vehicle_fmu.SetVariable("terrain_mesh_file", std::string("default_road.obj"));
        } else {
            if (!terrain_crg_file_arg.empty()) {
                vehicle_fmu.SetVariable("terrain_crg_file", std::filesystem::path(terrain_crg_file_arg).filename().string());
            } else {
                vehicle_fmu.SetVariable("terrain_crg_file", std::string("default_road.crg"));
            }
        }
        vehicle_fmu.SetVariable("terrain_crg_simplify", terrain_crg_simplify);
        vehicle_fmu.SetVariable("terrain_diffuse_texture", terrain_diffuse_texture);
        vehicle_fmu.SetVariable("terrain_normal_texture", terrain_normal_texture);
        vehicle_fmu.SetVariable("terrain_show_visual_lines", terrain_show_visual_lines);
        vehicle_fmu.SetVariable("tire_coll_type", tire_coll_type, FmuVariable::Type::Integer);
        vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("fps", fps, FmuVariable::Type::Real);

        // Exit initialization mode for Vehicle
        vehicle_fmu.ExitInitializationMode();

        // Disable image saving for speed (already false by default)
        // driver_fmu.SetVariable("save_img", false);
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
        double target_speed_val = init_vel;
        if (!speed_profile.empty()) {
            target_speed_val = interpolate_speed(time, speed_profile);
        }
        driver_fmu.SetVariable("target_speed", target_speed_val, FmuVariable::Type::Real);

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

        // Retrieve suspension travel
        double susp_trav_FL = 0, susp_trav_FR = 0, susp_trav_RL = 0, susp_trav_RR = 0;
        vehicle_fmu.GetVariable("susp_FL.travel", susp_trav_FL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_FR.travel", susp_trav_FR, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_RL.travel", susp_trav_RL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("susp_RR.travel", susp_trav_RR, FmuVariable::Type::Real);

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

        // Retrieve tire slip angles, slip ratios, and wheel angular speeds (omega)
        double slip_ang_FL = 0, slip_ang_FR = 0, slip_ang_RL = 0, slip_ang_RR = 0;
        double slip_rat_FL = 0, slip_rat_FR = 0, slip_rat_RL = 0, slip_rat_RR = 0;
        double omega_FL = 0, omega_FR = 0, omega_RL = 0, omega_RR = 0;
        vehicle_fmu.GetVariable("wheel_FL.slip_angle", slip_ang_FL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.slip_angle", slip_ang_FR, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.slip_angle", slip_ang_RL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.slip_angle", slip_ang_RR, FmuVariable::Type::Real);
        
        vehicle_fmu.GetVariable("wheel_FL.slip_ratio", slip_rat_FL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.slip_ratio", slip_rat_FR, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.slip_ratio", slip_rat_RL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.slip_ratio", slip_rat_RR, FmuVariable::Type::Real);
        
        vehicle_fmu.GetVariable("wheel_FL.ang_vel.y", omega_FL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_FR.ang_vel.y", omega_FR, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RL.ang_vel.y", omega_RL, FmuVariable::Type::Real);
        vehicle_fmu.GetVariable("wheel_RR.ang_vel.y", omega_RR, FmuVariable::Type::Real);

        // 7. Write trajectory, suspension velocities, tire forces, torques, wheel speeds, slips, travel, and driver controls to CSV
        csv << time << ref_frame.GetPos() << ref_frame.GetRot().GetCardanAnglesXYZ()
            << susp_vel_FL << susp_vel_FR << susp_vel_RL << susp_vel_RR
            << tire_f_FL_x << tire_f_FL_y << tire_f_FL_z
            << tire_f_FR_x << tire_f_FR_y << tire_f_FR_z
            << tire_f_RL_x << tire_f_RL_y << tire_f_RL_z
            << tire_f_RR_x << tire_f_RR_y << tire_f_RR_z
            << torque << torque << torque << torque
            << omega_FL << omega_FR << omega_RL << omega_RR
            << slip_ang_FL << slip_ang_FR << slip_ang_RL << slip_ang_RR
            << slip_rat_FL << slip_rat_FR << slip_rat_RL << slip_rat_RR
            << susp_trav_FL << susp_trav_FR << susp_trav_RL << susp_trav_RR
            << steering << throttle << braking << std::endl;

        time += step_size;
    }

    timer.stop();
    std::cout << "Co-simulation finished. CPU time: " << timer() << "s for " << time << "s of simulation time." << std::endl;

    csv.WriteToFile(out_file);
    std::cout << "Trajectory written to: " << out_file << std::endl;
    }

    try {
        std::filesystem::remove_all(vehicle_unpack_dir);
    } catch (const std::exception& e) {
        std::cout << "Note: Temporary vehicle unpack directory not removed: " << e.what() << std::endl;
    }
    try {
        std::filesystem::remove_all(driver_unpack_dir);
    } catch (const std::exception& e) {
        std::cout << "Note: Temporary driver unpack directory not removed: " << e.what() << std::endl;
    }

    return 0;
}
