# Chrono Custom FMI 2.0 Co-Simulation FMUs

This repository contains custom FMI 2.0 Co-Simulation Functional Mock-up Units (FMUs) for vehicle dynamics studies using [Project Chrono](https://projectchrono.org/). It packages models for a 4-wheel torque-input vehicle and a path-following driver, along with a co-simulation orchestrator demo.

---

## 📂 Repository Structure

* **`src/`**: Source code directory grouping all C++ modules and tools:
  * `fmu2_wheeled_vehicle_4torques/`: Wheeled vehicle FMU model source code and local config parameters.
  * `fmu2_path_follower_driver/`: Closed-loop driver FMU controller source code.
  * `demo_VEH_FMI2_WheeledVehicle_4torques/`: Co-simulation straight-line acceleration/braking/cornering demo.
  * `demo_VEH_FMI2_WheeledVehicle_lanechange/`: Co-simulation Stanley lateral path-following lane change demo.
  * `road_generator/`: Road profiling and path coordinates generator C++ utility and Python coordinator.
* **`docs/`**: Documentation directory subdivided by topic:
  * `docs/software/`: Software guidelines (installation, directory structure, Simulink co-simulation guide, parameter sweeps).
  * `docs/theoretical/`: Mathematical modeling reports (KPI calculations, vehicle parameters guide, Bezier spline paths).
* **`scripts/`**: Relative-path batch scripts to automate core Chrono bootstrapping, FMU builds, and cosim runs.
* **`parameter_sweep.py`**: Parameter sweep grid utility to optimize driver tracking gains.
* **`road_generator_gui.py`**: Tkinter GUI wizard to customize and generate road/path profiles interactively.

---

## ⚙️ FMU Specifications

### 1. Wheeled Vehicle FMU (`FMU2cs_WheeledVehicle4Torques`)
Models a 4-wheel vehicle chassis with double wishbone suspension, accepting independent wheel torque inputs directly at the axles. Tires and terrain are simulated internally.

* **Parameters (Variability: Fixed):**
  * `vehicle_JSON`: Vehicle structure file (.json)
  * `tire_JSON`: Tire model description file (.json)
  * `terrain_type`: Type of road (0: Flat, 1: OBJ mesh, 2: OpenCRG)
  * `terrain_mesh_file` / `terrain_crg_file`: Terrain geometry files
  * `terrain_friction`: Friction coefficient
  * `init_loc` / `init_yaw`: Starting location and heading
  * `step_size`: Solver integration time step
* **Inputs (Variability: Continuous):**
  * `steering` / `braking`: Normalized control inputs `[-1.0, 1.0]` / `[0.0, 1.0]`
  * `torque_FL`, `torque_FR`, `torque_RL`, `torque_RR`: Axle motor torques (`Nm`)
  * `act_force_FL`, `act_force_FR`, `act_force_RL`, `act_force_RR`: Active suspension vertical forces (`N`)
* **Outputs (Variability: Continuous):**
  * `ref_frame`: 6-DOF moving reference frame of the chassis (position, rotation, velocities, accelerations)
  * `wheel_<ID>.pos` / `wheel_<ID>.rot`: Position (`m`) and orientation quaternion (`1`) for each wheel
  * `wheel_<ID>.lin_vel` / `wheel_<ID>.ang_vel`: Linear (`m/s`) and angular (`rad/s`) velocities for each wheel
  * `wheel_<ID>.force`: 3D tire-road contact force vector (`N`) for wear and road-holding calculation
  * `wheel_<ID>.slip_angle` / `wheel_<ID>.slip_ratio`: Tire side slip angle (\(\alpha\)) and longitudinal slip ratio (\(\kappa\))
  * `susp_<ID>.travel` / `susp_<ID>.velocity`: Suspension TSDA deflection (`m`) and stroke speed (`m/s`)

---

### 2. Path Follower Driver FMU (`FMU2cs_PathFollowerDriver`)
Implements a lateral path-following controller (PID or Stanley) and a longitudinal cruise control speed controller.

* **Parameters (Variability: Fixed):**
  * `path_file`: Bezier path text file
  * `steering_type`: Steering controller (0: PID, 1: Stanley)
  * `look_ahead_dist`, `Kp_steering`, `Ki_steering`, `Kd_steering`: Lateral control gains
  * `Kp_speed`, `Ki_speed`, `Kd_speed`: Cruise control gains
* **Inputs (Variability: Continuous):**
  * `ref_frame`: The vehicle's 6-DOF chassis frame input
  * `target_speed`: Target velocity command (`m/s`)
* **Outputs (Variability: Continuous):**
  * `steering` / `throttle` / `braking`: Driver output control commands

---

## 🛠️ Installation, Build, and Run Guide

For complete workspace setup instructions, compiler prerequisites (using `winget`), clone commands, and automated building, see the [Installation Guide](file:///c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus/docs/software/installation.md).

For a quick reference, the `scripts/` folder contains helper batch files:

1. **Build Chrono Library:**
   Run [scripts/build_chrono.bat](file:///c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus/scripts/build_chrono.bat) to configure, compile, and install Project Chrono and its dependencies (Irrlicht, OpenCRG).
2. **Build Custom FMUs:**
   Run [scripts/build_fmus.bat](file:///c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus/scripts/build_fmus.bat) to generate paths/terrains and compile the FMUs.
3. **Run Co-Simulation Demo:**
   Run [scripts/run_demo.bat](file:///c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus/scripts/run_demo.bat) to execute the orchestrator demo.
