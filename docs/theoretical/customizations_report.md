# Vehicle & Driver FMU Customizations Report

This document reports the technical details of the customizations applied to the vehicle and driver models to adapt them for FMI-compliant co-simulation.

---

## 🏎️ 1. Vehicle FMU Customizations (`FMU_WheeledVehicle4Torques`)

The Wheeled Vehicle FMU implements several major modifications relative to standard projectchrono vehicle demos to enable external control and flat-packed deployment.

### A. Driveline Disconnection (Independent 4-Wheel Torque Control)
* **Customization**: The standard powertrain/driveline connection (where engine torque flows through a torque converter, transmission, and differential gearboxes to the axles) was bypassed.
* **Mechanism**: Independent torque inputs (`torque_FL`, `torque_FR`, `torque_RL`, `torque_RR`) are exposed directly as FMI real input variables. The FMU applies these torques directly to each wheel spindle at each simulation step, enabling external active torque vectoring, traction control, or hybrid powertrain co-simulation in platforms like Simulink.

### B. Dynamically Parameterized Terrains
The FMU vehicle model was upgraded to allow the simulation coordinator to dynamically select the terrain representation at initialization time using FMI parameters:
1. **Flat Terrain (`terrain_type = 0`)**: Uses a simple plane with configurable friction coefficient (`terrain_friction`).
2. **Rigid Mesh Terrain (`terrain_type = 1`)**: Loads an arbitrary Wavefront `.obj` 3D mesh file passed as an absolute path or relative resources path (`terrain_mesh_file`).
3. **OpenCRG Road Terrain (`terrain_type = 2`)**: Statically links the OpenCRG library to load curved elevation/roughness road files (`terrain_crg_file`).
* *Note*: Spawning vertical coordinates (`init_loc_z`) are automatically adjusted depending on terrain type (e.g. `0.2` for Flat, `0.1` for curved CRG road elevation) to ensure the vehicle tires spawn flat on the road.

### C. Flat-Resource Packaging (JSON Flattening)
Standard Chrono vehicle assemblies search for sub-components using nested relative directory paths (e.g., `"sedan/chassis/Sedan_Chassis.json"`). 
* **Customization**: The subsystem parameter files inside the FMU's resources directory have been modified to reference paths in a flat layout (e.g., `"Chassis.json"`). This keeps the FMU fully self-contained and allows FMI Forge to package all dependencies inside the ZIP structure, running seamlessly on target machines without Chrono installations.

---

## 🏁 2. Driver FMU Customizations (`FMU_PathFollowerDriver`)

The driver controller was customized to support high-fidelity closed-loop lateral tracking:

* **Dual Control Strategies**: Employs FMI integer parameters to switch between:
  - **PID Lateral Controller (`lateral_type = 0`)**: Computes steering output based on lateral deviation and heading error.
  - **Stanley Lateral Controller (`lateral_type = 1`)**: Uses a geometric path-following formulation measuring cross-track error at the front axle center.
* **Dynamic Path Loading**: The driver model accepts the Bezier centerline path file path via the FMI parameter `path_file` during initialization.

---

## 📦 3. Build-Time Staging & Asset Exclusion

To keep the repository clean and avoid version-controlling heavy binary files, standard Chrono assets are excluded from Git and staged at build time:

* **Visual Meshes & Fonts**: Mesh files (`sedan_chassis_vis.*`, etc.) and Irrlicht fonts (`arial8.*`) are not checked into the repository.
* **Dynamic Copying**: The `stage_fmu_resources` target in `src/fmu2_wheeled_vehicle_4torques/CMakeLists.txt` automatically retrieves these files from the sibling Chrono data folder (`chrono/data/vehicle/sedan/` and `chrono/data/fonts/`) at build-configure time and stages them inside a local `build/resources_staging/` folder before compilation and packaging.
