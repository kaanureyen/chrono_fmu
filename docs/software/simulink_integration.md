# Chrono Vehicle & Driver FMU Co-Simulation Integration Guide

This package contains two pre-compiled, self-contained **FMI 2.0 Co-Simulation Functional Mock-up Units (FMUs)**. They allow you to run high-fidelity wheeled vehicle and closed-loop driver simulations directly in MATLAB Simulink without requiring Chrono source compilation or local headers.

---

## 📦 Package Contents

1. **`FMU2cs_WheeledVehicle4Torques.fmu`**: Wheeled vehicle model (based on the Sedan archetype) with a disconnected driveline to accept independent torque inputs at all four wheels. Includes internal flat/mesh terrain, OpenCRG support (statically linked), and tire formulations.
2. **`FMU2cs_PathFollowerDriver.fmu`**: Closed-loop steering and speed driver. Supports both PID and Stanley lateral path-following, and longitudinal cruise control.
3. **`fmu_parameters.m`**: MATLAB script to load and initialize all simulation parameters into the MATLAB workspace. Path variables are resolved dynamically relative to the script's directory.
4. **`default_lane_change_path.txt`**: Generated Bezier path file defining the double lane-change maneuver.
5. **`default_road.crg`**: Generated OpenCRG road file representing the physical road surface.
6. **`bezier_path_generation.md`**: Guide explaining how to convert 3rd, 5th, and 7th-degree polynomials and discrete coordinate curves into Chrono Bezier paths.

---

## ⚙️ Simulink Setup Steps

Follow these steps to build and run the co-simulation:

### Step 1: Run Parameter Script
Open MATLAB, navigate to this folder, and run the `fmu_parameters.m` script. This initializes all the necessary configurations (such as gains, speeds, initial states, and paths) in the MATLAB workspace. All file path parameters are automatically populated with absolute paths using forward slashes.

### Step 2: Create a Simulink Model
1. Open Simulink and create a **Blank Model**.
2. Save the model in the same folder (e.g., `wheeled_vehicle_cosim.slx`).

### Step 3: Insert the FMU Blocks
1. Open the Simulink Library Browser.
2. Search for **FMU Import** (found under *Simulink / Import & Export*).
3. Drag two **FMU Import** blocks into your model canvas:
   - Name the first block: `Vehicle FMU`
   - Name the second block: `Driver FMU`

### Step 4: Configure the Blocks
1. **Vehicle FMU**:
   - Double-click the block, click **Browse**, and select `FMU2cs_WheeledVehicle4Torques.fmu`.
   - In the **Parameters** tab, reference the workspace variables (e.g., set `tire_JSON` to `tire_JSON`, `terrain_type` to `terrain_type`, `terrain_crg_file` to `terrain_crg_file`, `terrain_mesh_file` to `terrain_mesh_file`, and `step_size` to `step_size`).
2. **Driver FMU**:
   - Double-click the block, click **Browse**, and select `FMU2cs_PathFollowerDriver.fmu`.
   - In the **Parameters** tab, reference the workspace variables (e.g., set `Kp_steering` to `Kp_steering`, `steering_type` to `steering_type`, and `path_file` to `path_file`).

### Step 5: Connect the Ports
The two FMUs should be connected in a closed loop. Since FMI structured variables are expanded into flat scalar ports in Simulink, map them as follows:

```
                  +-----------------------------------+
                  |            Driver FMU             |
                  |                                   |
                  |  [Inputs]         [Outputs]       |
                  |  ref_frame.* <---+ steering ------+---------+
                  |  target_speed    | throttle ------+--[X]--+ |
                  |                  | braking -------+-+     | |
                  +-----------------------------------+ |     | |
                                                        |     | |
                                                        v     v v
                  +-----------------------------------+ |     | |
                  |            Vehicle FMU            | |     | |
                  |                                   | |     | |
                  |  [Inputs]         [Outputs]       | |     | |
                  |  steering <-----------------------+-+     | |
                  |  braking <------------------------+       | |
                  |  torque_FL <-----+                        | |
                  |  torque_FR <-----+------------------------+ |
                  |  torque_RL <-----+ (e.g. throttle * gain)   |
                  |  torque_RR <-----+                          |
                  |  ref_frame.* ----> ref_frame.pos/rot/... ---+
                  +-----------------------------------+
```

#### Port Connections Details:
* **Driver Outputs to Vehicle Inputs**:
  * Connect `steering` (Driver) -> `steering` (Vehicle).
  * Connect `braking` (Driver) -> `braking` (Vehicle).
* **Throttle to Spindle Torques (Drivetrain Mapping)**:
  * Since the vehicle's internal drivetrain is disconnected, the Driver's `throttle` output must be scaled to motor torques in Simulink.
  * Drag a **Gain** block from the Math Operations library. Set the gain value to `MaxMotorTorque` (defined in the workspace as `500.0` or custom).
  * Multiply `throttle` by the Gain block.
  * Connect the output of the Gain block to the torque inputs of the vehicle.
    * *For All-Wheel Drive (AWD)*: Distribute the torque by feeding the gain output (divided by 4) to `torque_FL`, `torque_FR`, `torque_RL`, and `torque_RR`.
    * *For Rear-Wheel Drive (RWD)*: Feed the gain output (divided by 2) to `torque_RL` and `torque_RR`. Feed `0` to `torque_FL` and `torque_FR`.
* **Vehicle Outputs to Driver Inputs**:
  * Connect `ref_frame.pos.x`, `ref_frame.pos.y`, `ref_frame.pos.z` (Vehicle) -> `ref_frame.pos.x`, `ref_frame.pos.y`, `ref_frame.pos.z` (Driver).
  * Connect `ref_frame.rot.e0`, `ref_frame.rot.e1`, `ref_frame.rot.e2`, `ref_frame.rot.e3` (Vehicle) -> `ref_frame.rot.e0`, `ref_frame.rot.e1`, `ref_frame.rot.e2`, `ref_frame.rot.e3` (Driver).
  * Connect `ref_frame.pos_dt.x`, `ref_frame.pos_dt.y`, `ref_frame.pos_dt.z` (Vehicle) -> `ref_frame.pos_dt.x`, `ref_frame.pos_dt.y`, `ref_frame.pos_dt.z` (Driver).
  * Connect `ref_frame.rot_dt.e0`, `ref_frame.rot_dt.e1`, `ref_frame.rot_dt.e2`, `ref_frame.rot_dt.e3` (Vehicle) -> `ref_frame.rot_dt.e0`, `ref_frame.rot_dt.e1`, `ref_frame.rot_dt.e2`, `ref_frame.rot_dt.e3` (Driver).
* **Target Speed Input**:
  * Feed a **Constant** block (set to variable `target_speed`) into the Driver's `target_speed` input port.

### Step 6: Configure Solver
1. In Simulink, open **Model Settings** (Ctrl+E).
2. Under **Solver**:
   - Set **Type** to `Fixed-step`.
   - Set **Solver** to `discrete (no continuous states)` or `Runge-Kutta`.
   - Set **Fixed-step size** to `step_size` (defined as `1e-3` / 1 millisecond).
3. Under **Simulation**:
   - Set the **Stop time** to `simulation_stop_time` (e.g. `15` seconds).

---

## 🛠️ Manipulating Parameters via JSON Files

You can modify internal vehicle and tire properties using either of the following two options:

### Option A: Edit files inside the `.fmu` package (Recommended for clean packaging)
An `.fmu` file is simply a standard ZIP archive. 
1. Rename the file extension from `.fmu` to `.zip` (e.g., `FMU2cs_WheeledVehicle4Torques.zip`).
2. Unpack it or open it directly with a ZIP utility (like 7-Zip, WinRAR, or Windows Explorer).
3. Navigate to the `resources/` folder where you will find the JSON files:
   - `Vehicle.json`: Defines chassis mass, center of gravity, locations of joints, and links to steering/suspension JSONs.
   - `sedan/tire/Sedan_Pac02Tire.json` & `.tir`: Specifies tire dimensions and standard Magic Formula parameters (Pacejka coefficients like `pCx1`, `pDx1`, `pEx1`, etc.).
   - `sedan/tire/Sedan_TMeasyTire.json`: Specifies TMeasy tire properties (vertical stiffness, damping, slip limits, peak force coordinates).
4. Edit the parameter values in a text editor (e.g. VS Code, Notepad++).
5. Zip the contents back up and rename the extension back to `.fmu`.

### Option B: Reference external JSON files (Recommended for rapid testing)
If you do not want to repack the FMU repeatedly:
1. Copy the default JSON files from the FMU resources to any local folder on your computer (e.g., `C:/MyVehicleProject/`).
2. Modify the parameters in these copied files.
3. In Simulink, double-click the Vehicle FMU import block, go to the parameters list, and set the **`vehicle_JSON`** or **`tire_JSON`** parameter to the **absolute path** of your modified file (e.g., `'C:/MyVehicleProject/MyTire.json'`).
   * *Note*: You must use absolute paths with forward slashes (e.g. `C:/MyProject/Tire.json`) so the FMU can find them at runtime.

---

## 🛣️ Generating Custom 3D Road Meshes (.obj)

Chrono's rigid terrain block performs physical 3D contact detection directly against the triangles of a Wavefront `.obj` file. You can generate these meshes from MATLAB.

### Method: Exporting from MATLAB
You can generate a grid of $X$ (length), $Y$ (width), and $Z$ (elevation) coordinates in MATLAB, triangulate it, and write it out as a mesh:
1. Define your grid:
   ```matlab
   [X, Y] = meshgrid(0:0.1:100, -2:0.1:2);
   Z = calculate_road_elevation(X, Y); % Define your bumps here
   ```
2. Convert the grid to a triangulated mesh (faces and vertices):
   ```matlab
   fv = surf2patch(X, Y, Z, 'triangles');
   ```
3. Export the vertices (`fv.vertices`) and faces (`fv.faces`) to a Wavefront `.obj` file. You can write a basic exporter script or use open-source utilities like `objwrite.m` available on MATLAB File Exchange.

---

## 📊 Road Roughness (ISO 8608) & OpenCRG Configuration

The precompiled vehicle FMU has **OpenCRG statically linked** directly into its binary DLL. It can load OpenCRG (`.crg`) road files natively without requiring any external libraries or installation steps on your colleagues' machines.

### 1. Setting Up OpenCRG in Simulink
1. In the `fmu_parameters.m` file, set:
   - `terrain_type = 2;`
   - `terrain_crg_file = 'C:/Path/To/Your/road_surface.crg';` *(Note: Must be an absolute path with forward slashes).*
2. When the simulation starts, the vehicle's tires will automatically perform elevation and normal vector lookups on the curved/sloped CRG surface.

### 2. How to Generate `.crg` Files in MATLAB
To generate a `.crg` file representing your road surface profile (including ISO 8608 roughness):
1. **Download the OpenCRG MATLAB API**: Download the free, open-source library from [OpenCRG.org](http://opencrg.org) and add it to your MATLAB path.
2. **Define Centerline Reference Path**: Define the centerline of your road in MATLAB (using coordinates or curvature $\kappa$):
   ```matlab
   % Example: Winding road centerline
   % Define longitudinal positions (u) and curvatures (kp)
   u  = [0  10  50  100  150];
   kp = [0   0  0.02  0.02  0]; % 0.02 curvature = 50m radius curve
   ```
3. **Define Your ISO 8608 Roughness Grid**: Use your existing road surface generator to compute a 2D matrix of elevation heights $Z(u, v)$ where $u$ is the along-track spacing and $v$ is the lateral spacing (across the road width).
4. **Assemble the CRG Data Structure**:
   ```matlab
   % Initialize a default CRG structure
   crg = crg_create_default();
   
   % Assign reference line curvature
   crg.rx = u;
   crg.rk = kp;
   
   % Assign your 2D elevation grid (Z matrix)
   % Let crg.vg be the M x N matrix of heights
   crg.vg = Z; 
   crg.ur = [0, 150, 0.05]; % [start, end, increment] longitudinal
   crg.vr = [-2, 2, 0.05];   % [left, right, increment] lateral
   ```
5. **Verify and Save the CRG file**:
   ```matlab
   % Perform validation checks on the data structure
   crg = crg_check(crg);
   
   % Write the binary CRG file
   crg_write(crg, 'C:/Path/To/Your/road_surface.crg');
   ```
6. Point the Vehicle FMU `terrain_crg_file` parameter to this output file.

---

## 🛠️ Advanced Configurations

### 1. Switching Tire Models (Pacejka vs TMeasy)
* **Pacejka Magic Formula (Default)**: In `fmu_parameters.m`, set `tire_JSON = 'sedan/tire/Sedan_Pac02Tire.json'`.
* **TMeasy Tire**: Set `tire_JSON = 'sedan/tire/Sedan_TMeasyTire.json'`.

### 2. Lateral Controller Selection (PID vs Stanley)
* **PID Controller**: Set `steering_type = 0` in `fmu_parameters.m`. Configure steering responsiveness using `look_ahead_dist` and `Kp_steering`.
* **Stanley Controller**: Set `steering_type = 1` in `fmu_parameters.m`. Tune performance using `stanley_dead_zone` and `Kp_steering`.

---

## 🔍 Troubleshooting & Best Practices

* **Simulation Instability / Blow-ups**: If the vehicle behaves erratically, it is usually due to the solver step size being too large for the tire stiffness. Reduce `step_size` in `fmu_parameters.m` (and the Simulink fixed step size) to `2e-4` (0.2 ms) to stabilize the contacts.
* **Paths**: The FMUs unpack their internal resources into temporary folders upon instantiation. For custom assets (like custom path files or road meshes), always use **absolute file paths** (e.g. `C:/my_folder/my_path.txt`) to ensure the FMU can find them regardless of MATLAB's current folder.
* **Runtime Visualization**: If you want to view the vehicle simulation in real-time, double click the FMU blocks and tick the **Visible** checkbox in the block instantiation settings. (Note: This opens an interactive window using Chrono's built-in visualization engine).
