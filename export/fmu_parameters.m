% =========================================================================
% MATLAB Simulink Co-Simulation Parameter Initialization Script
% Project: Chrono Wheeled Vehicle + Stanley Path-Follower Driver FMUs
% =========================================================================
% Run this script before starting the Simulink model to load all parameters
% into the MATLAB workspace. You can then reference these variable names
% directly in the FMU Import block parameters.

clear;
clc;

% Get absolute path of this script's directory with forward slashes
current_dir = strrep(fileparts(mfilename('fullpath')), '\', '/');

fprintf('Initializing vehicle and driver FMU parameters...\n');

%% 1. Solver & Simulation Settings
step_size = 1e-3;            % Fixed-step size for co-simulation (s)
simulation_stop_time = 15;   % Stop time of simulation (s)

%% 2. Vehicle Configuration
init_loc_x = 0.0;            % Initial X position (m)
init_loc_y = 0.0;            % Initial Y position (m)
init_loc_z = 0.5;            % Initial Z position (m) (Keep above ground to avoid contact issues on startup)
init_yaw   = 0.0;            % Initial yaw heading angle (rad)

% Contact formulation
system_SMC = true;           % true: SMC (penalty), false: NSC (rigid contact)

% Terrain configuration
terrain_friction = 0.8;      % Friction coefficient
terrain_type = 0;            % 0: FlatTerrain, 1: RigidTerrain (.obj mesh), 2: OpenCRG (.crg)
terrain_mesh_file = strrep(fullfile(current_dir, 'test_terrain.obj'), '\', '/');      % OBJ mesh path (required only if terrain_type = 1)
terrain_crg_file = strrep(fullfile(current_dir, 'circle_100m_left.crg'), '\', '/');   % OpenCRG file path (required only if terrain_type = 2)
% Note: When switching terrain_type, verify init_loc_z:
%   - For Flat/Mesh terrains: init_loc_z = 0.2 (chassis spawn at 0.7m, tire bottom at 0.0m)
%   - For circle_100m_left.crg: init_loc_z = 0.1 (chassis spawn at 0.6m, tire bottom at 0.0m)

% Tire selection (relative paths resolved internally inside FMU resources)
% Option A: Pacejka Magic Formula (Default)
tire_JSON = 'sedan/tire/Sedan_Pac02Tire.json';
% Option B: TMeasy Tire Model
% tire_JSON = 'sedan/tire/Sedan_TMeasyTire.json';

%% 3. Driver & Controller Configuration
target_speed = 10.0;         % Cruise control speed target (m/s)

% Path definition file (must be absolute path with forward slashes)
path_file = strrep(fullfile(current_dir, 'ISO_double_lane_change.txt'), '\', '/');

% Steering Controller Type
% 0: PID Path-Follower
% 1: Stanley lateral tracker
steering_type = 1; 

% Lateral Controller parameters (PID)
look_ahead_dist = 5.0;       % Look ahead distance (m)
Kp_steering     = 0.8;       % Proportional gain
Ki_steering     = 0.0;       % Integral gain
Kd_steering     = 0.0;       % Derivative gain

% Lateral Controller parameters (Stanley)
stanley_dead_zone    = 0.05; % Cross-track error dead zone (m)
max_wheel_turn_angle = 0.52; % Regular Ackermann steering limit (~30 deg in rad)

% Cruise Control parameters (PID)
Kp_speed = 0.4;              % Proportional gain
Ki_speed = 0.0;              % Integral gain
Kd_speed = 0.0;              % Derivative gain

%% 4. Drivetrain Power & Torque Distribution
% Since the driveline is disconnected inside the vehicle FMU to allow
% direct torque inputs, you can map the driver's throttle output to 4 spindle torques.
MaxMotorTorque = 500.0;      % Maximum torque per wheel (Nm)

fprintf('Parameters loaded successfully.\n');
fprintf('Default Configuration:\n');
fprintf('  - Lateral Controller: Stanley (steering_type = 1)\n');
fprintf('  - Tire Formulation:   Pacejka (Magic Formula)\n');
fprintf('  - Target Speed:       %d m/s\n', target_speed);
fprintf('Now open your Simulink model containing the FMUs and run the simulation.\n');
