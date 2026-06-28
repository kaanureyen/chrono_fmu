import os
import shutil
import numpy as np
import sys

# Configurable Flag for OBJ Mesh Generation
GENERATE_OBJ = True

def get_I(alpha):
    t = np.linspace(-2000, 2000, 200000)
    dt = t[1] - t[0]
    return np.sum((1.0 + t**2)**(-alpha/2.0)) * dt

def generate_road_profile(
    target_speed_kph=60.0,
    t_start=2.0,
    t_duration=5.0,
    t_end=13.0,
    width=5.0,
    start_length_margin=20.0,  # extra terrain generated before spawn point
    end_length_margin=50.0,    # extra terrain generated after end of run
    mesh_resolution=0.06,      # grid resolution (du, dv)
    v_width=8.0,
    iso_class='C',
    generate_obj=True,
    base_dir=None
):
    """
    Refactored, fully parameterized road profile generator.
    Calculates safety margins behind vehicle spawn point dynamically based on target speed.
    """
    if base_dir is None:
        # SCRIPT_DIR = src/road_generator
        # BASE_DIR = root chrono_fmus
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.dirname(os.path.dirname(script_dir))

    road_gen_dir = os.path.join(base_dir, "src", "road_generator")
    output_dir = os.path.join(base_dir, "build", "generated")
    os.makedirs(output_dir, exist_ok=True)

    # 1. Spawning Height & Speed conversion
    target_speed = target_speed_kph / 3.6  # Convert km/h to m/s
    if target_speed <= 0:
        raise ValueError("Target speed must be positive.")

    # Calculate total traveled distance and road length dynamically
    d_travel = target_speed * t_end
    u_length = d_travel + start_length_margin + end_length_margin
    du = mesh_resolution
    dv = mesh_resolution

    # Dynamic calculation of starting points behind vehicle
    t_start_crg = -start_length_margin / target_speed
    t_end_crg = t_end + end_length_margin / target_speed
    t_start_path = 0.0  # Vehicle spawns at t = 0.0 (x = start_length_margin)
    t_end_path = t_end + end_length_margin / target_speed

    # 2. Generate Bezier Lane Change Path
    dt_path = 0.25
    times_path = np.arange(t_start_path, t_end_path + dt_path/2, dt_path)
    points = []
    
    for t in times_path:
        x = target_speed * t + start_length_margin
        if t < t_start:
            y = 0.0
        elif t > t_start + t_duration:
            y = width
        else:
            tau = (t - t_start) / t_duration
            S = 35 * (tau**4) - 84 * (tau**5) + 70 * (tau**6) - 20 * (tau**7)
            y = width * S
        z = 0.2  # Spawning height (Z = 0.2)
        points.append((x, y, z))
        
    path_file = os.path.join(output_dir, "default_lane_change_path.txt")
    with open(path_file, "w") as f:
        f.write(f"{len(points)} 3\n")
        for p in points:
            f.write(f"  {p[0]:12.6f}  {p[1]:12.6f}  {p[2]:12.6f}\n")
    print(f"Generated Bezier path file with {len(points)} points at {path_file}")

    # 3. Generate OpenCRG Grid
    v_right = -v_width / 2.0
    v_left = v_width / 2.0
    u_grid = np.arange(0.0, u_length + du/2, du)
    
    Nu = len(u_grid)
    Nv = len(np.arange(v_right, v_left + dv/2, dv))

    # Integrate reference line station arc-length starting from t_start_crg
    dt_int = 0.001
    t_int = np.arange(t_start_crg, t_end_crg, dt_int)
    dx_dt = np.full_like(t_int, target_speed)
    dy_dt = np.zeros_like(t_int)
    
    for idx, t in enumerate(t_int):
        if t_start <= t <= t_start + t_duration:
            tau = (t - t_start) / t_duration
            dS_dtau = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt[idx] = (width / t_duration) * dS_dtau
            
    ds_int = np.sqrt(dx_dt**2 + dy_dt**2) * dt_int
    u_int = np.cumsum(ds_int)
    u_int = np.insert(u_int, 0, 0.0)
    t_int = np.insert(t_int, 0, t_start_crg)
    dx_dt = np.insert(dx_dt, 0, target_speed)
    dy_dt = np.insert(dy_dt, 0, 0.0)
    
    t_grid = np.interp(u_grid, u_int, t_int)
    
    phi_grid = np.zeros(Nu)
    x_ref = np.zeros(Nu)
    y_ref = np.zeros(Nu)
    
    for i, t in enumerate(t_grid):
        if t < t_start:
            y_ref[i] = 0.0
            phi_grid[i] = 0.0
        elif t > t_start + t_duration:
            y_ref[i] = width
            phi_grid[i] = 0.0
        else:
            tau = (t - t_start) / t_duration
            S = 35 * (tau**4) - 84 * (tau**5) + 70 * (tau**6) - 20 * (tau**7)
            y_ref[i] = width * S
            dS_dtau = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt_t = (width / t_duration) * dS_dtau
            phi_grid[i] = np.arctan2(dy_dt_t, target_speed)
        x_ref[i] = target_speed * t + start_length_margin

    # 4. Roughness parameters (ISO 8608 Classes A to E)
    roughness_coefficients = {
        'A': 16e-6,
        'B': 64e-6,
        'C': 256e-6,
        'D': 1024e-6,
        'E': 4096e-6
    }
    iso_class = iso_class.upper()
    if iso_class not in roughness_coefficients:
        print(f"Warning: Unknown ISO class '{iso_class}'. Defaulting to 'C'.")
        iso_class = 'C'
        
    Gd_n0 = roughness_coefficients[iso_class]
    seed = 42
    w = 2.0
    f_min = 0.01
    f_max = 2.0
    Nf = 256
    Ntheta = 16
    
    rng = np.random.RandomState(seed)
    n0 = 0.1
    C1 = Gd_n0 * (n0**w)
    alpha = w + 1.0
    I_val = get_I(alpha)
    C2 = C1 / I_val
    
    f_r = np.logspace(np.log10(f_min), np.log10(f_max), Nf + 1)
    df_r = np.diff(f_r)
    f_centers = 0.5 * (f_r[:-1] + f_r[1:])
    theta = np.linspace(0, np.pi, Ntheta, endpoint=False)
    dtheta = np.pi / Ntheta
    
    amps = []
    kx = []
    ky = []
    phis = []
    
    for i in range(Nf):
        fc = f_centers[i]
        dfc = df_r[i]
        S_2D_val = C2 * (fc**(-alpha))
        power_per_angle = S_2D_val * fc * dfc * dtheta
        amp = np.sqrt(2.0 * power_per_angle)
        
        for j in range(Ntheta):
            th = theta[j]
            phi = rng.uniform(0, 2*np.pi)
            
            amps.append(amp)
            kx.append(2.0 * np.pi * fc * np.cos(th))
            ky.append(2.0 * np.pi * fc * np.sin(th))
            phis.append(phi)
            
    amps = np.array(amps)
    kx = np.array(kx)
    ky = np.array(ky)
    phis = np.array(phis)
    
    # 5. Global Coordinates and Elevation Evaluation (Offloaded to C++)
    temp_input_file = os.path.join(output_dir, "temp_input.txt")
    crg_file = os.path.join(output_dir, "default_road.crg")
    
    with open(temp_input_file, "w") as f:
        f.write(f"{Nu} {Nv} {len(amps)} {u_length:.3f} {du:.6f} {v_right:.6f} {v_left:.6f} {dv:.6f}\n")
        for i in range(Nu):
            f.write(f"{phi_grid[i]:.10f} {x_ref[i]:.10f} {y_ref[i]:.10f}\n")
        for w in range(len(amps)):
            f.write(f"{amps[w]:.10f} {kx[w]:.10f} {ky[w]:.10f} {phis[w]:.10f}\n")
            
    print(f"Temporary input file written to {temp_input_file}. Invoking C++ generator...")
    
    exe_path = os.path.join(base_dir, "build", "generate_road.exe")
    if not os.path.exists(exe_path):
        # Fallback to local source dir lookup if build folder isn't populated
        exe_path = os.path.join(road_gen_dir, "generate_road.exe")
        
    if not os.path.exists(exe_path):
        raise FileNotFoundError(f"Compiler executable 'generate_road.exe' not found at {exe_path}. Run build_fmus.bat first.")

    import subprocess
    if generate_obj:
        obj_file = os.path.join(output_dir, "default_road.obj")
        subprocess.run([exe_path, temp_input_file, crg_file, obj_file], check=True)
        print(f"Generated road OBJ mesh at {obj_file}")
    else:
        subprocess.run([exe_path, temp_input_file, crg_file], check=True)
        
    if os.path.exists(temp_input_file):
        os.remove(temp_input_file)
        
    print(f"Generated OpenCRG road file at {crg_file}")

if __name__ == "__main__":
    # Command line usage fallback
    generate_road_profile(
        target_speed_kph=60.0,
        t_start=2.0,
        t_duration=5.0,
        t_end=13.0,
        width=5.0,
        start_length_margin=20.0,
        end_length_margin=50.0,
        mesh_resolution=0.06,
        v_width=8.0,
        iso_class='C',
        generate_obj=GENERATE_OBJ
    )
