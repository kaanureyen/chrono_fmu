import os
import shutil
import numpy as np

# Directory setup
BASE_DIR = r"c:\Users\novo\.gemini\antigravity\scratch\chrono_fmus"
ROAD_GEN_DIR = os.path.join(BASE_DIR, "road_generator")
DRIVER_RES_DIR = os.path.join(BASE_DIR, "fmu2_path_follower_driver", "resources")
VEHICLE_RES_DIR = os.path.join(BASE_DIR, "fmu2_wheeled_vehicle_4torques", "resources")

os.makedirs(ROAD_GEN_DIR, exist_ok=True)
os.makedirs(DRIVER_RES_DIR, exist_ok=True)
os.makedirs(VEHICLE_RES_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. Generate Bezier Lane Change Path
# -----------------------------------------------------------------------------
def generate_bezier_path():
    target_speed = 80.0 / 3.6  # 22.2222 m/s
    t_start = 2.0
    t_duration = 5.0
    t_end = 10.0
    dt = 0.25
    width = 5.0
    
    times = np.arange(0.0, t_end + dt/2, dt)
    points = []
    
    for t in times:
        x = target_speed * t
        if t < t_start:
            y = 0.0
        elif t > t_start + t_duration:
            y = width
        else:
            tau = (t - t_start) / t_duration
            # 7th order smooth step
            S = 35 * (tau**4) - 84 * (tau**5) + 70 * (tau**6) - 20 * (tau**7)
            y = width * S
        z = 0.1  # Height matching default ISO path
        points.append((x, y, z))
        
    path_file = os.path.join(ROAD_GEN_DIR, "default_lane_change_path.txt")
    with open(path_file, "w") as f:
        f.write(f"{len(points)} 3\n")
        for p in points:
            f.write(f"  {p[0]:12.6f}  {p[1]:12.6f}  {p[2]:12.6f}\n")
    print(f"Generated Bezier path file with {len(points)} points at {path_file}")
    
    # Copy to driver resources
    dest = os.path.join(DRIVER_RES_DIR, "default_lane_change_path.txt")
    shutil.copy(path_file, dest)
    print(f"Copied Bezier path to {dest}")

# -----------------------------------------------------------------------------
# 2. Generate OpenCRG Road Surface (with ISO 8608 Class C Roughness)
# -----------------------------------------------------------------------------
def get_I(alpha):
    t = np.linspace(-2000, 2000, 200000)
    dt = t[1] - t[0]
    return np.sum((1.0 + t**2)**(-alpha/2.0)) * dt

def generate_crg_road():
    # Grid definition (0.02m resolution)
    u_length = 245.0
    du = 0.02
    u_grid = np.arange(0.0, u_length + du/2, du)
    
    v_right = -2.0
    v_left = 2.0
    dv = 0.02
    v_grid = np.arange(v_right, v_left + dv/2, dv)
    
    Nu = len(u_grid)
    Nv = len(v_grid)
    
    # 1. Path Heading and Reference Line Integration
    # Path equations
    target_speed = 80.0 / 3.6  # 22.2222 m/s
    t_start = 0.5
    t_duration = 5.0
    t_end = 7.0  # long enough to cover 400m
    width = 5.0
    
    # Numerically integrate arc length u(t) along the path starting from t = -0.5s
    dt_int = 0.001
    t_int = np.arange(-0.5, t_end, dt_int)
    dx_dt = np.full_like(t_int, target_speed)
    dy_dt = np.zeros_like(t_int)
    
    for idx, t in enumerate(t_int):
        if t_start <= t <= t_start + t_duration:
            tau = (t - t_start) / t_duration
            # dS/dtau = 140*tau^3 - 420*tau^4 + 420*tau^5 - 140*tau^6
            dS_dtau = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt[idx] = (width / t_duration) * dS_dtau
            
    ds_int = np.sqrt(dx_dt**2 + dy_dt**2) * dt_int
    u_int = np.cumsum(ds_int)
    # Insert 0 at start
    u_int = np.insert(u_int, 0, 0.0)
    t_int = np.insert(t_int, 0, -0.5)
    dx_dt = np.insert(dx_dt, 0, target_speed)
    dy_dt = np.insert(dy_dt, 0, 0.0)
    
    # Interpolate time, heading, and coordinates for each grid station u_k
    t_grid = np.interp(u_grid, u_int, t_int)
    
    # Compute reference line heading phi and position (x, y)
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
        x_ref[i] = target_speed * t

    # 2. Roughness Parameters (ISO 8608 Class C)
    seed = 42
    Gd_n0 = 256e-6  # Class C
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
    
    # Build wave components
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
    
    # 3. Global Coordinates and Elevation Evaluation (Offloaded to C++)
    # Write parameters to a temporary file
    temp_input_file = os.path.join(ROAD_GEN_DIR, "temp_input.txt")
    crg_file = os.path.join(ROAD_GEN_DIR, "default_road.crg")
    
    with open(temp_input_file, "w") as f:
        f.write(f"{Nu} {Nv} {len(amps)} {u_length:.3f} {du:.6f} {v_right:.6f} {v_left:.6f} {dv:.6f}\n")
        # Reference line stations
        for i in range(Nu):
            f.write(f"{phi_grid[i]:.10f} {x_ref[i]:.10f} {y_ref[i]:.10f}\n")
        # Wave components
        for w in range(len(amps)):
            f.write(f"{amps[w]:.10f} {kx[w]:.10f} {ky[w]:.10f} {phis[w]:.10f}\n")
            
    print(f"Temporary input file written to {temp_input_file}. Invoking C++ generator...")
    
    exe_path = os.path.join(ROAD_GEN_DIR, "generate_road.exe")
    import subprocess
    subprocess.run([exe_path, temp_input_file, crg_file], check=True)
    
    # Remove temporary file
    if os.path.exists(temp_input_file):
        os.remove(temp_input_file)
        
    print(f"Generated OpenCRG road file at {crg_file}")
    
    # Copy to vehicle resources
    dest = os.path.join(VEHICLE_RES_DIR, "default_road.crg")
    shutil.copy(crg_file, dest)
    print(f"Copied OpenCRG road to {dest}")

if __name__ == "__main__":
    generate_bezier_path()
    generate_crg_road()
