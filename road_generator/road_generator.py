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
    t_end = 15.0
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
    # Grid definition (Nyquist compliant: spacing <= 0.25 m)
    u_length = 400.0
    du = 0.2
    u_grid = np.arange(0.0, u_length + du/2, du)
    
    v_right = -6.0
    v_left = 6.0
    dv = 0.2
    v_grid = np.arange(v_right, v_left + dv/2, dv)
    
    Nu = len(u_grid)
    Nv = len(v_grid)
    
    # 1. Path Heading and Reference Line Integration
    # Path equations
    target_speed = 80.0 / 3.6  # 22.2222 m/s
    t_start = 2.0
    t_duration = 5.0
    t_end = 25.0  # long enough to cover 400m
    width = 5.0
    
    # Numerically integrate arc length u(t) along the path
    dt_int = 0.001
    t_int = np.arange(0.0, t_end, dt_int)
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
    t_int = np.insert(t_int, 0, 0.0)
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
    
    # 3. Global Coordinates and Vectorized Elevation Evaluation
    # x_grid and y_grid of shape (Nu, Nv)
    x_grid = x_ref[:, np.newaxis] - v_grid[np.newaxis, :] * np.sin(phi_grid[:, np.newaxis])
    y_grid = y_ref[:, np.newaxis] + v_grid[np.newaxis, :] * np.cos(phi_grid[:, np.newaxis])
            
    # Vectorized wave accumulation (looping waves over Nu x Nv matrix)
    z_grid = np.zeros((Nu, Nv))
    for w_idx in range(len(amps)):
        z_grid += amps[w_idx] * np.cos(kx[w_idx] * x_grid + ky[w_idx] * y_grid + phis[w_idx])
            
    # Write OpenCRG ASCII LRFI file
    crg_file = os.path.join(ROAD_GEN_DIR, "default_road.crg")
    with open(crg_file, "w") as f:
        # Header block
        f.write("$ROAD_CRG\n")
        f.write(f"reference_line_start_u    =  0.0\n")
        f.write(f"reference_line_end_u      =  {u_length:.1f}\n")
        f.write(f"reference_line_increment  =  {du:.1f}\n")
        f.write(f"long_section_v_right      =  {v_right:.2f}\n")
        f.write(f"long_section_v_left       =  {v_left:.2f}\n")
        f.write(f"long_section_v_increment  =  {dv:.2f}\n")
        f.write(f"reference_line_start_x    =  0.0\n")
        f.write(f"reference_line_start_y    =  0.0\n")
        f.write(f"reference_line_start_phi  =  0.0\n")
        f.write("$\n")
        
        # Options block
        f.write("$ROAD_CRG_OPTS\n")
        f.write("refline_continuation = 1.0\n")
        f.write("$\n")
        
        # Data definition block
        f.write("$KD_DEFINITION\n")
        f.write("#:LRFI\n")
        f.write(f"U:reference line u,m,0.000,{du:.3f}\n")
        f.write("D:reference line phi,rad\n")
        f.write("D:reference line slope,m/m\n")
        f.write("D:reference line banking,m/m\n")
        for col in range(Nv):
            f.write(f"D:long section {col+1},m\n")
        f.write("$\n")
        f.write("$$$$\n")
        
        # Data block
        for i, u in enumerate(u_grid):
            # Row header: phi = phi_grid[i], slope = 0, banking = 0
            row_data = [phi_grid[i], 0.0, 0.0]
            # Grid elevations
            row_data.extend(z_grid[i, :])
            
            # Format each value to exactly 10 characters
            formatted_vals = []
            for val in row_data:
                s = f"{val:10.7f}"
                if len(s) > 10:
                    s = s[:10]
                elif len(s) < 10:
                    s = s.rjust(10)
                formatted_vals.append(s)
                
            # Write in chunks of 8 values per line
            for chunk_idx in range(0, len(formatted_vals), 8):
                chunk = formatted_vals[chunk_idx:chunk_idx+8]
                f.write("".join(chunk) + "\n")
            
    print(f"Generated OpenCRG road file at {crg_file}")
    
    # Copy to vehicle resources
    dest = os.path.join(VEHICLE_RES_DIR, "default_road.crg")
    shutil.copy(crg_file, dest)
    print(f"Copied OpenCRG road to {dest}")

if __name__ == "__main__":
    generate_bezier_path()
    generate_crg_road()
