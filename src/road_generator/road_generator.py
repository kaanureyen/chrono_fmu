import os
import shutil
import numpy as np
import sys
import json

# Configurable Flag for OBJ Mesh Generation
GENERATE_OBJ = False

def get_I(alpha):
    t = np.linspace(-2000, 2000, 200000)
    dt = t[1] - t[0]
    return np.sum((1.0 + t**2)**(-alpha/2.0)) * dt

def S_poly(tau):
    return 35 * (tau**4) - 84 * (tau**5) + 70 * (tau**6) - 20 * (tau**7)

def S_int(tau):
    return 7 * (tau**5) - 14 * (tau**6) + 10 * (tau**7) - 2.5 * (tau**8)

def evaluate_profile(u, profile_type, value, start, duration):
    if profile_type == "None" or profile_type == "Flat":
        return 0.0
    elif profile_type == "Constant":
        return value
    elif profile_type in ["Ramp", "Smooth Step"]:
        if u < start:
            return 0.0
        elif u > start + duration:
            return value
        else:
            tau = (u - start) / duration
            return value * S_poly(tau)
    return 0.0

def get_speed_at_time(t, v_init, v_target, t_speed_start, t_speed_dur):
    if t < t_speed_start:
        return v_init
    elif t > t_speed_start + t_speed_dur:
        return v_target
    else:
        if t_speed_dur <= 0:
            return v_target
        tau = (t - t_speed_start) / t_speed_dur
        return v_init + (v_target - v_init) * S_poly(tau)

def get_station_at_time(t, v_init, v_target, t_speed_start, t_speed_dur, start_length_margin):
    if t < 0:
        return v_init * t + start_length_margin
        
    if t_speed_dur <= 0:
        return v_target * t + start_length_margin

    if t < t_speed_start:
        d = v_init * t
    elif t < t_speed_start + t_speed_dur:
        d1 = v_init * t_speed_start
        tau = (t - t_speed_start) / t_speed_dur
        d = d1 + v_init * (t - t_speed_start) + (v_target - v_init) * t_speed_dur * S_int(tau)
    else:
        d1 = v_init * t_speed_start
        d2 = 0.5 * (v_init + v_target) * t_speed_dur
        d3 = v_target * (t - t_speed_start - t_speed_dur)
        d = d1 + d2 + d3
        
    return d + start_length_margin

def eval_pose_pos(time, params):
    params_base = params.copy()
    params_base["superpose_lc"] = False
    x_base, y_base, phi_base = get_base_trajectory_pose(time, params_base)
    
    t_start = params.get("lc_start_time", 2.0)
    t_dur = params.get("lc_duration", 5.0)
    width = params.get("lc_width", 5.0)
    
    if time < t_start:
        d_lc = 0.0
    elif time > t_start + t_dur:
        d_lc = width
    else:
        tau = (time - t_start) / t_dur
        d_lc = width * S_poly(tau)
        
    x = x_base - d_lc * np.sin(phi_base)
    y = y_base + d_lc * np.cos(phi_base)
    return x, y

def get_trajectory_pose(t, params):
    if params.get("superpose_lc", False):
        eps = 0.001
        x_m, y_m = eval_pose_pos(t - eps, params)
        x_c, y_c = eval_pose_pos(t, params)
        x_p, y_p = eval_pose_pos(t + eps, params)
        
        dx = (x_p - x_m) / (2.0 * eps)
        dy = (y_p - y_m) / (2.0 * eps)
        phi = np.arctan2(dy, dx)
        return x_c, y_c, phi
    else:
        return get_base_trajectory_pose(t, params)

def get_base_trajectory_pose(t, params):
    v_init = params["v_init"]
    v_target = params["v_target"]
    t_speed_start = params["t_speed_start"]
    t_speed_dur = params["t_speed_dur"]
    start_length_margin = params["start_length_margin"]
    maneuver_type = params["maneuver_type"]
    
    s = get_station_at_time(t, v_init, v_target, t_speed_start, t_speed_dur, start_length_margin)

    if maneuver_type == "Straight Line":
        return s, 0.0, 0.0

    elif maneuver_type == "Single Lane Change":
        t_start = params["t_start"]
        t_duration = params["t_duration"]
        width = params["width"]
        if t < t_start:
            return s, 0.0, 0.0
        elif t > t_start + t_duration:
            return s, width, 0.0
        else:
            tau = (t - t_start) / t_duration
            S_val = S_poly(tau)
            S_prime = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt = (width / t_duration) * S_prime
            v_curr = get_speed_at_time(t, v_init, v_target, t_speed_start, t_speed_dur)
            phi = np.arctan2(dy_dt, v_curr)
            return s, width * S_val, phi

    elif maneuver_type == "Double Lane Change":
        t_start = params["t_start"]
        t_duration = params["t_duration"]
        width = params["width"]
        dwell_time = params["dwell_time"]
        t1 = t_start
        t2 = t_start + t_duration
        t3 = t2 + dwell_time
        t4 = t3 + t_duration
        v_curr = get_speed_at_time(t, v_init, v_target, t_speed_start, t_speed_dur)

        if t < t1:
            return s, 0.0, 0.0
        elif t < t2:
            tau = (t - t1) / t_duration
            S_val = S_poly(tau)
            S_prime = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt = (width / t_duration) * S_prime
            phi = np.arctan2(dy_dt, v_curr)
            return s, width * S_val, phi
        elif t < t3:
            return s, width, 0.0
        elif t < t4:
            tau = (t - t3) / t_duration
            S_val = S_poly(tau)
            S_prime = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_dt = -(width / t_duration) * S_prime
            phi = np.arctan2(dy_dt, v_curr)
            return s, width * (1.0 - S_val), phi
        else:
            return s, 0.0, 0.0

    elif maneuver_type in ["Circular Path", "Braking in a Turn"]:
        lead_in_length = params["lead_in_length"]
        radius = params["radius"]
        direction = params["direction"]
        s_turn_start = start_length_margin + lead_in_length
        if s < s_turn_start:
            return s, 0.0, 0.0
        else:
            theta = (s - s_turn_start) / radius
            if direction == "Left":
                x = s_turn_start + radius * np.sin(theta)
                y = radius * (1.0 - np.cos(theta))
                return x, y, theta
            else:
                x = s_turn_start + radius * np.sin(theta)
                y = -radius * (1.0 - np.cos(theta))
                return x, y, -theta

    elif maneuver_type == "J-Turn":
        lead_in_length = params["lead_in_length"]
        radius = params["radius"]
        direction = params["direction"]
        s_turn_start = start_length_margin + lead_in_length
        theta_max = np.pi / 2.0  # 90 degrees
        s_turn_end = s_turn_start + radius * theta_max
        if s < s_turn_start:
            return s, 0.0, 0.0
        elif s <= s_turn_end:
            theta = (s - s_turn_start) / radius
            if direction == "Left":
                x = s_turn_start + radius * np.sin(theta)
                y = radius * (1.0 - np.cos(theta))
                return x, y, theta
            else:
                x = s_turn_start + radius * np.sin(theta)
                y = -radius * (1.0 - np.cos(theta))
                return x, y, -theta
        else:
            # Straight tangent at 90 deg
            if direction == "Left":
                x_end = s_turn_start + radius
                y_end = radius
                phi = np.pi / 2.0
                dx = 0.0
                dy = s - s_turn_end
            else:
                x_end = s_turn_start + radius
                y_end = -radius
                phi = -np.pi / 2.0
                dx = 0.0
                dy = -(s - s_turn_end)
            return x_end + dx, y_end + dy, phi

    elif maneuver_type == "U-Turn":
        lead_in_length = params["lead_in_length"]
        radius = params["radius"]
        direction = params["direction"]
        s_turn_start = start_length_margin + lead_in_length
        theta_max = np.pi  # 180 degrees
        s_turn_end = s_turn_start + radius * theta_max
        if s < s_turn_start:
            return s, 0.0, 0.0
        elif s <= s_turn_end:
            theta = (s - s_turn_start) / radius
            if direction == "Left":
                x = s_turn_start + radius * np.sin(theta)
                y = radius * (1.0 - np.cos(theta))
                return x, y, theta
            else:
                x = s_turn_start + radius * np.sin(theta)
                y = -radius * (1.0 - np.cos(theta))
                return x, y, -theta
        else:
            # Straight return parallel to start
            if direction == "Left":
                x_end = s_turn_start
                y_end = 2.0 * radius
                phi = np.pi
                dx = -(s - s_turn_end)
                dy = 0.0
            else:
                x_end = s_turn_start
                y_end = -2.0 * radius
                phi = -np.pi
                dx = -(s - s_turn_end)
                dy = 0.0
            return x_end + dx, y_end + dy, phi

    elif maneuver_type == "Slalom":
        lead_in_length = params["lead_in_length"]
        slalom_period = params["slalom_period"]
        slalom_amplitude = params["slalom_amplitude"]
        s_slalom_start = start_length_margin + lead_in_length
        if s < s_slalom_start:
            return s, 0.0, 0.0
        else:
            dx_s = s - s_slalom_start
            omega = 2.0 * np.pi / slalom_period
            
            # Smoothly fade in amplitude over the first half wavelength
            fade_len = slalom_period / 2.0
            if dx_s < fade_len:
                tau = dx_s / fade_len
                S_fade = S_poly(tau)
                y = S_fade * slalom_amplitude * np.sin(omega * dx_s)
            else:
                y = slalom_amplitude * np.sin(omega * dx_s)
                
            # Numerical derivative for heading angle
            eps = 0.001
            dx_plus = dx_s + eps
            if dx_plus < fade_len:
                tau_p = dx_plus / fade_len
                S_fade_p = S_poly(tau_p)
                y_plus = S_fade_p * slalom_amplitude * np.sin(omega * dx_plus)
            else:
                y_plus = slalom_amplitude * np.sin(omega * dx_plus)
                
            dx_minus = dx_s - eps
            if dx_minus < 0.0:
                y_minus = 0.0
            elif dx_minus < fade_len:
                tau_m = dx_minus / fade_len
                S_fade_m = S_poly(tau_m)
                y_minus = S_fade_m * slalom_amplitude * np.sin(omega * dx_minus)
            else:
                y_minus = slalom_amplitude * np.sin(omega * dx_minus)
                
            dy_ds = (y_plus - y_minus) / (2.0 * eps)
            phi = np.arctan2(dy_ds, 1.0)
            return s, y, phi

    elif maneuver_type == "Sine with Dwell":
        t_start = params["t_start"]
        t_duration = params["t_duration"]
        width = params["width"]
        dwell_time = params["dwell_time"]
        if t < t_start:
            return s, 0.0, 0.0
        else:
            dt = t - t_start
            T = t_duration # cycle duration
            v_curr = get_speed_at_time(t, v_init, v_target, t_speed_start, t_speed_dur)
            
            # Check phase
            if dt < 0.75 * T:
                y = width * np.sin(2.0 * np.pi * dt / T)
            elif dt < 0.75 * T + dwell_time:
                y = -width
            elif dt < 1.0 * T + dwell_time:
                dtau = dt - (0.75 * T + dwell_time)
                y = -width * np.cos(2.0 * np.pi * dtau / T)
            else:
                y = 0.0
                
            # Numerical derivative for heading angle
            eps = 0.001
            
            def eval_y_swd(time):
                dtime = time - t_start
                if dtime < 0.0:
                    return 0.0
                if dtime < 0.75 * T:
                    return width * np.sin(2.0 * np.pi * dtime / T)
                elif dtime < 0.75 * T + dwell_time:
                    return -width
                elif dtime < 1.0 * T + dwell_time:
                    dtau_p = dtime - (0.75 * T + dwell_time)
                    return -width * np.cos(2.0 * np.pi * dtau_p / T)
                return 0.0
                
            y_plus = eval_y_swd(t + eps)
            y_minus = eval_y_swd(t - eps)
            dy_dt = (y_plus - y_minus) / (2.0 * eps)
            phi = np.arctan2(dy_dt, v_curr)
            return s, y, phi

    elif maneuver_type == "Fishhook":
        lead_in_length = params["lead_in_length"]
        radius = params["radius"]
        radius2 = params["radius2"]
        direction = params["direction"]
        theta1 = params["theta1"]
        theta2 = params["theta2"]
        s_turn_start = start_length_margin + lead_in_length
        th1_rad = theta1 * np.pi / 180.0
        th2_rad = theta2 * np.pi / 180.0
        
        L_arc1 = radius * th1_rad
        L_arc2 = radius2 * th2_rad
        s_arc1_end = s_turn_start + L_arc1
        s_arc2_end = s_arc1_end + L_arc2
        
        if s < s_turn_start:
            return s, 0.0, 0.0
        elif s <= s_arc1_end:
            theta = (s - s_turn_start) / radius
            if direction == "Left":
                x = s_turn_start + radius * np.sin(theta)
                y = radius * (1.0 - np.cos(theta))
                return x, y, theta
            else:
                x = s_turn_start + radius * np.sin(theta)
                y = -radius * (1.0 - np.cos(theta))
                return x, y, -theta
        elif s <= s_arc2_end:
            # Turn 1 end pose
            x2 = s_turn_start + radius * np.sin(th1_rad)
            if direction == "Left":
                y2 = radius * (1.0 - np.cos(th1_rad))
                phi2 = th1_rad
            else:
                y2 = -radius * (1.0 - np.cos(th1_rad))
                phi2 = -th1_rad
                
            # Second turn (opposite direction) center
            if direction == "Left":
                # Turn 1 Left -> Turn 2 Right
                x_c = x2 + radius2 * np.sin(phi2)
                y_c = y2 - radius2 * np.cos(phi2)
                delta_theta = (s - s_arc1_end) / radius2
                phi = phi2 - delta_theta
                x = x_c - radius2 * np.sin(phi)
                y = y_c + radius2 * np.cos(phi)
                return x, y, phi
            else:
                # Turn 1 Right -> Turn 2 Left
                x_c = x2 - radius2 * np.sin(phi2)
                y_c = y2 + radius2 * np.cos(phi2)
                delta_theta = (s - s_arc1_end) / radius2
                phi = phi2 + delta_theta
                x = x_c + radius2 * np.sin(phi)
                y = y_c - radius2 * np.cos(phi)
                return x, y, phi
        else:
            # Tangent straight return
            x2 = s_turn_start + radius * np.sin(th1_rad)
            if direction == "Left":
                y2 = radius * (1.0 - np.cos(th1_rad))
                phi2 = th1_rad
                x_c = x2 + radius2 * np.sin(phi2)
                y_c = y2 - radius2 * np.cos(phi2)
                phi3 = phi2 - th2_rad
                x3 = x_c - radius2 * np.sin(phi3)
                y3 = y_c + radius2 * np.cos(phi3)
            else:
                y2 = -radius * (1.0 - np.cos(th1_rad))
                phi2 = -th1_rad
                x_c = x2 - radius2 * np.sin(phi2)
                y_c = y2 + radius2 * np.cos(phi2)
                phi3 = phi2 + th2_rad
                x3 = x_c - radius2 * np.sin(phi3)
                y3 = y_c - radius2 * np.cos(phi3)
                
            dx_s = s - s_arc2_end
            x = x3 + dx_s * np.cos(phi3)
            y = y3 + dx_s * np.sin(phi3)
            return x, y, phi3

    elif maneuver_type == "ISO 3888-2 Obstacle Avoidance":
        lead_in_length = params["lead_in_length"]
        vehicle_width = params["vehicle_width"]
        s_0 = start_length_margin + lead_in_length
        s1 = s_0 + 12.0
        s2 = s1 + 13.5
        s3 = s2 + 11.0
        s4 = s3 + 12.5
        s5 = s4 + 12.0
        W_offset = 1.05 * vehicle_width + 1.625

        if s < s1:
            return s, 0.0, 0.0
        elif s < s2:
            tau = (s - s1) / 13.5
            S_val = S_poly(tau)
            S_prime = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_ds = (W_offset / 13.5) * S_prime
            phi = np.arctan2(dy_ds, 1.0)
            return s, W_offset * S_val, phi
        elif s < s3:
            return s, W_offset, 0.0
        elif s < s4:
            tau = (s - s3) / 12.5
            S_val = S_poly(tau)
            S_prime = 140.0 * (tau**3) - 420.0 * (tau**4) + 420.0 * (tau**5) - 140.0 * (tau**6)
            dy_ds = -(W_offset / 12.5) * S_prime
            phi = np.arctan2(dy_ds, 1.0)
            return s, W_offset * (1.0 - S_val), phi
        else:
            return s, 0.0, 0.0

    elif maneuver_type == "Constant Speed Spiral":
        lead_in_length = params["lead_in_length"]
        clothoid_a = params["clothoid_a"]
        direction = params["direction"]
        s_clothoid = params["s_clothoid"]
        x_clothoid = params["x_clothoid"]
        y_clothoid = params["y_clothoid"]
        s_turn_start = start_length_margin + lead_in_length
        if s < s_turn_start:
            return s, 0.0, 0.0
        else:
            ds_s = s - s_turn_start
            x_offset = np.interp(ds_s, s_clothoid, x_clothoid)
            y_offset = np.interp(ds_s, s_clothoid, y_clothoid)
            phi = 0.5 * clothoid_a * ds_s**2
            if direction == "Right":
                y_offset = -y_offset
                phi = -phi
            return s_turn_start + x_offset, y_offset, phi

    return s, 0.0, 0.0

def generate_road_profile(
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
    generate_obj=False,
    base_dir=None,
    
    # Maneuver parameters
    maneuver_type="Single Lane Change",
    dwell_time=2.0,
    radius=50.0,
    direction="Left",
    lead_in_length=20.0,
    
    # Extended Maneuver parameters
    radius2=20.0,
    slalom_period=30.0,
    slalom_amplitude=2.0,
    theta1=45.0,
    theta2=180.0,
    
    # New Maneuver parameters
    clothoid_a=0.0005,
    vehicle_width=2.0,
    
    # New visual, packaging, and filtering parameters
    curvature_filter_window_m=15.0,
    superpose_lc=False,
    lc_start_time=2.0,
    lc_duration=5.0,
    lc_width=5.0,
    terrain_diffuse_texture="",
    terrain_normal_texture="",
    terrain_show_visual_lines=False,
    terrain_crg_simplify=True,
    
    # Slope & Banking parameters
    slope_type="None",
    slope_value=0.0,
    slope_start=30.0,
    slope_duration=20.0,
    
    banking_type="None",
    banking_value=0.0,
    banking_start=30.0,
    banking_duration=20.0,
    
    # Road condition parameter
    mu_value=0.85,
    
    # Speed Profile parameters
    speed_profile_time_start=2.0,
    speed_profile_duration=5.0,
    speed_profile_initial_speed=60.0,
    speed_profile_target_speed=60.0,
    
    # Controller gains for Matlab exporter
    steering_type=1,
    look_ahead_dist=3.615358,
    Kp_steering=2.398832,
    Ki_steering=0.0,
    Kd_steering=0.0,
    stanley_dead_zone=0.010965,
    max_wheel_turn_angle=25.0,
    Kp_speed=0.868900,
    Ki_speed=0.436516,
    Kd_speed=0.0,

    # Roughness Discretization parameters
    roughness_Nf=512,
    roughness_Ntheta=32
):
    """
    Refactored, fully parameterized road profile generator.
    Supports various maneuvers, slope & banking profiles, speed profiles, and exports Matlab setup files.
    """
    if mesh_resolution <= 0.0:
        raise ValueError("Mesh resolution must be strictly positive (e.g., 0.06).")
    if v_width <= 0.0:
        raise ValueError("Road width must be strictly positive.")
    if t_end <= 0.0:
        raise ValueError("Simulation end time must be positive.")
    if abs(banking_value) > 1.0:
        raise ValueError("Banking slope value must be between -1.0 and 1.0 (m/m).")
    if abs(slope_value) > 1.0:
        raise ValueError("Slope value must be between -1.0 and 1.0 (m/m).")

    if base_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.dirname(os.path.dirname(script_dir))

    road_gen_dir = os.path.join(base_dir, "src", "road_generator")
    output_dir = os.path.join(base_dir, "build", "generated")
    os.makedirs(output_dir, exist_ok=True)

    v_init = speed_profile_initial_speed / 3.6  # Convert km/h to m/s
    v_target = speed_profile_target_speed / 3.6  # Convert km/h to m/s
    
    if v_init <= 0 or v_target <= 0:
        raise ValueError("Initial and target speeds must be positive.")

    # Time bounds for CRG reference line
    t_start_crg = -start_length_margin / v_init
    t_speed_start = speed_profile_time_start
    t_speed_dur = speed_profile_duration

    # 1. Speed Profile Integration Parameters
    # 2. Precompute clothoid arrays if maneuver is Constant Speed Spiral
    s_clothoid = None
    x_clothoid = None
    y_clothoid = None
    if maneuver_type == "Constant Speed Spiral":
        s_clothoid = np.linspace(0.0, 2000.0, 20000)
        phi_clothoid = 0.5 * clothoid_a * s_clothoid**2
        x_clothoid = np.zeros(len(s_clothoid))
        y_clothoid = np.zeros(len(s_clothoid))
        ds_step = s_clothoid[1] - s_clothoid[0]
        x_clothoid[1:] = np.cumsum(np.cos(phi_clothoid[:-1])) * ds_step
        y_clothoid[1:] = np.cumsum(np.sin(phi_clothoid[:-1])) * ds_step

    # Package parameters for pose evaluation
    params = {
        "v_init": v_init,
        "v_target": v_target,
        "t_speed_start": t_speed_start,
        "t_speed_dur": t_speed_dur,
        "start_length_margin": start_length_margin,
        "maneuver_type": maneuver_type,
        "t_start": t_start,
        "t_duration": t_duration,
        "width": width,
        "dwell_time": dwell_time,
        "radius": radius,
        "direction": direction,
        "lead_in_length": lead_in_length,
        "radius2": radius2,
        "slalom_period": slalom_period,
        "slalom_amplitude": slalom_amplitude,
        "theta1": theta1,
        "theta2": theta2,
        "clothoid_a": clothoid_a,
        "vehicle_width": vehicle_width,
        "s_clothoid": s_clothoid,
        "x_clothoid": x_clothoid,
        "y_clothoid": y_clothoid,
        "superpose_lc": superpose_lc,
        "lc_start_time": lc_start_time,
        "lc_duration": lc_duration,
        "lc_width": lc_width
    }

    t_end_crg = t_end + end_length_margin / v_target
    u_length = get_station_at_time(t_end_crg, v_init, v_target, t_speed_start, t_speed_dur, start_length_margin) - \
               get_station_at_time(t_start_crg, v_init, v_target, t_speed_start, t_speed_dur, start_length_margin)



    # 5. Integrate reference line station arc-length starting from t_start_crg
    dt_int = 0.001
    t_int = np.arange(t_start_crg, t_end_crg + dt_int/2, dt_int)
    
    # Dense sample trajectory
    x_int = np.zeros(len(t_int))
    y_int = np.zeros(len(t_int))
    phi_int = np.zeros(len(t_int))
    for idx, t in enumerate(t_int):
        x_int[idx], y_int[idx], phi_int[idx] = get_trajectory_pose(t, params)

    # Arc length integration
    dx = np.diff(x_int)
    dy = np.diff(y_int)
    ds_int = np.sqrt(dx**2 + dy**2)
    u_int = np.zeros(len(t_int))
    u_int[1:] = np.cumsum(ds_int)

    u_length = u_int[-1]
    du = mesh_resolution
    dv = mesh_resolution

    u_grid = np.arange(0.0, u_length + du/2, du)
    Nu = len(u_grid)
    v_right = -v_width / 2.0
    v_left = v_width / 2.0
    Nv = len(np.arange(v_right, v_left + dv/2, dv))

    # Interpolate values on u_grid
    t_grid = np.interp(u_grid, u_int, t_int)
    phi_grid = np.zeros(Nu)
    x_ref = np.zeros(Nu)
    y_ref = np.zeros(Nu)
    for i, t in enumerate(t_grid):
        x_ref[i], y_ref[i], phi_grid[i] = get_trajectory_pose(t, params)

    # 6. Generate Slope and Banking Profiles
    slope_grid = np.zeros(Nu)
    banking_grid = np.zeros(Nu)
    
    # Pre-calculate local curvature profile (finite differences w.r.t time)
    kappa_grid = np.zeros(Nu)
    for i in range(Nu):
        t_curr = t_grid[i]
        dt_curv = 0.01
        x_m, y_m, _ = get_trajectory_pose(t_curr - dt_curv, params)
        x_c, y_c, _ = get_trajectory_pose(t_curr, params)
        x_p, y_p, _ = get_trajectory_pose(t_curr + dt_curv, params)
        
        dx_c = (x_p - x_m) / (2.0 * dt_curv)
        dy_c = (y_p - y_m) / (2.0 * dt_curv)
        ddx_c = (x_p - 2.0 * x_c + x_m) / (dt_curv**2)
        ddy_c = (y_p - 2.0 * y_c + y_m) / (dt_curv**2)
        
        denom = (dx_c**2 + dy_c**2)**1.5
        if denom < 1e-6:
            kappa_grid[i] = 0.0
        else:
            kappa_grid[i] = (dx_c * ddy_c - dy_c * ddx_c) / denom

    # Apply Curvature Filtering (Moving Average Filter) if window size > 0
    if curvature_filter_window_m > 0.0:
        window_size = int(round(curvature_filter_window_m / du))
        if window_size > 1:
            window = np.ones(window_size) / window_size
            padded_kappa = np.pad(kappa_grid, (window_size//2, window_size - 1 - window_size//2), mode='edge')
            kappa_grid = np.convolve(padded_kappa, window, mode='valid')
            
    for i, u in enumerate(u_grid):
        slope_grid[i] = evaluate_profile(u, slope_type, slope_value, slope_start, slope_duration)
        
        if banking_type == "Link to Curvature":
            scale = radius if maneuver_type in ["Circular Path", "J-Turn", "U-Turn", "Braking in a Turn", "Fishhook"] else 50.0
            banking_grid[i] = -banking_value * kappa_grid[i] * scale
        elif banking_type == "Balance Lateral Acceleration":
            v_curr = get_speed_at_time(t_grid[i], v_init, v_target, t_speed_start, t_speed_dur)
            # theta_bank = - v^2 * kappa / g
            raw_bank = - (v_curr**2 * kappa_grid[i]) / 9.80665
            # Cap the banking cross slope to the user-specified banking_value (superelevation limit)
            banking_grid[i] = np.clip(raw_bank, -abs(banking_value), abs(banking_value))
        else:
            banking_grid[i] = 0.0

    # 7. Roughness parameters (ISO 8608 Classes A to H)
    roughness_coefficients = {
        'A': 16e-6,
        'B': 64e-6,
        'C': 256e-6,
        'D': 1024e-6,
        'E': 4096e-6,
        'F': 16384e-6,
        'G': 65536e-6,
        'H': 262144e-6
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
    Nf = roughness_Nf
    Ntheta = roughness_Ntheta

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

    # Integrate z_ref reference elevation
    z_ref = np.zeros(Nu)
    for i in range(1, Nu):
        z_ref[i] = z_ref[i - 1] + du * slope_grid[i - 1]

    # 4. Generate Bezier Lane Change Path with exact road surface Z coordinate
    dt_path = 0.25
    t_end_path = t_end + end_length_margin / v_target
    times_path = np.arange(0.0, t_end_path + dt_path/2, dt_path)
    points = []

    for t in times_path:
        x, y, phi = get_trajectory_pose(t, params)
        u_val = np.interp(t, t_int, u_int)
        z_ref_val = np.interp(u_val, u_grid, z_ref)
        
        # Calculate random elevation process roughness at (x, y)
        roughness_z = np.sum(amps * np.cos(kx * x + ky * y + phis)) if len(amps) > 0 else 0.0
        z = z_ref_val + roughness_z
        points.append((x, y, z))

    path_file = os.path.join(output_dir, "default_lane_change_path.txt")
    with open(path_file, "w") as f:
        f.write(f"{len(points)} 3\n")
        for p in points:
            f.write(f"  {p[0]:12.6f}  {p[1]:12.6f}  {p[2]:12.6f}\n")
    print(f"Generated path file with {len(points)} points at {path_file}")

    # 8. Global Coordinates and Elevation Evaluation (Offloaded to C++)
    temp_input_file = os.path.join(output_dir, "temp_input.txt")
    crg_file = os.path.join(output_dir, "default_road.crg")

    with open(temp_input_file, "w") as f:
        f.write(f"{Nu} {Nv} {len(amps)} {u_length:.3f} {du:.6f} {v_right:.6f} {v_left:.6f} {dv:.6f}\n")
        for i in range(Nu):
            f.write(f"{phi_grid[i]:.10f} {x_ref[i]:.10f} {y_ref[i]:.10f} {slope_grid[i]:.10f} {banking_grid[i]:.10f}\n")
        for w in range(len(amps)):
            f.write(f"{amps[w]:.10f} {kx[w]:.10f} {ky[w]:.10f} {phis[w]:.10f}\n")

    print(f"Temporary input file written to {temp_input_file}. Invoking C++ generator...")

    exe_path = os.path.join(base_dir, "build", "generate_road.exe")
    if not os.path.exists(exe_path):
        exe_path = os.path.join(road_gen_dir, "generate_road.exe")

    if not os.path.exists(exe_path):
        raise FileNotFoundError(f"Compiler executable 'generate_road.exe' not found at {exe_path}. Run build_fmus.bat first.")

    import subprocess
    cmd = [exe_path, temp_input_file, crg_file]
    if generate_obj:
        obj_file = os.path.join(output_dir, "default_road.obj")
        cmd.append(obj_file)
        
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
        for line in process.stdout:
            print(line, end="")
        process.wait()
        if process.returncode != 0:
            raise subprocess.CalledProcessError(process.returncode, cmd)
    if generate_obj:
        print(f"Generated road OBJ mesh at {obj_file}")

    if os.path.exists(temp_input_file):
        os.remove(temp_input_file)

    print(f"Generated OpenCRG road file at {crg_file}")

    # 9. Generate Matlab script containing simulation parameters
    matlab_file = os.path.join(output_dir, "simulation_parameters.m")
    dx = points[1][0] - points[0][0]
    dy = points[1][1] - points[0][1]
    dz = points[1][2] - points[0][2]
    init_yaw = np.arctan2(dy, dx)
    init_pitch = -np.arctan2(dz, np.sqrt(dx**2 + dy**2))
    init_roll = 0.0
    max_wheel_turn_rad = max_wheel_turn_angle * np.pi / 180.0
    
    # Formulate speed profile vectors for Matlab / Simulink 1D lookup
    t_speed_end = t_speed_start + t_speed_dur
    if t_speed_dur <= 0:
        time_vec = [0.0, t_end]
        speed_vec = [v_target, v_target]
    else:
        time_vec = [0.0, t_speed_start, t_speed_end, t_end]
        speed_vec = [v_init, v_init, v_target, v_target]
        
    time_vec_str = "[" + ", ".join(f"{t:.2f}" for t in time_vec) + "]"
    speed_vec_str = "[" + ", ".join(f"{v:.6f}" for v in speed_vec) + "]"

    with open(matlab_file, "w") as f:
        f.write("% Chrono Co-Simulation Test Case Parameters\n")
        f.write("% Generated by Chrono Road Generator\n\n")
        
        f.write("% Scenario Configuration\n")
        f.write(f"target_speed_kph = {target_speed_kph:.6f};\n")
        f.write(f"target_speed_mps = {v_target:.6f};\n")
        f.write(f"simulation_end_time = {t_end:.6f};\n")
        f.write(f"road_friction_mu = {mu_value:.6f};\n")
        f.write(f"maneuver_type = '{maneuver_type}';\n\n")
        
        f.write("% Speed Profile Lookup Table Vectors (Simulink 1D Lookup feeding Driver FMU)\n")
        f.write("[script_dir, ~, ~] = fileparts(mfilename('fullpath'));\n")
        f.write("speed_profile_data = load(fullfile(script_dir, 'speed_profile.txt'));\n")
        f.write("speed_profile_time = speed_profile_data(:, 1)';\n")
        f.write("speed_profile_speed = speed_profile_data(:, 2)'; %% m/s\n\n")
        
        f.write("% Initial Pose (derived from trajectory starting point)\n")
        f.write(f"init_x = {points[0][0]:.6f};\n")
        f.write(f"init_y = {points[0][1]:.6f};\n")
        f.write(f"init_z = {points[0][2]:.6f};\n")
        f.write(f"init_yaw = {init_yaw:.6f};\n")
        f.write(f"init_roll = {init_roll:.6f};\n")
        f.write(f"init_pitch = {init_pitch:.6f};\n\n")
        
        f.write("% Path Follower Driver Parameters\n")
        f.write(f"steering_type = {steering_type}; % 0: PID, 1: Stanley\n")
        f.write(f"look_ahead_dist = {look_ahead_dist:.6f};\n")
        f.write(f"Kp_steering = {Kp_steering:.6f};\n")
        f.write(f"Ki_steering = {Ki_steering:.6f};\n")
        f.write(f"Kd_steering = {Kd_steering:.6f};\n")
        f.write(f"stanley_dead_zone = {stanley_dead_zone:.6f};\n")
        f.write(f"max_wheel_turn_angle = {max_wheel_turn_rad:.6f}; % rad ({max_wheel_turn_angle} deg)\n\n")
        
        f.write("% Cruise Controller Parameters\n")
        f.write(f"Kp_speed = {Kp_speed:.6f};\n")
        f.write(f"Ki_speed = {Ki_speed:.6f};\n")
        f.write(f"Kd_speed = {Kd_speed:.6f};\n\n")
        
        f.write("% Terrain Visualization and Packaging Parameters\n")
        f.write(f"terrain_crg_simplify = {1 if terrain_crg_simplify else 0};\n")
        f.write(f"terrain_diffuse_texture = '{terrain_diffuse_texture}';\n")
        f.write(f"terrain_normal_texture = '{terrain_normal_texture}';\n")
        f.write(f"terrain_show_visual_lines = {1 if terrain_show_visual_lines else 0};\n")
        
    print(f"Generated Matlab simulation parameters at {matlab_file}")

    # Generate simple speed profile text file for C++ co-simulation launcher
    speed_profile_file = os.path.join(output_dir, "speed_profile.txt")
    with open(speed_profile_file, "w") as f:
        for t, v in zip(time_vec, speed_vec):
            f.write(f"{t:.2f} {v:.6f}\n")
    print(f"Generated Speed Profile text file at {speed_profile_file}")



if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    settings_file = os.path.join(script_dir, "road_generator_settings.json")
    
    params = {}
    if os.path.exists(settings_file):
        try:
            with open(settings_file, "r") as f:
                params = json.load(f)
            print(f"Loaded settings from {settings_file}")
        except Exception as e:
            print(f"Error loading settings file: {e}")

    generate_road_profile(
        target_speed_kph=params.get("target_speed_kph", 60.0),
        t_start=params.get("t_start", 2.0),
        t_duration=params.get("t_duration", 5.0),
        t_end=params.get("t_end", 13.0),
        width=params.get("width", 5.0),
        start_length_margin=params.get("start_length_margin", 20.0),
        end_length_margin=params.get("end_length_margin", 50.0),
        mesh_resolution=params.get("mesh_resolution", 0.06),
        v_width=params.get("v_width", 8.0),
        iso_class=params.get("iso_class", "C"),
        generate_obj=params.get("generate_obj", False),
        base_dir=None,
        
        maneuver_type=params.get("maneuver_type", "Single Lane Change"),
        dwell_time=params.get("dwell_time", 2.0),
        radius=params.get("radius", 50.0),
        direction=params.get("direction", "Left"),
        lead_in_length=params.get("lead_in_length", 20.0),
        
        radius2=params.get("radius2", 20.0),
        slalom_period=params.get("slalom_period", 30.0),
        slalom_amplitude=params.get("slalom_amplitude", 2.0),
        theta1=params.get("theta1", 45.0),
        theta2=params.get("theta2", 180.0),
        
        clothoid_a=params.get("clothoid_a", 0.0005),
        vehicle_width=params.get("vehicle_width", 2.0),
        
        slope_type=params.get("slope_type", "None"),
        slope_value=params.get("slope_value", 0.0),
        slope_start=params.get("slope_start", 30.0),
        slope_duration=params.get("slope_duration", 20.0),
        
        banking_type=params.get("banking_type", "None"),
        banking_value=params.get("banking_value", 0.0),
        banking_start=params.get("banking_start", 30.0),
        banking_duration=params.get("banking_duration", 20.0),
        
        mu_value=params.get("mu_value", 0.85),
        
        speed_profile_time_start=params.get("speed_profile_time_start", 2.0),
        speed_profile_duration=params.get("speed_profile_duration", 5.0),
        speed_profile_initial_speed=params.get("speed_profile_initial_speed", 60.0),
        speed_profile_target_speed=params.get("speed_profile_target_speed", 60.0),
        
        steering_type=params.get("steering_type", 1),
        look_ahead_dist=params.get("look_ahead_dist", 3.615358),
        Kp_steering=params.get("Kp_steering", 2.398832),
        Ki_steering=params.get("Ki_steering", 0.0),
        Kd_steering=params.get("Kd_steering", 0.0),
        stanley_dead_zone=params.get("stanley_dead_zone", 0.010965),
        max_wheel_turn_angle=params.get("max_wheel_turn_angle", 25.0),
        
        Kp_speed=params.get("Kp_speed", 0.868900),
        Ki_speed=params.get("Ki_speed", 0.436516),
        Kd_speed=params.get("Kd_speed", 0.0),
        
        roughness_Nf=params.get("roughness_Nf", 512),
        roughness_Ntheta=params.get("roughness_Ntheta", 32)
    )
