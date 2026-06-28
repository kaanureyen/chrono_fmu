import os
import subprocess
import numpy as np
import csv
import sys

# Reference path calculation (purely spatial)
def get_ref_y(x):
    target_speed = 80.0 / 3.6  # 22.2222 m/s
    t_start = 2.0
    t_duration = 5.0
    width = 5.0
    
    t = x / target_speed
    if t < t_start:
        return 0.0
    elif t > t_start + t_duration:
        return width
    else:
        tau = (t - t_start) / t_duration
        S = 35 * (tau**4) - 84 * (tau**5) + 70 * (tau**6) - 20 * (tau**7)
        return width * S

def run_simulation(exe_path, Kp_steering, look_ahead_dist, output_csv):
    cmd = [
        exe_path,
        "--headless",
        "--Kp_steering", str(Kp_steering),
        "--look_ahead_dist", str(look_ahead_dist),
        "--output", output_csv
    ]
    try:
        # Run headlessly and capture output
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
        return res.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"  Timeout for Kp={Kp_steering}, look_ahead={look_ahead_dist}")
        return False
    except Exception as e:
        print(f"  Error running simulation: {e}")
        return False

def analyze_trajectory(csv_path):
    if not os.path.exists(csv_path):
        return None, "File not found"
        
    y_errors = []
    max_error = 0.0
    last_x = 0.0
    
    try:
        with open(csv_path, "r") as f:
            reader = csv.reader(f, delimiter=" ")
            for row in reader:
                if not row or len(row) < 5:
                    continue
                # Row format: time pos_x pos_y pos_z roll pitch yaw
                t = float(row[0])
                x = float(row[1])
                y = float(row[2])
                
                # Only analyze when vehicle is moving forward
                if x < 0:
                    continue
                    
                ref_y = get_ref_y(x)
                err = y - ref_y
                y_errors.append(err)
                max_error = max(max_error, abs(err))
                last_x = x
                
        if not y_errors:
            return None, "No data points analyzed"
            
        rmse = np.sqrt(np.mean(np.array(y_errors)**2))
        
        # Stability check: vehicle must not deviate by more than 2.0 meters, 
        # and must have traveled at least 150 meters (ensuring it completed the lane change)
        is_stable = (max_error < 2.0) and (last_x > 150.0)
        
        status = "OK" if is_stable else f"FAILED (max_err={max_error:.2f}m, dist={last_x:.1f}m)"
        return {
            "rmse": rmse,
            "max_error": max_error,
            "distance": last_x,
            "stable": is_stable
        }, status
        
    except Exception as e:
        return None, f"Analysis error: {e}"

def main():
    # Find the compiled demo executable relative to the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    possible_paths = [
        os.path.join(script_dir, "build", "src", "demo_VEH_FMI2_WheeledVehicle_lanechange", "demo_VEH_FMI2_WheeledVehicle_lanechange.exe"),
        os.path.join(script_dir, "build", "demo_VEH_FMI2_WheeledVehicle_lanechange", "demo_VEH_FMI2_WheeledVehicle_lanechange.exe"),
        "demo_VEH_FMI2_WheeledVehicle_lanechange.exe"
    ]
    
    exe_path = None
    for p in possible_paths:
        if os.path.exists(p):
            exe_path = p
            break
            
    if not exe_path:
        print("ERROR: demo_VEH_FMI2_WheeledVehicle_lanechange.exe not found.")
        sys.exit(1)
        
    print(f"Found executable at: {exe_path}")
    
    # Define Parameter Sweep Grid
    kp_vals = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    look_ahead_vals = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    
    results = []
    temp_csv = os.path.join(script_dir, "build", "temp_sweep_output.csv")
    
    print("\nStarting Parameter Sweep Grid...")
    print(f"{'Kp':>6} | {'LookAhead':>9} | {'Status':>20} | {'RMSE (m)':>10} | {'MaxErr (m)':>10}")
    print("-" * 65)
    
    best_rmse = float("inf")
    best_params = None
    
    for kp in kp_vals:
        for la in look_ahead_vals:
            # Clean up old file
            if os.path.exists(temp_csv):
                os.remove(temp_csv)
                
            success = run_simulation(exe_path, kp, la, temp_csv)
            if not success:
                print(f"{kp:6.2f} | {la:9.2f} | {'RUN_FAILED':>20} | {'-':>10} | {'-':>10}")
                continue
                
            metrics, status = analyze_trajectory(temp_csv)
            if metrics is None:
                print(f"{kp:6.2f} | {la:9.2f} | {status:>20} | {'-':>10} | {'-':>10}")
                continue
                
            rmse = metrics["rmse"]
            max_err = metrics["max_error"]
            
            print(f"{kp:6.2f} | {la:9.2f} | {status:>20} | {rmse:10.4f} | {max_err:10.4f}")
            
            if metrics["stable"]:
                results.append({
                    "Kp": kp,
                    "look_ahead": la,
                    "rmse": rmse,
                    "max_error": max_err,
                    "distance": metrics["distance"]
                })
                if rmse < best_rmse:
                    best_rmse = rmse
                    best_params = (kp, la)
                    
    # Clean up temp file
    if os.path.exists(temp_csv):
        os.remove(temp_csv)
        
    print("\n" + "=" * 50)
    print("SWEEP RESULTS SUMMARY")
    print("=" * 50)
    if best_params:
        print(f"Optimal Stanley Steering Parameters found:")
        print(f"  Kp_steering:     {best_params[0]:.2f}")
        print(f"  look_ahead_dist: {best_params[1]:.2f}")
        print(f"  Best RMSE:       {best_rmse:.4f} m")
    else:
        print("ERROR: No stable Stanley steering configurations found!")
        
if __name__ == "__main__":
    main()
