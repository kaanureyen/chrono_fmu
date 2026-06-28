# Lateral Controller Parameter Sweep Tool

The repository provides an automated parameter sweep utility (`parameter_sweep.py`) located at the root of the `chrono_fmus` workspace. This tool performs grid searches to evaluate and optimize the PID/Stanley path-following gains of the driver model.

---

## ⚙️ How it Works

The tool automates the tuning process by running the lane-change co-simulation demo headlessly over a grid of different configuration values for:
1. **`Kp_steering`** (Proportional Gain): Grid values: `[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]`.
2. **`look_ahead_dist`** (Look-Ahead Distance in meters): Grid values: `[2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]`.

For each grid point combination, the script:
* Launches `demo_VEH_FMI2_WheeledVehicle_lanechange.exe` in headless mode.
* Measures and logs vehicle tracking errors relative to the reference centerline trajectory.
* Evaluates simulation stability (RMSE tracking error, maximum lateral deviation limit $< 2.0$ meters, and minimum distance traveled $> 150.0$ meters).
* Identifies the optimal parameters yielding the lowest Root Mean Squared Error (RMSE).

---

## 🏃 Running the Sweep

To run the parameter sweep tool, ensure you have built the custom FMUs first, then run:

```cmd
python parameter_sweep.py
```

### Output Example:
```
Found executable at: C:\Users\novo\.gemini\antigravity\scratch\chrono_fmus\build\src\demo_VEH_FMI2_WheeledVehicle_lanechange\demo_VEH_FMI2_WheeledVehicle_lanechange.exe

Starting Parameter Sweep Grid...
    Kp | LookAhead |               Status |   RMSE (m) | MaxErr (m)
-----------------------------------------------------------------
   0.1 |       2.0 |                   OK |     0.0768 |     0.1852
   0.1 |       3.0 |                   OK |     0.1120 |     0.2541
...
   1.0 |       8.0 |                   OK |     0.0543 |     0.1240

Parameter Sweep Complete!
Best Configuration:
  - Kp_steering:      0.800000
  - look_ahead_dist:  4.000000
  - Min RMSE:         0.038294 meters
```

Temporary CSV trajectory outputs generated during evaluation are stored cleanly inside `build/temp_sweep_output.csv`.
