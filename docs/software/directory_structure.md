# Workspace Directory Structure Guide

This document maps out the clean structure of the Chrono simulation workspace. All primary workspace directories live under the scratch folder root, while temporary execution and build target contents are strictly confined within their respective build subdirectories.

---

## 📂 Scratch Root Layout

The scratch directory (`scratch/`) contains only the following four recommended top-level subdirectories:

```
scratch/
├── chrono/           # Core Project Chrono library source tree (Git repository)
├── chrono_build/     # Chrono static build folder (MSVC Release static build output)
├── packages/         # Chrono third-party dependency SDK packages
└── chrono_fmus/      # Custom FMUs project (Git repository)
```

### 1. `packages/` Subdirectory
Holds pre-compiled SDK dependencies linked statically by Project Chrono during CMake configuration:
- `irrlicht-1.8.5/` - Irrlicht Engine SDK (DLLs in `bin/Win64-VisualStudio/` are added to the system `PATH` at runtime).
- `openCRG/` - OpenCRG road profiling library headers and static `.lib` archives.

### 2. `chrono_fmus/` Custom FMUs Subdirectory
The repository containing our custom driver and vehicle FMU targets. 

---

## 📁 Custom FMUs Repository Internal Layout

Inside the `chrono_fmus/` folder, files are organized by purpose into source code, documentation, and automated script helpers:

```
chrono_fmus/
├── CMakeLists.txt        # Root CMake configuration using relative package lookups
├── docs/                 # Documentation directory subdivided by topic
│   ├── software/         # Software guidelines (installation, directory_structure, parameter_sweep, simulink_integration)
│   │   └── fmu_parameters.m
│   └── theoretical/      # Mathematical and vehicle parameter guidelines (kpi_analysis, subsystem_parameters)
├── src/                  # Source code directory grouping all C++ sources
│   ├── fmu2_wheeled_vehicle_4torques/ # Vehicle FMU source and local resources
│   ├── fmu2_path_follower_driver/     # Driver FMU source and local resources
│   ├── demo_VEH_FMI2_WheeledVehicle_4torques/ # Cosim orchestrator demo 1
│   ├── demo_VEH_FMI2_WheeledVehicle_lanechange/ # Cosim orchestrator demo 2
│   └── road_generator/                # Road profiling C++ tool and python generator
├── scripts/              # Command batch helpers (independent of scratch absolute paths)
│   ├── build_chrono.bat  # Configures/builds core Chrono from scratch\chrono
│   ├── build_fmus.bat    # Cleans and compiles custom FMUs into build/
│   └── run_demo.bat      # Launches 4-torques cosim demo using local build/ FMUs
├── parameter_sweep.py    # Closed-loop driver parameter sweep utility
├── road_generator_gui.py # Interactive Tkinter road generation wizard GUI
└── build/                # Exclusively contains all compiled outputs and temporary content
    ├── FMU2cs_WheeledVehicle4Torques/ # Staged vehicle FMU binaries, resources, and .fmu
    ├── FMU2cs_PathFollowerDriver/     # Staged driver FMU binaries, resources, and .fmu
    ├── src/              # Target executables output (compiled from src/)
    │   ├── demo_VEH_FMI2_WheeledVehicle_4torques/
    │   └── demo_VEH_FMI2_WheeledVehicle_lanechange/
    ├── tmp_unpack_vehicle_4torques/   # Unpacked vehicle FMU temp directory
    ├── tmp_unpack_vehicle_lc/         # Unpacked vehicle FMU temp directory (lane change)
    ├── tmp_unpack_driver_lc/          # Unpacked driver FMU temp directory (lane change)
    ├── DEMO_OUTPUT/      # Output directories containing simulation result CSV logs
    └── lane_change_trajectory.csv     # Simulation output files
```

---

## ⚙️ Relative Paths and Execution Integrity

1. **Pruning Scratch Root**: All batch build and run scripts have been removed from the scratch root and consolidated under `chrono_fmus/scripts/`.
2. **Confined Execution**: Executables resolve the `build/` directory relative to their directory using `argv[0]`. Consequently:
   - All temporary zip unpacking folders (`tmp_unpack_*`) are created inside the build directory.
   - All simulation output CSV trajectories and `DEMO_OUTPUT/` plots are written inside `build/`.
3. **Pristine Source Tree**: The repository directories (`src/`, `docs/`, `scripts/`) remain completely clean of temporary generated files and compile outputs.
