# Custom FMU Installation Guide

This document describes the step-by-step procedure to set up, download, compile, and run the custom Functional Mock-up Units (FMUs) and co-simulation orchestrator from scratch.

---

## 🛠️ 1. Prerequisites Installation

You need the following compiler toolchains, build systems, and libraries. The commands below use the Windows Package Manager (`winget`) and Python's `pip` manager to install the exact tool versions verified on this machine.

### A. Core Build Tools (via `winget`)

Run the following commands in an administrator Command Prompt or PowerShell:

```powershell
# 1. Install Git
winget install --id Git.Git --silent

# 2. Install Visual Studio 2022 Community (includes cl.exe compiler)
# Note: Ensure the C++ Desktop workload is included
winget install --id Microsoft.VisualStudio.2022.Community --silent --override "--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"

# 3. Install CMake (Version 3.31.6 verified)
winget install --id Kitware.CMake --exact --version 3.31.6 --silent

# 4. Install Ninja (Version 1.13.2 verified)
winget install --id Ninja-build.Ninja --exact --version 1.13.2 --silent

# 5. Install Python 3 (Python 3.14.3 verified)
winget install --id Python.Python.3.12 --silent
```

*Note: Restart your terminal after installing the tools to update your environment `PATH` variables.*

### B. Python Dependencies (via `pip`)
The road profile generator script uses `numpy` to calculate wave components and trajectory headings:
```cmd
pip install numpy
```

---

## 📂 2. Folder Structure Setup

To guarantee compile-time path resolution, the core Chrono library repository and the custom FMUs repository must be cloned as **sibling directories** under a single workspace root folder. 

Follow this sequence of shell commands to clone and set up the directory structure:

```cmd
# 1. Create a parent workspace folder and enter it
mkdir chrono_workspace
cd chrono_workspace

# 2. Clone Core Project Chrono (Locked to tag 10.0.0 for static FMI compatibility)
git clone --branch 10.0.0 https://github.com/projectchrono/chrono.git chrono

# 3. Clone the Custom FMUs Repository
git clone <your-fmus-repo-url> chrono_fmus
```

This guarantees the following workspace folder structure:
```
chrono_workspace/       <-- Workspace root
├── chrono/             <-- Core Chrono repository
└── chrono_fmus/        <-- Custom FMUs repository (this repository)
```

---

## ⚙️ 3. Compilation & Build Steps

Run the compilation helper scripts in the exact order below from the `chrono_fmus/scripts/` directory:

```cmd
cd chrono_fmus\scripts
```

### Step A: Bootstrap & Build Core Chrono
```cmd
build_chrono.bat
```
* **What it does**: 
  1. Checks for Irrlicht 1.8.5. If missing, it downloads and extracts the SDK automatically to `packages/`.
  2. Checks for OpenCRG. If missing, it downloads OpenCRG v1.1.2, compiles it statically for all configurations (Release/Debug) using static MSVC linking flags (`/MT`, `/MTd`), installs include files and compiled `.lib` archives to `packages/openCRG/`, and cleans the temp build cache.
  3. Configures Core Chrono via CMake using the **Ninja** build system and compiles static library objects into `chrono_build/`.

### Step B (Optional): Compile & Generate Roads
```cmd
generate_road.bat
```
* **What it does**: 
  1. Compiles the C++ road profile generator (`src/road_generator/generate_road.cpp`) using AVX2 instruction sets and OpenMP multi-threading.
  2. Runs `road_generator.py` to calculate Bezier path coordinate files and ISO Class C rough road OpenCRG surfaces, copying them into the FMUs' resource trees.
  * *Note: This step is automatically called inside `build_fmus.bat` so you do not need to run it separately unless you are tweaking path parameters.*

### Step C: Compile & Package Custom FMUs
```cmd
build_fmus.bat
```
* **What it does**: 
  1. Cleans target staging folders (`build/FMU2cs_*`) to prevent stale files.
  2. Calls `generate_road.bat` to refresh paths and terrain files at build time.
  3. Configures CMake, compiles custom FMU DLLs, and generates self-contained `.fmu` packages under `build/`.

---

## 🏃 4. Run Verification

To run the co-simulation demo and confirm everything was built correctly:

```cmd
run_demo.bat --headless
```

This executes the simulation orchestrator. It extracts the compiled vehicle `.fmu` internally, unpacks all configurations, terrain geometries, and visual models, and completes the simulation. All temporary unpacked files and trajectory CSV files are written entirely inside the `build/` directory.

---

## 🎨 5. Interactive Road Customization (GUI Wizard)

An interactive graphical interface is provided to customize road profiles (speed, roughness, dimensions, and OBJ mesh options) without editing scripts. To run the GUI:

```cmd
python road_generator_gui.py
```

This opens a Tkinter GUI wizard. Any profiles generated here are placed directly in `build/generated/` and staged during compilation.

---

## 📈 6. Parameter Optimization (Sweep Tool)

To execute the parameter grid search to optimize the Stanley/PID lateral tracking controller:

```cmd
python parameter_sweep.py
```

