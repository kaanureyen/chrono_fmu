import os
import shutil
import zipfile

def package():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scratch_dir = os.path.dirname(script_dir)
    
    zip_name = os.path.join(script_dir, "chrono_wheeled_vehicle_simulation.zip")
    print(f"Creating release package: {zip_name}...")
    
    # Paths in source
    src_gui = os.path.join(script_dir, "road_generator_gui.py")
    src_road_gen_dir = os.path.join(script_dir, "src", "road_generator")
    
    build_dir = os.path.join(script_dir, "build")
    src_gen_road_exe = os.path.join(build_dir, "generate_road.exe")
    src_veh_fmu = os.path.join(build_dir, "FMU2cs_WheeledVehicle4Torques", "FMU2cs_WheeledVehicle4Torques.fmu")
    src_drv_fmu = os.path.join(build_dir, "FMU2cs_PathFollowerDriver", "FMU2cs_PathFollowerDriver.fmu")
    
    src_demo_exe = os.path.join(build_dir, "src", "demo_VEH_FMI2_WheeledVehicle_lanechange", "demo_VEH_FMI2_WheeledVehicle_lanechange.exe")
    
    # Locate Irrlicht DLL
    irrlicht_dll = os.path.join(scratch_dir, "packages", "irrlicht-1.8.5", "bin", "Win64-VisualStudio", "Irrlicht.dll")
    
    # Validate critical files exist
    critical_files = [src_gui, src_gen_road_exe, src_veh_fmu, src_drv_fmu, src_demo_exe, irrlicht_dll]
    missing = [f for f in critical_files if not os.path.exists(f)]
    if missing:
        print("\nERROR: The following required files/build artifacts are missing:")
        for m in missing:
            print(f"  - {m}")
        print("\nPlease run 'scripts\\build_fmus.bat' first to build all binaries and FMUs.")
        return False
        
    # Write requirements.txt
    req_file = os.path.join(script_dir, "requirements.txt")
    with open(req_file, "w") as f:
        f.write("numpy\nmatplotlib\n")
        
    # Write release README
    readme_content = """# Chrono Co-Simulation & Road Generator GUI Release

This package contains the Road Generator GUI and FMI-based co-simulation software.

## Prerequisites
1. **Python 3.10+**: Ensure Python is installed and added to your system PATH.
2. **Dependencies**: Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

## Running the GUI
Double-click `road_generator_gui.py` or run it from the command line:
```bash
python road_generator_gui.py
```

## Package Contents
- `road_generator_gui.py`: Interactive GUI for configuring roads, maneuvers, and controller gains.
- `requirements.txt`: Python package requirements.
- `src/road_generator/`: Python logic for generating reference coordinates.
- `build/generate_road.exe`: C++ road generator binary.
- `build/FMU2cs_WheeledVehicle4Torques/FMU2cs_WheeledVehicle4Torques.fmu`: Vehicle FMU file.
- `build/FMU2cs_PathFollowerDriver/FMU2cs_PathFollowerDriver.fmu`: Driver FMU file.
- `build/src/demo_VEH_FMI2_WheeledVehicle_lanechange/`: Contains the simulation runner executable and its dependency Irrlicht.dll.
"""
    readme_file = os.path.join(script_dir, "RELEASE_README.md")
    with open(readme_file, "w") as f:
        f.write(readme_content)
        
    # Create ZIP archive
    with zipfile.ZipFile(zip_name, "w", zipfile.ZIP_DEFLATED) as zipf:
        # Add GUI
        zipf.write(src_gui, "road_generator_gui.py")
        zipf.write(req_file, "requirements.txt")
        zipf.write(readme_file, "README.md")
        
        # Add src/road_generator files
        for root, dirs, files in os.walk(src_road_gen_dir):
            for file in files:
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, script_dir)
                zipf.write(file_path, rel_path)
                
        # Add build binaries
        zipf.write(src_gen_road_exe, "build/generate_road.exe")
        zipf.write(src_veh_fmu, "build/FMU2cs_WheeledVehicle4Torques/FMU2cs_WheeledVehicle4Torques.fmu")
        zipf.write(src_drv_fmu, "build/FMU2cs_PathFollowerDriver/FMU2cs_PathFollowerDriver.fmu")
        
        # Add Demo Exe and Irrlicht DLL
        zipf.write(src_demo_exe, "build/src/demo_VEH_FMI2_WheeledVehicle_lanechange/demo_VEH_FMI2_WheeledVehicle_lanechange.exe")
        zipf.write(irrlicht_dll, "build/src/demo_VEH_FMI2_WheeledVehicle_lanechange/Irrlicht.dll")
        
    # Clean up temp requirements and readme files
    os.remove(req_file)
    os.remove(readme_file)
    
    print(f"\nSUCCESS: Release package created successfully at:")
    print(f"  {zip_name}")
    return True

if __name__ == "__main__":
    package()
