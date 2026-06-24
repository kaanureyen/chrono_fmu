@echo off
set "PATH=c:\Users\novo\.gemini\antigravity\scratch\packages\irrlicht-1.8.5\bin\Win64-VisualStudio;%PATH%"
set "DEMO_DIR=c:\Users\novo\.gemini\antigravity\scratch\chrono_fmus\build\demo_VEH_FMI2_WheeledVehicle_4torques"
set "DEMO_EXE=demo_VEH_FMI2_WheeledVehicle_4torques.exe"
set "FMU_FILE=c:\Users\novo\.gemini\antigravity\scratch\chrono_fmus\build\FMU2cs_WheeledVehicle4Torques\FMU2cs_WheeledVehicle4Torques.fmu"

echo Running Wheeled Vehicle 4-Torque FMU Demo...
echo FMU:        %FMU_FILE%
echo Arguments:  %*
echo.

pushd "%DEMO_DIR%"
"%DEMO_EXE%" "%FMU_FILE%" %*
popd

echo.
pause
