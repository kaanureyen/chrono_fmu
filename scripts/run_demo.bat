@echo off
set "SCRATCH_DIR=%~dp0..\..\"
for %%I in ("%SCRATCH_DIR%") do set "SCRATCH_DIR=%%~fI"

set "PATH=%SCRATCH_DIR%\packages\irrlicht-1.8.5\bin\Win64-VisualStudio;%PATH%"
set "DEMO_DIR=%SCRATCH_DIR%\chrono_fmus\build\src\demo_VEH_FMI2_WheeledVehicle_4torques"
set "DEMO_EXE=demo_VEH_FMI2_WheeledVehicle_4torques.exe"
set "FMU_FILE=%SCRATCH_DIR%\chrono_fmus\build\FMU2cs_WheeledVehicle4Torques\FMU2cs_WheeledVehicle4Torques.fmu"

echo Running Wheeled Vehicle 4-Torque FMU Demo...
echo FMU:        %FMU_FILE%
echo Arguments:  %*
echo.

pushd "%DEMO_DIR%"
"%DEMO_EXE%" "%FMU_FILE%" %*
popd

echo.
pause
