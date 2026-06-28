@echo off
set "SCRATCH_DIR=%~dp0..\..\"
for %%I in ("%SCRATCH_DIR%") do set "SCRATCH_DIR=%%~fI"

set "PATH=%SCRATCH_DIR%\packages\irrlicht-1.8.5\bin\Win64-VisualStudio;%PATH%"
set "DEMO_DIR=%SCRATCH_DIR%\chrono_fmus\build\src\demo_VEH_FMI2_WheeledVehicle_lanechange"
set "DEMO_EXE=demo_VEH_FMI2_WheeledVehicle_lanechange.exe"

echo Running Wheeled Vehicle Lane Change Co-Simulation Demo...
echo Directory:  %DEMO_DIR%
echo Arguments:  %*
echo.

pushd "%DEMO_DIR%"
"%DEMO_EXE%" %*
popd

echo.
pause
