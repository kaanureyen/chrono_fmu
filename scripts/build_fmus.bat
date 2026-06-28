@echo off
if not defined VS_PATH (
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Community" (
        set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional" (
        set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Professional"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Enterprise" (
        set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Enterprise"
    ) else (
        echo ERROR: Visual Studio 2022 path not found. Please set VS_PATH manually.
        exit /b 1
    )
)
set "SCRATCH_DIR=%~dp0..\..\"
for %%I in ("%SCRATCH_DIR%") do set "SCRATCH_DIR=%%~fI"

if "%SRC_DIR%"=="" set "SRC_DIR=%SCRATCH_DIR%\chrono_fmus"
if "%BUILD_DIR%"=="" set "BUILD_DIR=%SCRATCH_DIR%\chrono_fmus\build"
if "%CHRONO_BUILD_DIR%"=="" set "CHRONO_BUILD_DIR=%SCRATCH_DIR%\chrono_build"

call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1

set "PATH=%SCRATCH_DIR%\packages\irrlicht-1.8.5\bin\Win64-VisualStudio;%PATH%"

echo Cleaning existing FMU build folders to ensure clean repackaging...
if exist "%BUILD_DIR%\FMU2cs_WheeledVehicle4Torques" (
    echo Deleting %BUILD_DIR%\FMU2cs_WheeledVehicle4Torques...
    rmdir /s /q "%BUILD_DIR%\FMU2cs_WheeledVehicle4Torques"
)
if exist "%BUILD_DIR%\FMU2cs_PathFollowerDriver" (
    echo Deleting %BUILD_DIR%\FMU2cs_PathFollowerDriver...
    rmdir /s /q "%BUILD_DIR%\FMU2cs_PathFollowerDriver"
)

echo Generating driver paths and terrain profile files...
call "%~dp0generate_road.bat" || exit /b 1

echo Configuring custom FMUs with Ninja...
cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DChrono_DIR="%CHRONO_BUILD_DIR%\cmake" || exit /b 1

echo Building custom FMUs...
cmake --build "%BUILD_DIR%" --config Release -j || exit /b 1

echo FMU compilation completed successfully!
echo.
echo Built FMU artifacts are located in the following directories:
echo   - Vehicle FMU: %BUILD_DIR%\FMU2cs_WheeledVehicle4Torques\
echo   - Driver FMU:  %BUILD_DIR%\FMU2cs_PathFollowerDriver\
