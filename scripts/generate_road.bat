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

if not defined DevEnvDir (
    call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1
)

set "ROAD_GEN_DIR=%SCRATCH_DIR%\chrono_fmus\src\road_generator"
set "BUILD_DIR=%SCRATCH_DIR%\chrono_fmus\build"

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo Compiling C++ Road Profile Generator...
cl.exe /O2 /openmp /arch:AVX2 /EHsc /Fo"%BUILD_DIR%/" /Fe"%BUILD_DIR%/generate_road.exe" "%ROAD_GEN_DIR%/generate_road.cpp" || exit /b 1

echo Running Python road_generator script to generate path and terrain...
python --version >nul 2>&1 || (
    echo ERROR: Python is required to run road_generator.py but was not found in PATH.
    exit /b 1
)

python "%ROAD_GEN_DIR%\road_generator.py" || exit /b 1

echo Path and terrain generation completed successfully!
