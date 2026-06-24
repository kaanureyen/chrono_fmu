@echo off
set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
set "SRC_DIR=c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus"
set "BUILD_DIR=c:/Users/novo/.gemini/antigravity/scratch/chrono_fmus/build"
set "CHRONO_BUILD_DIR=c:/Users/novo/.gemini/antigravity/scratch/chrono_build"

call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1

set "PATH=c:\Users\novo\.gemini\antigravity\scratch\packages\irrlicht-1.8.5\bin\Win64-VisualStudio;%PATH%"

echo Configuring custom FMUs with Ninja...
cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DChrono_DIR="%CHRONO_BUILD_DIR%\cmake" || exit /b 1

echo Building custom FMUs...
cmake --build "%BUILD_DIR%" --config Release -j || exit /b 1
