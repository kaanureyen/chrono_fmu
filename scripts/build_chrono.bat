@echo off
set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
set "SRC_DIR=c:/Users/novo/.gemini/antigravity/scratch/chrono"
set "BUILD_DIR=c:/Users/novo/.gemini/antigravity/scratch/chrono_build"
set "IRRLICHT_ROOT=c:/Users/novo/.gemini/antigravity/scratch/packages/irrlicht-1.8.5"
set "OPENCRG_DIR=c:/Users/novo/.gemini/antigravity/scratch/packages/openCRG"

call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1

echo Configuring Chrono with Ninja...
cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCH_ENABLE_MODULE_FMI=ON ^
  -DCH_ENABLE_MODULE_VEHICLE=ON ^
  -DCH_ENABLE_MODULE_IRRLICHT=ON ^
  -DIrrlicht_ROOT="%IRRLICHT_ROOT%" ^
  -DCH_ENABLE_OPENCRG=ON ^
  -DOpenCRG_INCLUDE_DIR="%OPENCRG_DIR%\include" ^
  -DOpenCRG_LIBRARY="%OPENCRG_DIR%\lib\OpenCRG.lib" ^
  -DBUILD_DEMOS=OFF ^
  -DBUILD_TESTING=OFF ^
  -DBUILD_SHARED_LIBS=OFF ^
  -DCH_USE_MSVC_STATIC_RUNTIME=ON || exit /b 1

echo Building Chrono...
cmake --build "%BUILD_DIR%" --config Release -j || exit /b 1
