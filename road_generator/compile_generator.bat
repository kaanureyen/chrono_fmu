@echo off
set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1
cd road_generator
cl.exe /O2 /openmp /arch:AVX2 /EHsc generate_road.cpp /Fe:generate_road.exe || exit /b 1
cd ..
