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

if "%SRC_DIR%"=="" set "SRC_DIR=%SCRATCH_DIR%\chrono"
if "%BUILD_DIR%"=="" set "BUILD_DIR=%SCRATCH_DIR%\chrono_build"
if "%IRRLICHT_ROOT%"=="" set "IRRLICHT_ROOT=%SCRATCH_DIR%\packages\irrlicht-1.8.5"
if "%OPENCRG_DIR%"=="" set "OPENCRG_DIR=%SCRATCH_DIR%\packages\openCRG"

if not defined DevEnvDir (
    call "%VS_PATH%\VC\Auxiliary\Build\vcvarsall.bat" amd64 || exit /b 1
)

:: -----------------------------------------------------------------------------
:: Step 1: Download & Install Irrlicht 1.8.5 if missing
:: -----------------------------------------------------------------------------
if not exist "%IRRLICHT_ROOT%" (
    echo Irrlicht 1.8.5 not found. Downloading and extracting...
    if not exist "%SCRATCH_DIR%\packages" mkdir "%SCRATCH_DIR%\packages"
    
    powershell -Command "Write-Host 'Downloading Irrlicht 1.8.5...'; [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -UserAgent 'Wget' -Uri 'https://downloads.sourceforge.net/irrlicht/irrlicht-1.8.5.zip' -OutFile '%SCRATCH_DIR%\packages\irrlicht-1.8.5.zip'" || exit /b 1
    
    powershell -Command "Write-Host 'Extracting Irrlicht 1.8.5...'; Expand-Archive -Path '%SCRATCH_DIR%\packages\irrlicht-1.8.5.zip' -DestinationPath '%SCRATCH_DIR%\packages' -Force" || exit /b 1
    
    del /f /q "%SCRATCH_DIR%\packages\irrlicht-1.8.5.zip"
    echo Irrlicht 1.8.5 installed successfully.
) else (
    echo Irrlicht 1.8.5 is already present.
)

:: -----------------------------------------------------------------------------
:: Step 2: Download & Build OpenCRG if missing
:: -----------------------------------------------------------------------------
if not exist "%OPENCRG_DIR%" (
    echo OpenCRG not found. Downloading and compiling...
    if not exist "%SCRATCH_DIR%\chrono_fmus\build" mkdir "%SCRATCH_DIR%\chrono_fmus\build"
    
    set "DOWNLOAD_DIR=%SCRATCH_DIR%\chrono_fmus\build\download_crg"
    set "BUILD_CRG_DIR=%SCRATCH_DIR%\chrono_fmus\build\build_crg"
    
    if exist "%DOWNLOAD_DIR%" rmdir /s /q "%DOWNLOAD_DIR%"
    if exist "%BUILD_CRG_DIR%" rmdir /s /q "%BUILD_CRG_DIR%"
    
    mkdir "%DOWNLOAD_DIR%"
    mkdir "%BUILD_CRG_DIR%"
    
    powershell -Command "Write-Host 'Downloading OpenCRG 1.1.2...'; Invoke-WebRequest -UserAgent 'Wget' -Uri 'https://github.com/hlrs-vis/opencrg/archive/refs/tags/v1.1.2.zip' -OutFile '%DOWNLOAD_DIR%\crg.zip'" || exit /b 1
    powershell -Command "Write-Host 'Extracting OpenCRG...'; Expand-Archive -Force '%DOWNLOAD_DIR%\crg.zip' '%DOWNLOAD_DIR%'" || exit /b 1
    
    set "CRG_SRC_DIR=%DOWNLOAD_DIR%\opencrg-1.1.2"
    
    echo Compiling OpenCRG with static runtime...
    pushd "%BUILD_CRG_DIR%"
    
    :: Release (/MT)
    cl.exe /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc /MT /O2 /Ob2 /DNDEBUG -I"%CRG_SRC_DIR%\inc" "%CRG_SRC_DIR%\src\*.c" || (popd & exit /b 1)
    lib.exe /out:OpenCRG.lib *.obj || (popd & exit /b 1)
    del *.obj
    
    :: Debug (/MTd)
    cl.exe /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc /MTd /Zi /Ob0 /Od /RTC1 -I"%CRG_SRC_DIR%\inc" "%CRG_SRC_DIR%\src\*.c" || (popd & exit /b 1)
    lib.exe /out:OpenCRG_d.lib *.obj || (popd & exit /b 1)
    del *.obj
    
    :: Release with Debug Info (/MT)
    cl.exe /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc /MT /Zi /O2 /Ob1 /DNDEBUG -I"%CRG_SRC_DIR%\inc" "%CRG_SRC_DIR%\src\*.c" || (popd & exit /b 1)
    lib.exe /out:OpenCRG_rd.lib *.obj || (popd & exit /b 1)
    del *.obj
    
    :: MinSizeRel (/MT)
    cl.exe /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc /MT /O1 /Ob1 /DNDEBUG -I"%CRG_SRC_DIR%\inc" "%CRG_SRC_DIR%\src\*.c" || (popd & exit /b 1)
    lib.exe /out:OpenCRG_s.lib *.obj || (popd & exit /b 1)
    del *.obj
    
    echo Installing OpenCRG headers and libs...
    if not exist "%OPENCRG_DIR%\include" mkdir "%OPENCRG_DIR%\include"
    if not exist "%OPENCRG_DIR%\lib" mkdir "%OPENCRG_DIR%\lib"
    
    copy /y "%CRG_SRC_DIR%\inc\*.h" "%OPENCRG_DIR%\include" || (popd & exit /b 1)
    copy /y *.lib "%OPENCRG_DIR%\lib" || (popd & exit /b 1)
    
    popd
    
    :: Clean up OpenCRG build cache
    rmdir /s /q "%DOWNLOAD_DIR%"
    rmdir /s /q "%BUILD_CRG_DIR%"
    echo OpenCRG built and installed successfully.
) else (
    echo OpenCRG is already present.
)

:: -----------------------------------------------------------------------------
:: Step 3: Configure and Build core Chrono with Ninja
:: -----------------------------------------------------------------------------
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
