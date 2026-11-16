@echo off
setlocal

set "ROOT=%~dp0"
set "CLION=C:\Clion\CLion 2025.2.6"
set "CUBECLT=C:\ST\STM32CubeCLT_1.21.0"
set "GCC_BIN=%CUBECLT%\GNU-tools-for-STM32\bin"
set "CMAKE=%CLION%\bin\cmake\win\x64\bin\cmake.exe"
set "MAKE=%CLION%\bin\mingw\bin\mingw32-make.exe"
set "BUILD_DIR=%ROOT%build"

if /I "%~1"=="clean" (
    if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
    exit /b 0
)

set "PATH=%GCC_BIN%;%CLION%\bin\cmake\win\x64\bin;%CLION%\bin\mingw\bin;%PATH%"

if /I "%~1"=="configure" goto configure
if /I "%~1"=="rebuild" if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"

:configure
"%CMAKE%" -S "%ROOT%." -B "%BUILD_DIR%" -G "MinGW Makefiles" ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ^
    -DCMAKE_C_COMPILER=C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe ^
    -DCMAKE_CXX_COMPILER=C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-g++.exe ^
    "-DCMAKE_MAKE_PROGRAM=C:/Clion/CLion 2025.2.6/bin/mingw/bin/mingw32-make.exe"
if errorlevel 1 exit /b 1

if /I "%~1"=="configure" exit /b 0

"%CMAKE%" --build "%BUILD_DIR%" --parallel
exit /b %errorlevel%
