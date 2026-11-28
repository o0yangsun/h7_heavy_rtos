@echo off
setlocal

rem Requires gcc/g++ on PATH (MSYS2 mingw64 works).
rem Set UL_BCP_CODEC_DIR to the ul_bcp_bridge directory to run the optional
rem differential test against the PC codec.

set "TESTS=%~dp0"
set "ROOT=%TESTS%..\.."
set "OUT=%TESTS%out"
if not exist "%OUT%" mkdir "%OUT%"

if not defined GCC set "GCC=gcc"
if not defined GXX set "GXX=g++"

where %GCC% >nul 2>nul || (echo [ERR] gcc not found in PATH & exit /b 1)
where %GXX% >nul 2>nul || (echo [ERR] g++ not found in PATH & exit /b 1)

%GCC% -std=c11 -Wall -Wextra -Werror -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" -c "%ROOT%\USER\Module\Protocol\trans_frame.c" -o "%OUT%\trans_frame.o" || exit /b 1
%GCC% -std=c11 -Wall -Wextra -Werror -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" -c "%ROOT%\USER\Module\Protocol\trans_msg.c" -o "%OUT%\trans_msg.o" || exit /b 1
%GXX% -std=c++17 -Wall -Wextra -Werror -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" "%TESTS%test_protocol.cpp" "%OUT%\trans_frame.o" "%OUT%\trans_msg.o" -o "%OUT%\test_protocol.exe" -lm || exit /b 1
"%OUT%\test_protocol.exe" || exit /b 1

%GCC% -std=c11 -Wall -Wextra -Werror -I"%TESTS%fake" -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" -c "%ROOT%\USER\Module\Protocol\trans_usb.c" -o "%OUT%\trans_usb.o" || exit /b 1
%GXX% -std=c++17 -Wall -Wextra -Werror -I"%TESTS%fake" -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" "%TESTS%test_usb.cpp" "%OUT%\trans_usb.o" -o "%OUT%\test_usb.exe" || exit /b 1
"%OUT%\test_usb.exe" || exit /b 1

if not defined UL_BCP_CODEC_DIR (
    echo [SKIP] UL_BCP_CODEC_DIR not set, skip PC differential test.
    exit /b 0
)
if not exist "%UL_BCP_CODEC_DIR%\src\bcp_codec.cpp" (
    echo [SKIP] %UL_BCP_CODEC_DIR%\src\bcp_codec.cpp not found.
    exit /b 0
)

%GXX% -std=c++17 -Wall -Wextra -Werror -I"%ROOT%\USER\Module\Protocol" -I"%ROOT%\USER\Task\Robot_Config" -I"%UL_BCP_CODEC_DIR%\include" "%TESTS%test_pc_differential.cpp" "%OUT%\trans_frame.o" "%OUT%\trans_msg.o" "%UL_BCP_CODEC_DIR%\src\bcp_codec.cpp" -o "%OUT%\test_pc_differential.exe" -lm || exit /b 1
"%OUT%\test_pc_differential.exe" || exit /b 1

exit /b 0
