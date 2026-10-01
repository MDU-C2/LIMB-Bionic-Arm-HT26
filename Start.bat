@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup.ps1" -Start %*
set "start_exit=%ERRORLEVEL%"
if not "%start_exit%"=="0" (
    echo.
    echo AURORA could not start. See the error above.
    pause
)
exit /b %start_exit%
