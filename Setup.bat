@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup.ps1" %*
set "setup_exit=%ERRORLEVEL%"
if not "%setup_exit%"=="0" (
    echo.
    echo Setup failed. See the error above and run Setup.bat again to retry.
    pause
)
exit /b %setup_exit%
