@echo off
setlocal

set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM3"
if not "%~1"=="" shift /1

set "BUNDLED_PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"

if exist "%BUNDLED_PY%" (
    set "PYTHON_EXE=%BUNDLED_PY%"
) else (
    set "PYTHON_EXE=python"
)

echo Running RA4M2 position tuner on %PORT%
echo Python: %PYTHON_EXE%
echo.

"%PYTHON_EXE%" "%~dp0ra4m2_position_tuner.py" --port "%PORT%" --run %*

echo.
echo Done. If the motor is unsafe, press P000 or cut power.
echo Log file: %~dp0position_tuner_last_result.txt
pause
