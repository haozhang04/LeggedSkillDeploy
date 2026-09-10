@echo off
chcp 65001 >nul

:: 获取ESC字符用于彩色输出
for /F %%a in ('echo prompt $E ^| cmd') do set "ESC=%%a"

:: 定义颜色变量
set "Cyan=%ESC%[36m"
set "Green=%ESC%[32m"
set "Yellow=%ESC%[33m"
set "Red=%ESC%[31m"
set "Reset=%ESC%[0m"

for /F "usebackq delims=" %%a in (`powershell -NoProfile -Command "Get-Date -Format 'yyyy-MM-dd HH:mm:ss'"`) do set "commit_message=%%a"

if "%~1"=="-m" (
    set "commit_message=%~2"
)

echo %Cyan%==========================================%Reset%
echo %Cyan%          Auto Deploy Script%Reset%
echo %Cyan%==========================================%Reset%
echo.

echo %Yellow%[1/4] Adding changes...%Reset%
git add .
if %errorlevel% neq 0 (
    exit /b %errorlevel%
)

echo.
echo %Yellow%[2/4] Committing changes...%Reset%
git commit -m "%commit_message%"
if %errorlevel% neq 0 (
    exit /b %errorlevel%
)

echo.
echo %Yellow%[3/4] Pushing to remote...%Reset%
git push
if %errorlevel% neq 0 (
    echo.
    echo %Red%Error: Git push failed.%Reset%
    exit /b %errorlevel%
)

echo.
echo %Green%==========================================%Reset%
echo %Green%          Deploy Success!%Reset%
echo %Green%==========================================%Reset%
