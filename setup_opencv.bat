@echo off
REM OpenCV 4.8.0 Downloader for Windows
REM Run this script to automatically download and install OpenCV

setlocal enabledelayedexpansion

echo.
echo ========================================
echo   OpenCV 4.8.0 Setup for Windows
echo ========================================
echo.

REM Check if running as Administrator
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [WARNING] This script is running WITHOUT administrator privileges.
    echo Recommend running as Administrator for best results.
    echo.
)

set "OPENCV_URL=https://github.com/opencv/opencv/releases/download/4.8.0/opencv-4.8.0-windows.zip"
set "DOWNLOAD_PATH=%UserProfile%\Downloads\opencv-4.8.0.zip"
set "EXTRACT_PATH=C:\"
set "FINAL_PATH=C:\opencv"

echo [1/4] Checking if OpenCV already exists...
if exist "%FINAL_PATH%" (
    echo [OK] OpenCV found at %FINAL_PATH%
    echo Installation skipped.
    echo.
    pause
    exit /b 0
)

echo [2/4] Downloading OpenCV (this will take 2-3 minutes)...
echo URL: %OPENCV_URL%
powershell -Command "& { $ProgressPreference = 'SilentlyContinue'; Invoke-WebRequest -Uri '%OPENCV_URL%' -OutFile '%DOWNLOAD_PATH%' }" >nul 2>&1

if exist "%DOWNLOAD_PATH%" (
    echo [OK] Download complete at %DOWNLOAD_PATH%
) else (
    echo [ERROR] Download failed. Please manually download from:
    echo %OPENCV_URL%
    echo.
    pause
    exit /b 1
)

echo [3/4] Extracting OpenCV...
powershell -Command "& { Expand-Archive -Path '%DOWNLOAD_PATH%' -DestinationPath '%EXTRACT_PATH%' -Force }" >nul 2>&1

if exist "%FINAL_PATH%" (
    echo [OK] Extraction complete at %FINAL_PATH%
) else if exist "%EXTRACT_PATH%\opencv-4.8.0" (
    echo [OK] Extraction complete
    echo [3.5/4] Renaming folder...
    ren "%EXTRACT_PATH%\opencv-4.8.0" "opencv"
    echo [OK] Renamed to %FINAL_PATH%
) else (
    echo [ERROR] Extraction failed.
    echo Please manually extract %DOWNLOAD_PATH% to C:\
    echo and rename the folder to "opencv"
    pause
    exit /b 1
)

echo [4/4] Verifying installation...
if exist "%FINAL_PATH%\include\opencv2" (
    echo [OK] OpenCV is ready at %FINAL_PATH%
    echo.
    echo ========================================
    echo   Installation successful!
    echo ========================================
    echo.
    echo Next steps:
    echo 1. Close VS Code completely
    echo 2. Reopen your workspace
    echo 3. IntelliSense errors should now disappear
    echo.
    del "%DOWNLOAD_PATH%"
    pause
) else (
    echo [ERROR] Installation verification failed.
    echo Folder structure incorrect at %FINAL_PATH%
    echo Please check and try again.
    pause
    exit /b 1
)

endlocal
