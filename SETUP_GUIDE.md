# C++ Development Environment Setup for Raspberry Pi QR Detection Project

## What You Have Now
✅ **MSVC Compiler** (Visual Studio Build Tools 2022) — Installed  
✅ **VS Code Configuration** — Created at `.vscode/c_cpp_properties.json`  
⏳ **OpenCV** — Needs manual download

---

## Step 1: Download OpenCV Pre-built Binaries

### Option A: Direct Download (Fastest)

1. **Go to:** https://github.com/opencv/opencv/releases/tag/4.8.0
2. **Download:** `opencv-4.8.0-windows.zip` (approx. 240 MB)
3. **Extract to:** `C:\` 
   - After extraction, you should have: `C:\opencv-4.8.0`
4. **Rename folder:** Rename `C:\opencv-4.8.0` → `C:\opencv`
   - Right-click on folder → Rename
   - New name: `opencv`

### Option B: Using PowerShell (If comfortable with terminal)

```powershell
# Run PowerShell as Administrator, then:
$ProgressPreference = 'SilentlyContinue'
$url = "https://github.com/opencv/opencv/releases/download/4.8.0/opencv-4.8.0-windows.zip"
$output = "C:\opencv-4.8.0.zip"

Write-Host "Downloading OpenCV (this may take 2-3 minutes)..."
Invoke-WebRequest -Uri $url -OutFile $output -ErrorAction Stop
Write-Host "Download complete! Extracting..."

Expand-Archive -Path $output -DestinationPath "C:\" -Force
Rename-Item -Path "C:\opencv-4.8.0" -NewName "opencv" -Force
Remove-Item $output

Write-Host "✓ OpenCV is ready at C:\opencv" -ForegroundColor Green
```

---

## Step 2: Verify Installation

Open PowerShell and run:

```powershell
Test-Path "C:\opencv\include\opencv2"
```

**Expected output:** `True`

If you see `False`, check that the folder structure is correct:
- `C:\opencv\include\opencv2\` should exist
- `C:\opencv\build\` should exist

---

## Step 3: Update VS Code Configuration (Optional, if compiler path differs)

If VS Code still shows errors after OpenCV is installed:

1. **In VS Code:** Press `Ctrl+Shift+P`
2. **Type:** `C/C++: Edit Configurations (UI)`
3. **Under "Include path":** Add `C:/opencv/include`
4. **Save**

VS Code will auto-detect your MSVC compiler; if it doesn't, manually set the path:

1. **Press `Ctrl+Shift+P` → "C/C++: Select IntelliSense Configuration"**
2. **Choose:** "MSVC"
3. If prompted for compiler path, use:
   ```
   C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\[VERSION]\bin\Hostx64\x64\cl.exe
   ```
   (Replace `[VERSION]` with your installed version, e.g., `14.38.33130`)

---

## Step 4: Compile Your Code

### Option A: Using VS Code's built-in compiler (Easiest)

1. **Open V1.cpp** in VS Code
2. **Press:** `Ctrl+Shift+B` (Run Build Task)
3. **If no task exists:** Press `Ctrl+Shift+P → Tasks: Run Build Task → Create tasks.json from template → C++`

### Option B: Manual Compilation (Command Line)

```powershell
cd C:\Users\arjun\OneDrive\Documents\Desktop\ADDC

# Set up MSVC environment
& "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

# Compile
cl.exe V1.cpp /I"C:\opencv\include" /link "C:\opencv\build\x64\vc15\lib\opencv_core480.lib" "C:\opencv\build\x64\vc15\lib\opencv_imgproc480.lib"
```

---

## Step 5: Fix Remaining Issues

### If you still see "cannot open source file" errors:

1. **Close VS Code completely**
2. **Reopen** your workspace
3. **Wait** 5-10 seconds for IntelliSense to re-index
4. **Check** that OpenCV is at `C:\opencv\include\opencv2`

### If compilation fails with "library not found":

The pre-built binaries may not have `lib` files. In that case, you'll need to either:
- **Build OpenCV from source** (1-2 hours on your machine) — See section below
- **Use a different pre-built package** with libs included (e.g., from opencv-python-headless)

---

## Bonus: Build OpenCV from Source (For Pi-Optimized Version Later)

This is **optional** for now, but if you want to compile with NEON/TBB optimizations later (for actual Pi 4 deployment), run:

```powershell
cd C:\
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build

# Configure with optimizations
cmake -G "Visual Studio 17 2022" -A x64 `
  -D CMAKE_BUILD_TYPE=Release `
  -D BUILD_TESTS=OFF `
  -D BUILD_PERF_TESTS=OFF `
  -D WITH_OPENMP=ON `
  ..

# Build (this will take 30-60 minutes)
cmake --build . --config Release -j4

# Install to C:\opencv
cmake --install . --prefix "C:\opencv"
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "cannot open source file 'opencv2/opencv.hpp'" | Verify `C:\opencv\include` exists; update path in `c_cpp_properties.json` |
| Compiler not found | Run: `"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"` before compiling |
| Linker errors (missing .lib files) | Pre-built doesn't have libs; build from source or use conda/vcpkg |
| VS Code doesn't recognize includes after install | Restart VS Code completely |
| IntelliSense errors persist | Press `Ctrl+Shift+P → C/C++: Rescan Solutions` |

---

## Next Steps Once Errors Are Fixed

1. **Install ZBar** for faster QR detection:
   ```
   git clone https://github.com/mchehab/zbar.git
   cd zbar
   # Follow README for Windows build
   ```

2. **Start implementing Pi optimizations** from the analysis document

3. **Profile your code** to see where time is spent:
   ```cpp
   #include <chrono>
   auto start = std::chrono::high_resolution_clock::now();
   // ... code to time ...
   auto end = std::chrono::high_resolution_clock::now();
   std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";
   ```

---

**Questions?** Let me know if any step fails and I'll help troubleshoot!
