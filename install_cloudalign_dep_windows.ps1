Write-Host "==========================================="
Write-Host " CloudAlign Dependency Installer (Windows)"
Write-Host " Using vcpkg + PowerShell"
Write-Host "==========================================="

# Set install path
$vcpkgPath = "$env:USERPROFILE\vcpkg"

# 1. Install Git if missing
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Host "[1/8] Git not found. Installing Git..."
    winget install --id Git.Git -e --source winget
} else {
    Write-Host "[1/8] Git already installed."
}

# 2. Clone vcpkg
if (!(Test-Path $vcpkgPath)) {
    Write-Host "[2/8] Cloning vcpkg..."
    git clone https://github.com/microsoft/vcpkg $vcpkgPath
} else {
    Write-Host "[2/8] vcpkg already exists, updating..."
    cd $vcpkgPath
    git pull
}

# 3. Bootstrap vcpkg
Write-Host "[3/8] Bootstrapping vcpkg..."
cd $vcpkgPath
cmd /c ".\bootstrap-vcpkg.bat"

# 4. Integrate with system
Write-Host "[4/8] Integrating vcpkg..."
.\vcpkg integrate install

# 5. Install dependencies
Write-Host "[5/8] Installing Eigen, PCL, JSON, NFD..."
.\vcpkg install eigen3:x64-windows
.\vcpkg install pcl:x64-windows
.\vcpkg install pcl[visualization]:x64-windows
.\vcpkg install nlohmann-json:x64-windows
.\vcpkg install nativefiledialog-extended:x64-windows


# 6. Install happly (header-only)
Write-Host "[6/8] Installing happly (header-only)..."
$includeDir = "$vcpkgPath\installed\x64-windows\include\happly"
New-Item -ItemType Directory -Force -Path $includeDir | Out-Null

$tempHapply = "$env:TEMP\happly"
git clone https://github.com/nmwsharp/happly.git $tempHapply

# Correct path for headers
if (Test-Path "$tempHapply\include") {
    Copy-Item "$tempHapply\include\*" $includeDir -Recurse -Force
} else {
    # Fallback: copy all files if include folder missing
    Copy-Item "$tempHapply\*" $includeDir -Recurse -Force
}

# Clean up temporary folder
Remove-Item $tempHapply -Recurse -Force

Write-Host "[7/8] Cleaning up..."
# Nothing extra to clean

Write-Host "[8/8] DONE!"
Write-Host "==========================================="
Write-Host " All CloudAlign dependencies installed!"
Write-Host ""
Write-Host " Add this to your CMake configuration:"
Write-Host "   -DCMAKE_TOOLCHAIN_FILE=$vcpkgPath\scripts\buildsystems\vcpkg.cmake"
Write-Host "==========================================="
