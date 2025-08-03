# PCL Python Bindings - Build and Repair Wheel Script
# This script automates the process of building wheels and repairing them with delvewheel
# for local development on Windows.

param(
    [switch]$Build = $false,
    [switch]$Repair = $false,
    [switch]$Install = $false,
    [switch]$All = $false,
    [switch]$Clean = $false,
    [switch]$Verbose = $false,
    [string]$BuildDir = "dist",
    [string]$RepairedDir = "dist_repaired"
)

# Show help if no parameters
if (-not ($Build -or $Repair -or $Install -or $All -or $Clean)) {
    Write-Host "PCL Python Bindings - Build and Repair Wheel Script" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "USAGE:" -ForegroundColor Yellow
    Write-Host "    .\scripts\build-wheel.ps1 [OPTIONS]" -ForegroundColor White
    Write-Host ""
    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host "    -Build          Build wheel only" -ForegroundColor White
    Write-Host "    -Repair         Repair existing wheel with delvewheel only" -ForegroundColor White
    Write-Host "    -Install        Install repaired wheel only" -ForegroundColor White
    Write-Host "    -All            Build, repair, and install (recommended)" -ForegroundColor White
    Write-Host "    -Clean          Clean build directories" -ForegroundColor White
    Write-Host "    -Verbose        Show detailed output" -ForegroundColor White
    Write-Host "    -BuildDir       Directory for built wheels (default: dist)" -ForegroundColor White
    Write-Host "    -RepairedDir    Directory for repaired wheels (default: dist_repaired)" -ForegroundColor White
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor Yellow
    Write-Host "    # Full workflow (recommended)" -ForegroundColor Green
    Write-Host "    .\scripts\build-wheel.ps1 -All" -ForegroundColor White
    Write-Host ""
    Write-Host "    # Build and repair only (for CI/testing)" -ForegroundColor Green
    Write-Host "    .\scripts\build-wheel.ps1 -Build -Repair" -ForegroundColor White
    Write-Host ""
    Write-Host "    # Clean all build artifacts" -ForegroundColor Green
    Write-Host "    .\scripts\build-wheel.ps1 -Clean" -ForegroundColor White
    Write-Host ""
    Write-Host "    # Verbose output for debugging" -ForegroundColor Green
    Write-Host "    .\scripts\build-wheel.ps1 -All -Verbose" -ForegroundColor White
    Write-Host ""
    Write-Host "NOTES:" -ForegroundColor Yellow
    Write-Host "    - This script loads environment variables from .env automatically" -ForegroundColor White
    Write-Host "    - Delvewheel bundles all required DLLs into the wheel" -ForegroundColor White
    Write-Host "    - The repaired wheel is self-contained and works on any Windows machine" -ForegroundColor White
    Write-Host "    - For production builds, use cibuildwheel instead" -ForegroundColor White
    exit 0
}

# Load environment variables from .env.local or .env file
function Load-Environment {
    # Prioritize .env.local over .env if it exists
    $envLocalFile = ".env.local"
    $envFile = ".env"
    
    $fileToLoad = $null
    if (Test-Path $envLocalFile) {
        $fileToLoad = $envLocalFile
        Write-Host "Loading environment from .env.local file..." -ForegroundColor Yellow
    } elseif (Test-Path $envFile) {
        $fileToLoad = $envFile
        Write-Host "Loading environment from .env file..." -ForegroundColor Yellow
    } else {
        Write-Warning "Neither .env.local nor .env file found"
        return $false
    }
    
    Get-Content $fileToLoad | ForEach-Object {
        if ($_ -match '^([^#][^=]+)=(.*)$') {
            $name = $matches[1].Trim()
            $value = $matches[2].Trim()
            [Environment]::SetEnvironmentVariable($name, $value, "Process")
            if ($Verbose) {
                Write-Host "  Set $name = $value" -ForegroundColor Gray
            }
        }
    }
    
    # Verify required environment variables
    $required_vars = @("PCL_BIN_DIR", "VCPKG_BIN_DIR")
    $missing_vars = @()
    
    foreach ($var in $required_vars) {
        $value = [Environment]::GetEnvironmentVariable($var)
        if (-not $value) {
            $missing_vars += $var
        }
    }
    
    if ($missing_vars.Count -gt 0) {
        Write-Host "ERROR: Missing required environment variables: $($missing_vars -join ', ')" -ForegroundColor Red
        Write-Host "Please check your .env/.env.local file configuration." -ForegroundColor Yellow
        return $false
    }
    
    Write-Host "SUCCESS: Environment loaded successfully!" -ForegroundColor Green
    return $true
}

# Clean build directories
function Clean-BuildDirs {
    Write-Host "Cleaning build directories..." -ForegroundColor Yellow
    
    $dirs_to_clean = @("build", $BuildDir, $RepairedDir, "*.egg-info")
    
    foreach ($dir in $dirs_to_clean) {
        if (Test-Path $dir) {
            Remove-Item $dir -Recurse -Force
            Write-Host "  Removed: $dir" -ForegroundColor Gray
        }
    }
    
    Write-Host "SUCCESS: Build directories cleaned!" -ForegroundColor Green
}

# Build wheel
function Build-Wheel {
    Write-Host "Building wheel..." -ForegroundColor Yellow
    
    # Ensure build directory exists
    if (-not (Test-Path $BuildDir)) {
        New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
    }
    
    # Build wheel
    $build_cmd = "pip wheel . --no-build-isolation -w $BuildDir"
    if ($Verbose) {
        $build_cmd += " --verbose"
    }
    
    Write-Host "Running: $build_cmd" -ForegroundColor Cyan
    
    $process = Start-Process -FilePath "powershell" -ArgumentList "-Command", $build_cmd -NoNewWindow -Wait -PassThru
    
    if ($process.ExitCode -ne 0) {
        Write-Host "ERROR: Wheel build failed!" -ForegroundColor Red
        exit 1
    }
    
    # Find the built wheel
    $wheel_files = Get-ChildItem "$BuildDir\pcl-*.whl"
    if ($wheel_files.Count -eq 0) {
        Write-Host "ERROR: No wheel files found in $BuildDir" -ForegroundColor Red
        exit 1
    }
    
    $wheel_file = $wheel_files[0].FullName
    Write-Host "SUCCESS: Wheel built successfully: $(Split-Path $wheel_file -Leaf)" -ForegroundColor Green
    return $wheel_file
}

# Repair wheel with delvewheel
function Repair-Wheel {
    param([string]$WheelPath)
    
    Write-Host "Repairing wheel with delvewheel..." -ForegroundColor Yellow
    
    # Ensure delvewheel is installed
    try {
        $null = Get-Command delvewheel -ErrorAction Stop
    } catch {
        Write-Host "Installing delvewheel..." -ForegroundColor Cyan
        pip install delvewheel
        if ($LASTEXITCODE -ne 0) {
            Write-Host "ERROR: Failed to install delvewheel!" -ForegroundColor Red
            exit 1
        }
    }
    
    # Ensure repaired directory exists
    if (-not (Test-Path $RepairedDir)) {
        New-Item -ItemType Directory -Path $RepairedDir -Force | Out-Null
    }
    
    # Get environment variables
    $pcl_bin_dir = [Environment]::GetEnvironmentVariable("PCL_BIN_DIR")
    $vcpkg_bin_dir = [Environment]::GetEnvironmentVariable("VCPKG_BIN_DIR")
    
    # Repair wheel
    $repair_cmd = "delvewheel repair `"$WheelPath`" --add-path `"$pcl_bin_dir`" --add-path `"$vcpkg_bin_dir`" -w `"$RepairedDir`""
    if ($Verbose) {
        $repair_cmd += " --verbose"
    }
    
    Write-Host "Running: $repair_cmd" -ForegroundColor Cyan
    
    $process = Start-Process -FilePath "powershell" -ArgumentList "-Command", $repair_cmd -NoNewWindow -Wait -PassThru
    
    if ($process.ExitCode -ne 0) {
        Write-Host "ERROR: Wheel repair failed!" -ForegroundColor Red
        exit 1
    }
    
    # Find the repaired wheel
    $repaired_files = Get-ChildItem "$RepairedDir\pcl-*.whl"
    if ($repaired_files.Count -eq 0) {
        Write-Host "ERROR: No repaired wheel files found in $RepairedDir" -ForegroundColor Red
        exit 1
    }
    
    $repaired_wheel = $repaired_files[0].FullName
    Write-Host "SUCCESS: Wheel repaired successfully: $(Split-Path $repaired_wheel -Leaf)" -ForegroundColor Green
    return $repaired_wheel
}

# Install wheel
function Install-Wheel {
    param([string]$WheelPath)
    
    Write-Host "Installing repaired wheel..." -ForegroundColor Yellow
    
    $install_cmd = "pip install `"$WheelPath`" --force-reinstall"
    
    Write-Host "Running: $install_cmd" -ForegroundColor Cyan
    
    $process = Start-Process -FilePath "powershell" -ArgumentList "-Command", $install_cmd -NoNewWindow -Wait -PassThru
    
    if ($process.ExitCode -ne 0) {
        Write-Host "ERROR: Wheel installation failed!" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "SUCCESS: Wheel installed successfully!" -ForegroundColor Green
}

# Test installation
function Test-Installation {
    Write-Host "Testing installation..." -ForegroundColor Yellow
    
    $test_script = @"
import pcl
print('SUCCESS: PCL import successful')
import pcl.common
import pcl.features
import pcl.filters
print('SUCCESS: All modules imported successfully')
"@
    
    $test_script | python
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Installation test failed!" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "SUCCESS: Installation test passed!" -ForegroundColor Green
}

# Main execution
try {
    Write-Host "PCL Python Bindings - Build and Repair Wheel Script" -ForegroundColor Magenta
    Write-Host "============================================================" -ForegroundColor Magenta
    
    # Load environment
    if (-not (Load-Environment)) {
        exit 1
    }
    
    # Handle clean operation
    if ($Clean) {
        Clean-BuildDirs
        if (-not ($Build -or $Repair -or $Install -or $All)) {
            exit 0
        }
    }
    
    $wheel_path = $null
    $repaired_wheel_path = $null
    
    # Build phase
    if ($Build -or $All) {
        $wheel_path = Build-Wheel
    } else {
        # Find existing wheel for repair-only operation
        $existing_wheels = Get-ChildItem "$BuildDir\pcl-*.whl" -ErrorAction SilentlyContinue
        if ($existing_wheels.Count -gt 0) {
            $wheel_path = $existing_wheels[0].FullName
            Write-Host "Using existing wheel: $(Split-Path $wheel_path -Leaf)" -ForegroundColor Cyan
        } else {
            Write-Host "ERROR: No existing wheel found in $BuildDir. Use -Build to create one." -ForegroundColor Red
            exit 1
        }
    }
    
    # Repair phase
    if ($Repair -or $All) {
        $repaired_wheel_path = Repair-Wheel -WheelPath $wheel_path
    } else {
        # Find existing repaired wheel for install-only operation
        $existing_repaired = Get-ChildItem "$RepairedDir\pcl-*.whl" -ErrorAction SilentlyContinue
        if ($existing_repaired.Count -gt 0) {
            $repaired_wheel_path = $existing_repaired[0].FullName
            Write-Host "Using existing repaired wheel: $(Split-Path $repaired_wheel_path -Leaf)" -ForegroundColor Cyan
        } else {
            Write-Host "ERROR: No existing repaired wheel found in $RepairedDir. Use -Repair to create one." -ForegroundColor Red
            exit 1
        }
    }
    
    # Install phase
    if ($Install -or $All) {
        Install-Wheel -WheelPath $repaired_wheel_path
        Test-Installation
    }
    
    Write-Host ""
    Write-Host "COMPLETE: All operations completed successfully!" -ForegroundColor Green
    
    if ($All -or $Install) {
        Write-Host ""
        Write-Host "The PCL Python bindings are now installed and ready to use." -ForegroundColor Cyan
        Write-Host "The installed wheel is self-contained and includes all required DLLs." -ForegroundColor Cyan
    }
    
} catch {
    Write-Host "ERROR: Script failed with error: $_" -ForegroundColor Red
    exit 1
}
