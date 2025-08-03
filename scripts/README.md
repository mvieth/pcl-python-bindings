# PCL Python Bindings - Build Scripts

This directory contains production-ready PowerShell scripts to automate building, repairing, and installing PCL Python bindings wheels on Windows.

## Scripts Overview

### 🔄 `load-env.ps1`
**Environment variable loading script**

Load all required environment variables from `.env.local` or `.env` file:
```powershell
.\scripts\load-env.ps1
```

### 🔧 `build-wheel.ps1` (Primary Script)
**PowerShell script for complete wheel build and repair workflow**

**Usage:**
```powershell
# Full workflow (recommended for development)
.\scripts\build-wheel.ps1 -All

# Individual operations
.\scripts\build-wheel.ps1 -Build      # Build wheel only
.\scripts\build-wheel.ps1 -Repair     # Repair with delvewheel only  
.\scripts\build-wheel.ps1 -Install    # Install repaired wheel only
.\scripts\build-wheel.ps1 -Clean      # Clean build directories

# Debugging
.\scripts\build-wheel.ps1 -All -Verbose
```

**Features:**
- ✅ Automatically loads environment variables from `.env.local` or `.env` (prioritizes `.env.local`)
- ✅ Builds wheel using scikit-build-core
- ✅ Repairs wheel with delvewheel (bundles all required DLLs)
- ✅ Installs and tests the repaired wheel
- ✅ Comprehensive error handling and status reporting
- ✅ Supports individual operations or complete workflow

### 🔄 `load-env.ps1`
**Environment variable loading script**

Load all required environment variables from `.env` or `.env.local` file:
```powershell
.\scripts\load-env.ps1
```

## Environment Requirements

The scripts require these environment variables (set in `.env.local` or `.env` file):

```properties
# Example paths - update to match your actual installations
VCPKG_ROOT=C:/vcpkg
VCPKG_TARGET_TRIPLET=x64-windows
PCL_DIR=C:/Program Files/PCL/cmake
PCL_BIN_DIR=C:/Program Files/PCL/bin
VCPKG_BIN_DIR=C:/vcpkg/installed/x64-windows/bin
```

**💡 Tip**: Use `.env.local` for personal development paths (git-ignored) and `.env` for shared examples.

## Workflow Explanation

### 1. **Build Phase** (`-Build`)
- Uses `pip wheel . --no-build-isolation` 
- Creates wheel in `dist/` directory
- Uses scikit-build-core + CMake + nanobind

### 2. **Repair Phase** (`-Repair`) 
- Uses `delvewheel` to analyze dependencies
- Copies required DLLs from PCL and vcpkg directories
- Bundles DLLs into wheel's `.libs` directory
- Creates self-contained wheel in `dist_repaired/`

### 3. **Install Phase** (`-Install`)
- Installs the repaired wheel with `pip install --force-reinstall`
- Tests imports to verify installation
- The installed package works without environment variables

## VS Code Integration

The scripts are integrated with VS Code tasks (`.vscode/tasks.json`):

- **Build Wheel (Complete)** - Full workflow
- **Build Wheel Only** - Build phase only
- **Repair Wheel with Delvewheel** - Repair phase only  
- **Install Repaired Wheel** - Install phase only
- **Clean Build Directories** - Clean up

Access via: `Ctrl+Shift+P` → "Tasks: Run Task"

## Local Development vs Production

### Local Development (These Scripts)
- Use `build-wheel.ps1 -All` for development
- Creates self-contained wheels with delvewheel
- Fast iteration for testing changes

### Production (CI/CD)
- Use **cibuildwheel** configured in `pyproject.toml`
- Automatically builds for multiple Python versions
- Handles delvewheel repair via `[tool.cibuildwheel.windows]`
- Uploads to PyPI with proper distribution metadata

## Troubleshooting

### Common Issues

1. **"Environment variables not set"**
   - Ensure `.env` file exists with all required variables
   - Run `.\scripts\load-env.ps1` to verify environment

2. **"delvewheel not found"**
   - Scripts automatically install delvewheel if missing
   - Manual install: `pip install delvewheel`

3. **"No wheel files found"**
   - Build phase failed - check CMake and dependencies
   - Ensure environment variables point to valid directories

4. **"DLL load failed"**
   - Use repaired wheel from `dist_repaired/` directory
   - Original wheel from `dist/` lacks bundled DLLs

### Debug Mode
```powershell
.\scripts\build-wheel.ps1 -All -Verbose
```

Shows detailed output for all operations.

## File Structure After Build

```
pcl-python-bindings/
├── dist/                           # Original wheels (need DLLs in PATH)
│   └── pcl-0.0.1-cp312-abi3-win_amd64.whl
├── dist_repaired/                  # Self-contained wheels (recommended)
│   └── pcl-0.0.1-cp312-abi3-win_amd64.whl
└── build/                          # CMake build artifacts
    └── cp312-abi3-win_amd64/
```

The **repaired wheel** in `dist_repaired/` is self-contained and includes:
- All PCL Python extension modules (`.pyd` files)
- Required DLLs in `pcl.libs/` directory  
- Modified `__init__.py` that loads DLLs automatically

## Summary

For most development work, simply run:
```powershell
.\scripts\build-wheel.ps1 -All
```

This handles the complete workflow and produces a ready-to-use, self-contained wheel that works on any Windows machine without requiring PCL or vcpkg to be installed.
