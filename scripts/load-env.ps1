# Load environment variables from .env file
# This script can be sourced in PowerShell to set up the environment
# Prioritizes .env.local over .env if it exists

$projectRoot = Split-Path $PSScriptRoot -Parent
$envLocalFile = Join-Path $projectRoot ".env.local"
$envFile = Join-Path $projectRoot ".env"

# Choose which file to load (prioritize .env.local)
$fileToLoad = $null
if (Test-Path $envLocalFile) {
    $fileToLoad = $envLocalFile
    Write-Host "Loading environment from .env.local file..." -ForegroundColor Green
} elseif (Test-Path $envFile) {
    $fileToLoad = $envFile
    Write-Host "Loading environment from .env file..." -ForegroundColor Green
} else {
    Write-Warning "Neither .env.local nor .env file found"
    Write-Warning "Expected locations:"
    Write-Warning "  $envLocalFile"
    Write-Warning "  $envFile"
    return
}

Get-Content $fileToLoad | ForEach-Object {
    if ($_ -match '^([^#=]+)=(.*)$') {
        $name = $matches[1].Trim()
        $value = $matches[2].Trim()
        
        # Skip comments and empty lines
        if (-not $name.StartsWith("#") -and $name -ne "") {
            [Environment]::SetEnvironmentVariable($name, $value, "Process")
            Write-Host "  Set $name = $value" -ForegroundColor Cyan
        }
    }
}

Write-Host "Environment loaded successfully!" -ForegroundColor Green
Write-Host "PCL_BIN_DIR = $env:PCL_BIN_DIR" -ForegroundColor Yellow
Write-Host "VCPKG_BIN_DIR = $env:VCPKG_BIN_DIR" -ForegroundColor Yellow
