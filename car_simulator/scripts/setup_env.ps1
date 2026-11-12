<#
.SYNOPSIS
    One-time setup: ScenarioRunner, .venv (Python 3.11 via uv) and all Python dependencies.

.DESCRIPTION
    Thin wrapper around scripts\setup_env.py, which does the work and can also be
    run directly with any Python on PATH:  python scripts\setup_env.py

.EXAMPLE
    .\scripts\setup_env.ps1
    .\scripts\setup_env.ps1 -Recreate
#>
param(
    [string]$PythonVersion = "3.11",
    [switch]$Recreate
)

$SetupArgs = @((Join-Path $PSScriptRoot "setup_env.py"), "--python", $PythonVersion)
if ($Recreate) { $SetupArgs += "--recreate" }

& python @SetupArgs
exit $LASTEXITCODE
