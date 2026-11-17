<#
.SYNOPSIS
    Starts the CARLA 0.9.16 server.

.EXAMPLE
    .\scripts\start_carla.ps1                    # Low quality, good for large maps like Town12
    .\scripts\start_carla.ps1 -Quality Epic
    .\scripts\start_carla.ps1 -OffScreen         # no window (sensors still render)
#>
param(
    [ValidateSet("Low", "Epic")]
    [string]$Quality = "Low",
    [int]$Port = 2000,
    [switch]$OffScreen
)

$RepoRoot = Split-Path -Parent $PSScriptRoot
$CarlaRoot = if ($env:CARLA_ROOT) { $env:CARLA_ROOT } else { Join-Path $RepoRoot "CARLA_0.9.16" }
$Exe = Join-Path $CarlaRoot "CarlaUE4.exe"
if (-not (Test-Path $Exe)) { throw "CarlaUE4.exe not found in $CarlaRoot (set CARLA_ROOT)" }

# -prefernvidia: on laptops with an integrated GPU too, render on the NVIDIA one
$CarlaArgs = @("-quality-level=$Quality", "-carla-rpc-port=$Port", "-prefernvidia")
if ($OffScreen) { $CarlaArgs += "-RenderOffScreen" }

Write-Host "Starting $Exe $($CarlaArgs -join ' ')"
Start-Process -FilePath $Exe -ArgumentList $CarlaArgs -WorkingDirectory $CarlaRoot
