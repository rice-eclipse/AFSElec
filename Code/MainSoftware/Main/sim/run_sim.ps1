<#
  run_sim.ps1 - Builds the flight replay simulator (if anything changed) and runs it.

    .\sim\run_sim.ps1 flights\nav_003.csv                  -> flights\nav_003_sim.csv
    .\sim\run_sim.ps1 flights\nav_003.csv out.csv
    .\sim\run_sim.ps1 flights\nav_003.csv -FromFirmwareConfig   # first copy the firmware's config.h
                                                                 # over sim\config.h (your sim edits are lost)

  Settings: sim\config.h (flight software) and sim\sim_settings.h (sim only).
  Needs CMake and Visual Studio 2022 with the C++ workload.
#>
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$InputCsv,
    [Parameter(Position = 1)][string]$OutputCsv = "",
    [switch]$FromFirmwareConfig
)
$ErrorActionPreference = 'Stop'
$here  = $PSScriptRoot
$build = Join-Path $here 'build'

if ($FromFirmwareConfig) {
    Copy-Item (Join-Path $here '..\config.h') (Join-Path $here 'config.h') -Force
    Write-Host "sim\config.h replaced with the firmware's config.h"
}

if (-not (Test-Path (Join-Path $build 'CMakeCache.txt'))) {
    cmake -S $here -B $build -G "Visual Studio 17 2022" -A x64 | Out-Null
    if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed" }
}
cmake --build $build --config Release -- /nologo /verbosity:minimal
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed" }

$exe = Join-Path $build 'Release\navsim.exe'
if ($OutputCsv) { & $exe $InputCsv $OutputCsv } else { & $exe $InputCsv }
exit $LASTEXITCODE
