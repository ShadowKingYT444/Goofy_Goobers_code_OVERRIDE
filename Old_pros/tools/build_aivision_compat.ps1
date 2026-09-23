param(
    [string]$SourceArchive = "ai_vision_smoke\firmware\libpros.a",
    [string]$OutputArchive = "firmware\libaivision_compat.a"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$toolchain = Join-Path $root ".pros-toolchain\usr\bin"
$ar = Join-Path $toolchain "arm-none-eabi-ar.exe"
$ld = Join-Path $toolchain "arm-none-eabi-ld.exe"
$work = Join-Path $root "aivision_compat_smoke\compat_objs"
New-Item -ItemType Directory -Force $work | Out-Null

Push-Location $work
try {
    & $ar x (Join-Path $root $SourceArchive) vdml_ai_vision.c.o v5_apijump.c.obj
    if ($LASTEXITCODE -ne 0) { throw "failed to extract AI Vision objects" }
} finally {
    Pop-Location
}

$jumpInput = Join-Path $work "v5_apijump.c.obj"
$jumpOutput = Join-Path $work "v5_apijump_aivision_only.o"
$symbols = @(
    "vexDeviceAiVisionClassNameGet", "vexDeviceAiVisionCodeGet",
    "vexDeviceAiVisionCodeSet", "vexDeviceAiVisionColorGet",
    "vexDeviceAiVisionColorSet", "vexDeviceAiVisionEnableGet",
    "vexDeviceAiVisionEnableSet", "vexDeviceAiVisionModeSet",
    "vexDeviceAiVisionObjectCountGet", "vexDeviceAiVisionObjectGet",
    "vexDeviceAiVisionReset", "vexDeviceAiVisionStatusGet",
    "vexDeviceAiVisionTemperatureGet"
)
$ldArgs = @("-r", "--gc-sections")
foreach ($symbol in $symbols) { $ldArgs += @("-u", $symbol) }
$ldArgs += @($jumpInput, "-o", $jumpOutput)
& $ld @ldArgs
if ($LASTEXITCODE -ne 0) { throw "failed to isolate AI Vision VEX API jumps" }

& $ar rcs (Join-Path $root $OutputArchive) (Join-Path $work "vdml_ai_vision.c.o") $jumpOutput
if ($LASTEXITCODE -ne 0) { throw "failed to build AI Vision compatibility archive" }
Write-Host "Built $OutputArchive"
