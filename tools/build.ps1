# One-stop build for agents and humans.
#   pwsh tools/build.ps1                      # msvc-release: configure + build
#   pwsh tools/build.ps1 -Preset msvc-debug -Test
#   pwsh tools/build.ps1 -Shaders             # regenerate shader headers with sokol-shdc first
param(
    [string]$Preset = 'msvc-release',
    [switch]$Test,
    [switch]$Shaders,
    [switch]$Fresh
)
$root = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'devenv.ps1')
Set-Location $root

if ($Shaders) {
    $shdc = Join-Path $PSScriptRoot 'bin\sokol-shdc.exe'
    if (-not (Test-Path $shdc)) { & (Join-Path $PSScriptRoot 'get-shdc.ps1') }
    Get-ChildItem app/sandbox/shaders -Filter *.glsl | ForEach-Object {
        $out = Join-Path $_.DirectoryName ("generated\" + $_.Name + '.h')
        & $shdc --input $_.FullName --output $out --slang hlsl5 --format sokol
        if ($LASTEXITCODE -ne 0) { throw "sokol-shdc failed on $($_.Name)" }
    }
}

if ($Fresh -and (Test-Path "build/$Preset")) { Remove-Item -Recurse -Force "build/$Preset" }
if (-not (Test-Path "build/$Preset/CMakeCache.txt")) {
    cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
cmake --build --preset $Preset
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($Test) {
    ctest --preset $Preset
    exit $LASTEXITCODE
}
