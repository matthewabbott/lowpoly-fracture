# Dot-source this to get a VS2022 x64 developer environment plus CMake, Ninja and LLVM on PATH.
#   . tools/devenv.ps1
# Idempotent: skips vcvars if cl.exe is already on PATH.

$ErrorActionPreference = 'Stop'

$extra = @(
    'C:\Program Files\CMake\bin',
    'C:\Program Files\LLVM\bin',
    (Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages\Ninja-build.Ninja_Microsoft.Winget.Source_8wekyb3d8bbwe')
)
foreach ($p in $extra) {
    if ((Test-Path $p) -and -not (($env:Path -split ';') -contains $p)) { $env:Path = "$p;$env:Path" }
}

if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    $vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vs) { throw 'Visual Studio with C++ tools not found' }
    $vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
    $envDump = cmd /c "`"$vcvars`" >nul 2>&1 && set"
    foreach ($line in $envDump) {
        if ($line -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($Matches[1])" -Value $Matches[2] }
    }
    # vcvars rewrites PATH; put our tools back in front.
    foreach ($p in $extra) {
        if ((Test-Path $p) -and -not (($env:Path -split ';') -contains $p)) { $env:Path = "$p;$env:Path" }
    }
}
$ErrorActionPreference = 'Continue'
