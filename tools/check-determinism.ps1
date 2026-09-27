# Replays the same scripted session in the sandbox with different worker counts and compares per-tick state hashes.
#   pwsh tools/check-determinism.ps1 [-Scene walls] [-Script scripts/walls_demo.txt] [-Frames 240] [-Bombard 0]
param(
    [string]$Scene = 'walls',
    [string]$Script = 'scripts/walls_demo.txt',
    [int]$Frames = 240,
    [int]$Bombard = 0,
    [string]$Preset = 'msvc-release'
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = "build/$Preset/bin/sandbox.exe"
$out = "build/determinism"
New-Item -ItemType Directory -Force $out | Out-Null

$logs = @()
foreach ($workers in 1, 4, 8) {
    $log = "$out/hash_$Scene`_w$workers.txt"
    $args = "--scene $Scene --frames $Frames --workers $workers --hash-log $log --hide-ui --vsync 0"
    if ($Script) { $args += " --script $Script" }
    if ($Bombard -gt 0) { $args += " --bombard $Bombard" }
    $p = Start-Process -FilePath $exe -ArgumentList $args -NoNewWindow -PassThru -Wait -RedirectStandardOutput "$out/stdout_w$workers.txt"
    if ($p.ExitCode -ne 0) { Write-Host "sandbox failed with workers=$workers"; exit 1 }
    $logs += ,$log
}

$reference = Get-Content $logs[0]
$ok = $true
for ($i = 1; $i -lt $logs.Count; ++$i) {
    $other = Get-Content $logs[$i]
    if ($other.Count -ne $reference.Count) { Write-Host "length differs: $($logs[$i])"; $ok = $false; continue }
    for ($t = 0; $t -lt $reference.Count; ++$t) {
        if ($reference[$t] -ne $other[$t]) {
            Write-Host "DIVERGED at line $t between $($logs[0]) and $($logs[$i]): $($reference[$t]) vs $($other[$t])"
            $ok = $false
            break
        }
    }
}
if ($ok) { Write-Host "deterministic: $($reference.Count) ticks identical across 1, 4, 8 workers (final $($reference[-1]))" ; exit 0 }
exit 2
