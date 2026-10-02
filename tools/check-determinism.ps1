# Replays the same scripted session headless (lpf_bench) at 1, 4 and 8 workers and compares per-tick state hashes
# (the world's and the stress solver's).
#   pwsh tools/check-determinism.ps1 [-Scene walls] [-Script scripts/walls_demo.txt] [-Ticks 240] [-Period 0]
# -Period N also bombards the scene every N ticks (lpSceneBombard). The bench replays a script exactly as the sandbox
# does (same world settings, same order of events), so its hashes match the sandbox's --hash-log.
param(
    [string]$Scene = 'walls',
    [string]$Script = 'scripts/walls_demo.txt',
    [int]$Ticks = 240,
    [int]$Period = 0,
    [string]$Preset = 'msvc-release'
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = "build/$Preset/bin/lpf_bench.exe"
$out = "build/determinism"
New-Item -ItemType Directory -Force $out | Out-Null

$prefix = "$out/hash_$Scene"
$a = @('--scene', $Scene, '--ticks', $Ticks, '--period', $Period, '--workers', '1,4,8', '--hash-log', $prefix)
if ($Script) { $a += @('--script', $Script) }
& $exe @a | Out-File "$out/bench_$Scene.txt"
if ($LASTEXITCODE -eq 1) { Write-Host "lpf_bench failed"; exit 1 }

$logs = @(1, 4, 8 | ForEach-Object { "$prefix.w$_.txt" })
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
if ($ok) { Write-Host "deterministic: $($reference.Count - 1) ticks identical across 1, 4, 8 workers (final $($reference[-1]))"; exit 0 }
exit 2
