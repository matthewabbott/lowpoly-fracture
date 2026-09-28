# Benchmark ladder: runs lpf_bench on a fixed set of scenes and compares against the committed baseline.
#   pwsh tools/bench.ps1                 # compare with bench/baseline.json; fails if a simulation hash changed
#   pwsh tools/bench.ps1 -AcceptHashes   # compare timings only (after an intended behaviour change)
#   pwsh tools/bench.ps1 -Update         # write the current numbers as the new baseline
#   pwsh tools/bench.ps1 -Repeat 3       # best of 3 runs per scene (timings are noisy, about +-10% run to run)
# A rung is a scene under the standard bombardment, or 'barrage': town with a blast every 3 ticks, many structures
# breaking and re-solving at once (the stress solve's parallel case).
# After a behaviour change, a rung's timings can move because a different amount comes down (destruction is chaotic):
# compare its pieces and contacts, or try other --period values with lpf_bench, before calling it a regression.
param(
    [string[]]$Scenes = @('walls', 'town', 'pile', 'lumber', 'tower', 'ruins', 'barrage'),
    [string]$Workers = '1,8',
    [int]$Ticks = 600,
    [int]$Period = 12,
    [string]$Baseline = 'bench/baseline.json',
    [string]$Preset = 'msvc-release',
    [int]$Repeat = 1,
    [switch]$Update,
    [switch]$AcceptHashes
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = "build/$Preset/bin/lpf_bench.exe"
$out = 'build/bench'
New-Item -ItemType Directory -Force $out | Out-Null

$rungs = @{ barrage = @{ scene = 'town'; period = 3 } }
$current = [ordered]@{}
foreach ($scene in $Scenes) {
    $name = $scene
    $every = $Period
    if ($rungs.ContainsKey($scene)) { $name = $rungs[$scene].scene; $every = $rungs[$scene].period }
    $best = $null
    for ($r = 0; $r -lt $Repeat; ++$r) {
        $json = "$out/$scene.json"
        & $exe --scene $name --workers $Workers --ticks $Ticks --period $every --json $json | Out-Null
        if ($LASTEXITCODE -eq 2) { Write-Host "NONDETERMINISTIC: $scene differs across worker counts"; exit 2 }
        if ($LASTEXITCODE -ne 0) { Write-Host "lpf_bench failed on $scene"; exit 1 }
        $runs = (Get-Content $json -Raw | ConvertFrom-Json).runs
        if ($null -eq $best) { $best = $runs; continue }
        for ($i = 0; $i -lt $runs.Count; ++$i) {
            foreach ($key in 'stepAvgMs', 'stepP95Ms', 'stepMaxMs', 'fractureAvgMs', 'fractureMaxMs', 'physicsAvgMs', 'physicsP95Ms') {
                if ($runs[$i].$key -lt $best[$i].$key) { $best[$i].$key = $runs[$i].$key }
            }
        }
    }
    $current[$scene] = $best
}

$commit = (git rev-parse --short HEAD).Trim()
if ($Update -or -not (Test-Path $Baseline)) {
    $doc = [ordered]@{ commit = $commit; date = (Get-Date -Format 'yyyy-MM-dd'); ticks = $Ticks; period = $Period; scenes = $current }
    $doc | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 $Baseline
    Write-Host "baseline written: $Baseline (commit $commit)"
    exit 0
}

$base = Get-Content $Baseline -Raw | ConvertFrom-Json
function Delta($now, $was) {
    if ($null -eq $was -or $was -eq 0) { return ('{0,7:F2}' -f $now) }
    $pct = 100.0 * ($now - $was) / $was
    return ('{0,7:F2} ({1:+0;-0;0}%)' -f $now, $pct)
}

Write-Host "vs baseline $($base.commit) ($($base.date)); ms per step, change in brackets"
Write-Host ('{0,-7} {1,3} | {2,-17} {3,-17} {4,-17} | {5,-8} {6,-15} | {7}' -f 'scene', 'w', 'avg', 'p95', 'max', 'pieces', 'contacts', 'hash')
$hashChanged = $false
foreach ($scene in $Scenes) {
    $was = $base.scenes.$scene
    foreach ($run in $current[$scene]) {
        $old = $was | Where-Object { $_.workers -eq $run.workers } | Select-Object -First 1
        $same = $old -and $old.hash -eq $run.hash
        if ($old -and -not $same) { $hashChanged = $true } # a scene new to the baseline is not a change
        $mark = if ($same) { 'same' } elseif ($old) { "CHANGED (was $($old.hash))" } else { 'new' }
        $contacts = if ($null -ne $run.awakeContactsAvg) { Delta $run.awakeContactsAvg $old.awakeContactsAvg } else { '-' }
        Write-Host ('{0,-7} {1,3} | {2,-17} {3,-17} {4,-17} | {5,-8} {6,-15} | {7} {8}' -f $scene, $run.workers,
            (Delta $run.stepAvgMs $old.stepAvgMs), (Delta $run.stepP95Ms $old.stepP95Ms), (Delta $run.stepMaxMs $old.stepMaxMs),
            $run.maxPieces, $contacts, $run.hash, $mark)
    }
}
if ($hashChanged -and -not $AcceptHashes) {
    Write-Host 'simulation hashes changed: behaviour differs from the baseline (use -AcceptHashes if intended, then -Update)'
    exit 3
}
exit 0
