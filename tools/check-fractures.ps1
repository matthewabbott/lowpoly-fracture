# Milestone 11a's fracture check: lpf_bench --check-fractures on four rungs at 1 and 8 workers (src/fcheck.h).
#   pwsh tools/check-fractures.ps1                       # keep, barrage, siege, town: two tables; JSON in build/fractures/
#   pwsh tools/check-fractures.ps1 -Record               # also each rung's recording, build/fractures/<rung>.lpfr
#   pwsh tools/check-fractures.ps1 -ReportOnly:$false    # fail on any exactness violation (from C5)
# Rungs as in tools/bench.ps1: keep (the keep, a blast every 12 ticks), barrage (town every 3), siege (keep every 4), town
# (every 12). Fails on a crash, on worker counts whose hashes differ (exit 2) or whose fracture counts differ (exit 6),
# and, with -ReportOnly:$false, on an exactness violation (exit 5). Floats have violations until C5, so reporting is
# the default. A recording replays alone: lpf_bench --replay-fractures build/fractures/barrage.lpfr [--job k] --repeat 3
param(
    [string[]]$Rungs = @('keep', 'barrage', 'siege', 'town'),
    [string]$Workers = '1,8',
    [int]$Ticks = 600,
    [string]$Preset = 'msvc-release',
    [switch]$Record,
    [bool]$ReportOnly = $true
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = "build/$Preset/bin/lpf_bench.exe"
$out = 'build/fractures'
New-Item -ItemType Directory -Force $out | Out-Null

$defs = @{ keep = @{ scene = 'keep'; period = 12 }; barrage = @{ scene = 'town'; period = 3 }; siege = @{ scene = 'keep'; period = 4 }; town = @{ scene = 'town'; period = 12 } }
$failed = $false
$results = [ordered]@{}
foreach ($rung in $Rungs) {
    $d = if ($defs.ContainsKey($rung)) { $defs[$rung] } else { @{ scene = $rung; period = 12 } }
    $json = "$out/$rung.json"
    $a = @('--scene', $d.scene, '--period', $d.period, '--ticks', $Ticks, '--workers', $Workers, '--check-fractures', '--json', $json)
    if ($Record) { $a += @('--record-fractures', "$out/$rung.lpfr") }
    if (Test-Path $json) { Remove-Item $json }
    & $exe @a | Out-File "$out/$rung.txt"
    $code = $LASTEXITCODE
    if ($code -eq 2) { Write-Host "NONDETERMINISTIC: $rung's hashes differ across worker counts"; $failed = $true }
    elseif ($code -eq 6) { Write-Host "NONDETERMINISTIC: $rung's fracture counts differ across worker counts"; $failed = $true }
    elseif ($code -eq 5) { if (-not $ReportOnly) { Write-Host "VIOLATIONS: $rung"; $failed = $true } }
    elseif ($code -ne 0) { Write-Host "lpf_bench failed on $rung (exit $code; see $out/$rung.txt)"; $failed = $true; continue }
    if (-not (Test-Path $json)) { Write-Host "no JSON from $rung"; $failed = $true; continue }
    $runs = (Get-Content $json -Raw | ConvertFrom-Json).runs
    # The counts must agree between worker counts (the bench checks it too: exit 6); timings may not
    $counts = $runs | ForEach-Object { $_.fractures | Select-Object * -ExcludeProperty jobCpuMs, validateMs | ConvertTo-Json -Compress -Depth 4 }
    if (($counts | Select-Object -Unique).Count -ne 1) { Write-Host "COUNTS DIFFER between worker counts on $rung"; $failed = $true }
    $results[$rung] = $runs
}

function Sum($values) { ($values | Measure-Object -Sum).Sum }
function Max($values) { ($values | Measure-Object -Maximum).Maximum }

Write-Host ''
Write-Host 'what the jobs did (shifts: clips that pushed their plane; merges: tried/accepted; bonds: by tag/by contact)'
Write-Host ('{0,-8} {1,2} | {2,6} {3,7} | {4,8} {5,-17} {6,5} | {7,-13} {8,6} | {9,-13} {10,5} {11,-11} | {12}' -f 'rung', 'w', 'jobs', 'cells',
    'clips', 'shifts (%, mm)', 'fails', 'merges', 'qhulls', 'bonds', 'hullX', 'chips', 'hash')
foreach ($rung in $results.Keys) {
    foreach ($run in $results[$rung]) {
        $f = $run.fractures
        $clips = Sum $f.clips
        $shifts = Sum $f.planeShifts
        $share = if ($clips -gt 0) { 100.0 * $shifts / $clips } else { 0 }
        Write-Host ('{0,-8} {1,2} | {2,6} {3,7} | {4,8} {5,-17} {6,5} | {7,-13} {8,6} | {9,-13} {10,5} {11,-11} | {12}' -f $rung, $run.workers,
            $f.jobs, $f.outputCells, $clips, ('{0} ({1:F2}, {2:F3})' -f $shifts, $share, (1000.0 * $f.maxShift)), (Sum $f.clipFailures),
            ('{0}/{1}' -f $f.mergeTried, $f.mergeAccepted), $f.mergeHulls, ('{0}/{1}' -f $f.bondsByTag, $f.bondsByContact),
            ($f.hullFailsLarge + $f.hullFailsOther), ('{0}->{1}' -f $f.ghostsChipped, $f.chipsMade), $run.hash)
    }
}
Write-Host ''
Write-Host 'what the checks found (violations; siblings: exact/covered/near/unmatched; census of the cells out)'
Write-Host ('{0,-8} {1,2} | {2,6} | {3,-16} {4,-18} {5,-23} {6,-14} {7,-14} {8,-9} | {9,-8} {10,-9} {11}' -f 'rung', 'w', 'viol',
    'tiling (n, max)', 'overlap (n, m^3)', 'siblings', 'not convex', 'contain', 'chips', 'max |x|', 'min edge', 'close')
foreach ($rung in $results.Keys) {
    foreach ($run in $results[$rung]) {
        $f = $run.fractures
        Write-Host ('{0,-8} {1,2} | {2,6} | {3,-16} {4,-18} {5,-23} {6,-14} {7,-14} {8,-9} | {9,-8:F2} {10,-9:G3} {11}' -f $rung, $run.workers,
            $f.violations, ('{0} {1:G2}' -f $f.tilingViolations, $f.tilingMaxError), ('{0} {1:G2}' -f $f.overlapViolations, $f.overlapMax),
            ('{0}/{1}/{2}/{3}' -f $f.siblingExact, $f.siblingCovered, $f.siblingNear, $f.siblingUnmatched),
            ('{0} {1:G2}' -f $f.convexViolations, $f.maxConvexExcess), ('{0} {1:G2}' -f $f.containViolations, $f.maxContainExcess),
            ('{0}/{1}' -f $f.chipViolations, $f.chipSets), $f.census.maxCoordinate, $f.census.minEdge, $f.census.closePairs)
    }
}
Write-Host ''
Write-Host "JSON and the bench's own summaries: $out/<rung>.json, $out/<rung>.txt$(if ($Record) { "; recordings: $out/<rung>.lpfr" })"
if ($failed) { exit 1 }
exit 0
