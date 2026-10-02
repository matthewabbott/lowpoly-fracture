# Renders the outcome catalogue's contact sheets (docs/catalogue/shots.txt): each line runs a scene and a replay script
# in the sandbox, saves frames at the listed ticks, and tiles them two to a row into docs/catalogue/<id>.png (small,
# compressed: committed, so any agent or reviewer can look without building). Needs ffmpeg on PATH and the sandbox build.
#   pwsh tools/catalogue-shots.ps1 [-Only F9] [-Preset msvc-release] [-Width 960]
param(
    [string]$Only = '',
    [string]$Preset = 'msvc-release',
    [int]$Width = 960
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$exe = "build/$Preset/bin/sandbox.exe"
$work = 'build/shots/catalogue'
New-Item -ItemType Directory -Force $work | Out-Null
if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) { Write-Host 'ffmpeg is not on PATH'; exit 1 }
$height = [int]($Width * 9 / 16)

foreach ($line in Get-Content 'docs/catalogue/shots.txt') {
    if ($line -match '^\s*(#|$)') { continue }
    # id scene script ticks view   (view: follow, or camera:x,y,z,yawDeg,pitchDeg)
    $id, $scene, $script, $ticks, $view = -split $line
    if ($Only -and $id -ne $Only) { continue }
    $frames = @($ticks -split ',' | ForEach-Object { [int]$_ })
    $last = ($frames | Measure-Object -Maximum).Maximum
    Get-ChildItem "$work/$id`_*.png" -ErrorAction SilentlyContinue | Remove-Item
    $a = @('--scene', $scene, '--frames', $last, '--screenshot-at', $ticks, '--screenshot', "$work/$id.png", '--hide-ui',
        '--vsync', '0', '--width', $Width, '--height', $height, '--workers', 4)
    if ($script -ne '-') { $a += @('--script', "scripts/$script") }
    if ($view -eq 'follow') { $a += '--follow' } elseif ($view -like 'camera:*') { $a += @('--camera', $view.Substring(7)) }
    & $exe @a | Out-Null
    $shots = @($frames | ForEach-Object { '{0}/{1}_{2:D4}.png' -f $work, $id, $_ })
    $missing = @($shots | Where-Object { -not (Test-Path $_) })
    if ($missing.Count -gt 0) { Write-Host "$id`: missing $($missing -join ', ')"; continue }

    # Two to a row, each at half width; an odd last frame is padded with black
    $inputs = @()
    foreach ($s in $shots) { $inputs += @('-i', $s) }
    $n = $shots.Count
    $cells = ''
    for ($k = 0; $k -lt $n; ++$k) { $cells += "[$k]scale=$($Width / 2):-2[s$k];" }
    if ($n % 2 -eq 1) { $cells += "color=black:s=$($Width / 2)x$($height / 2)[s$n];"; $n += 1 }
    $layout = @(for ($k = 0; $k -lt $n; ++$k) { $x = if ($k % 2 -eq 0) { '0' } else { 'w0' }; $y = if ($k -lt 2) { '0' } else { (@('h0') * [math]::Floor($k / 2)) -join '+' }; "$x`_$y" }) -join '|'
    $stack = (@(for ($k = 0; $k -lt $n; ++$k) { "[s$k]" }) -join '') + "xstack=inputs=$n`:layout=$layout,split[a][b];[a]palettegen=max_colors=192[p];[b][p]paletteuse=dither=none"
    $out = "docs/catalogue/$id.png"
    & ffmpeg -y -loglevel error @inputs -filter_complex ($cells + $stack) -frames:v 1 $out
    Write-Host ('{0}: {1} frames, {2:N0} KB' -f $id, $shots.Count, ((Get-Item $out).Length / 1KB))
}
