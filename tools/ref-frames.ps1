# Grabs frames from a reference clip at given times (docs/references.md) and tiles them into a sheet beside the
# catalogue entry's sandbox sheet, so the two can be compared frame by frame. Everything it writes stays in refs/
# (third-party footage: personal reference, never committed). Needs ffmpeg on PATH.
#   pwsh tools/ref-frames.ps1 -Clip refs/felling.mp4 -At 1.0,1.6,2.2,3.0 -Entry S5
param(
    [Parameter(Mandatory = $true)][string]$Clip,
    [Parameter(Mandatory = $true)][string]$At,
    [Parameter(Mandatory = $true)][string]$Entry,
    [int]$Width = 960
)
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) { Write-Host 'ffmpeg is not on PATH'; exit 1 }
$dir = "refs/$Entry"
New-Item -ItemType Directory -Force $dir | Out-Null
$name = [IO.Path]::GetFileNameWithoutExtension($Clip)
$times = @($At -split ',')
$height = [int]($Width * 9 / 16)

$frames = @()
foreach ($t in $times) {
    $f = "$dir/$name`_$t.png"
    & ffmpeg -y -loglevel error -ss $t -i $Clip -frames:v 1 -vf "scale=$($Width / 2):$($height / 2):force_original_aspect_ratio=decrease,pad=$($Width / 2):$($height / 2):(ow-iw)/2:(oh-ih)/2" $f
    $frames += $f
}
$inputs = @()
foreach ($f in $frames) { $inputs += @('-i', $f) }
$n = $frames.Count
$cells = ''
$pad = ''
if ($n % 2 -eq 1) { $pad = "color=black:s=$($Width / 2)x$($height / 2)[x$n];"; $n += 1 }
$layout = @(for ($k = 0; $k -lt $n; ++$k) { $x = if ($k % 2 -eq 0) { '0' } else { 'w0' }; $y = if ($k -lt 2) { '0' } else { (@('h0') * [math]::Floor($k / 2)) -join '+' }; "$x`_$y" }) -join '|'
$labels = (@(for ($k = 0; $k -lt $n; ++$k) { if ($k -lt $frames.Count) { "[$k]" } else { "[x$k]" } }) -join '')
$sheet = "$dir/$name`_sheet.png"
& ffmpeg -y -loglevel error @inputs -filter_complex ($pad + $labels + "xstack=inputs=$n`:layout=$layout") -frames:v 1 $sheet
Write-Host "reference sheet: $sheet"

# Beside the sandbox's sheet for the entry, when there is one
$ours = "docs/catalogue/$Entry.png"
if (Test-Path $ours) {
    $compare = "$dir/$name`_compare.png"
    & ffmpeg -y -loglevel error -i $sheet -i $ours -filter_complex "[1]scale=$Width`:-2[o];[0]scale=$Width`:-2[r];[r][o]vstack=inputs=2" -frames:v 1 $compare
    Write-Host "reference above, sandbox below: $compare"
}
