# The core's size in tokens, measured one way every time: LF-normalised bytes / 4 over src/ and include/lpf/ (the
# milestone 9 review logs it before and after). With -Commit, at that commit instead of the working tree.
#   pwsh tools/size.ps1 [-Commit f38a72d]
param([string]$Commit = '')
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$files = if ($Commit) { git ls-tree -r --name-only $Commit -- src include/lpf } else { git ls-files -- src include/lpf }
$files = @($files | Where-Object { $_ -match '\.(c|h)$' })
$bytes = 0
$lines = 0
foreach ($f in $files) {
    $text = if ($Commit) { (git show "${Commit}:$f") -join "`n" } else { (Get-Content -Raw $f) -replace "`r`n", "`n" }
    $bytes += [Text.Encoding]::UTF8.GetByteCount($text)
    $lines += ($text -split "`n").Count
}
Write-Host ("core: {0} files, {1} lines, {2:N0} bytes, about {3:N1}k tokens" -f $files.Count, $lines, $bytes, ($bytes / 4000))
