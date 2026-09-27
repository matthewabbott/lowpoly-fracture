# Fetch sokol-shdc (shader cross-compiler) at the commit Box3D's samples pin.
$commit = '1a9a4e54090fec42c5d13169b638f09f25474953'
$dir = Join-Path $PSScriptRoot 'bin'
New-Item -ItemType Directory -Force $dir | Out-Null
$url = "https://github.com/floooh/sokol-tools-bin/raw/$commit/bin/win32/sokol-shdc.exe"
Invoke-WebRequest -Uri $url -OutFile (Join-Path $dir 'sokol-shdc.exe')
