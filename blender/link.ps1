$src = $PSScriptRoot
$dst = "$env:APPDATA\Blender Foundation\Blender\4.5\extensions\user_default\hinacloth"

New-Item -ItemType Directory -Force -Path (Split-Path $dst) | Out-Null
if (Test-Path $dst) { Remove-Item $dst -Force -Recurse }
New-Item -ItemType SymbolicLink -Path $dst -Target $src
