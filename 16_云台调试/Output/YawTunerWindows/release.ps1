param([switch]$UseCurrentBuild)
$ErrorActionPreference = 'Stop'
if (-not $UseCurrentBuild) {
    & (Join-Path $PSScriptRoot 'build.ps1') -Test
    if (-not $?) { throw 'Build and tests must pass before packaging.' }
} else {
    $verifiedExe = Get-Item -LiteralPath (Join-Path $PSScriptRoot 'bin\AI\YawTuner.exe')
    $newerSources = @(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'src') -Filter '*.cs' | Where-Object { $_.LastWriteTimeUtc -gt $verifiedExe.LastWriteTimeUtc })
    if ($newerSources.Count -gt 0) { throw 'Sources changed after the current executable; run build.ps1 -Test first.' }
}
$releaseDirectory = Join-Path $PSScriptRoot 'release'
New-Item -ItemType Directory -Force -Path $releaseDirectory | Out-Null
$package = Join-Path $releaseDirectory ('YawTunerWindows_AI_0.2.0_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Path $package | Out-Null
New-Item -ItemType Directory -Path (Join-Path $package 'bin') | Out-Null
New-Item -ItemType Directory -Path (Join-Path $package 'bin\AI') | Out-Null
foreach ($name in @('YawTuner.exe', 'YawTuner.exe.config')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot ('bin\AI\' + $name)) -Destination (Join-Path $package ('bin\AI\' + $name))
}
foreach ($name in @('build.ps1', 'release.ps1', 'YawTuner.exe.config', 'app.manifest', 'start-ai-demo.cmd')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $package $name)
}
Get-ChildItem -LiteralPath $PSScriptRoot -Filter '*.md' -File | ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $package }
foreach ($name in @('src', 'tests')) { Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $package $name) -Recurse }
if (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'preview.png')) { Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'preview.png') -Destination $package }
if (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'ai-preview.png')) { Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'ai-preview.png') -Destination $package }
$hashes = Get-ChildItem -LiteralPath $package -File -Recurse | ForEach-Object { [pscustomobject]@{ Path = $_.FullName.Substring($package.Length + 1); SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash } }
$hashes | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $package 'SHA256.json') -Encoding UTF8
$archive = $package + '.zip'
Compress-Archive -LiteralPath $package -DestinationPath $archive
Write-Host "Package: $archive"
