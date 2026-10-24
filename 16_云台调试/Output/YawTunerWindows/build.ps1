param([switch]$Test)
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$compiler = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (-not (Test-Path -LiteralPath $compiler)) { $compiler = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe' }
if (-not (Test-Path -LiteralPath $compiler)) { throw '.NET Framework compiler not found.' }
$outputDirectory = Join-Path $taskRoot 'bin\AI'
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$sources = @(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'src') -Filter '*.cs' | ForEach-Object { $_.FullName })
$application = Join-Path $outputDirectory 'YawTuner.exe'
& $compiler /nologo /target:winexe /platform:anycpu /optimize+ /warn:4 /langversion:5 "/out:$application" "/win32manifest:$(Join-Path $taskRoot 'app.manifest')" /reference:System.dll /reference:System.Core.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll /reference:System.Web.Extensions.dll /reference:System.Net.Http.dll /reference:System.Security.dll $sources
if ($LASTEXITCODE -ne 0) { throw 'Application compilation failed.' }
Copy-Item -LiteralPath (Join-Path $taskRoot 'YawTuner.exe.config') -Destination (Join-Path $outputDirectory 'YawTuner.exe.config') -Force
if ($Test) {
    $tests = @(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'tests') -Filter '*.cs' | ForEach-Object { $_.FullName })
    $testExe = Join-Path $outputDirectory 'YawTuner.Tests.exe'
    & $compiler /nologo /target:exe /platform:anycpu /optimize+ /warn:4 /langversion:5 "/out:$testExe" /main:YawTuner.Tests.TestRunner /reference:System.dll /reference:System.Core.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll /reference:System.Web.Extensions.dll /reference:System.Net.Http.dll /reference:System.Security.dll $sources $tests
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Offline tests failed.' }
}
Write-Host "Built: $application"
