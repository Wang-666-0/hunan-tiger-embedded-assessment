param(
    [Parameter(Mandatory=$true)][string]$Scenario,
    [string[]]$IncludePaths = @(),
    [string[]]$ExcludePaths = @(),
    [string[]]$IncludeTabs = @(),
    [int]$Seconds = 25
)
$ErrorActionPreference = 'Stop'
$repairRoot = (Get-Content -LiteralPath (Join-Path $PSScriptRoot 'VOFA_Report_Assistant\validation\active-repair-path.txt') -Raw).Trim()
if ($Scenario -notmatch '^[a-z0-9-]+$') { throw 'Invalid scenario.' }
$current = [IO.Path]::GetFullPath('C:\Users\Administrator\AppData\Local\vofa+\100\context')
$held = [IO.Path]::GetFullPath((Join-Path $repairRoot ($Scenario + '-held')))
$completed = [IO.Path]::GetFullPath((Join-Path $repairRoot ($Scenario + '-completed')))
if ($current -ne 'C:\Users\Administrator\AppData\Local\vofa+\100\context' -or -not $held.StartsWith($repairRoot + '\') -or -not $completed.StartsWith($repairRoot + '\')) { throw 'Invalid test paths.' }
if (Get-Process -Name 'vofa+' -ErrorAction SilentlyContinue) { throw 'VOFA already running.' }
if ((Get-Item -LiteralPath $current).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Refusing context link.' }
if ((Test-Path -LiteralPath $held) -or (Test-Path -LiteralPath $completed)) { throw 'Test destinations already exist.' }
$p = $null
Move-Item -LiteralPath $current -Destination $held
try {
    $null = New-Item -ItemType Directory -Path $current
    Copy-Item -LiteralPath (Join-Path $held 'vofa+.config.json') -Destination $current
    Copy-Item -LiteralPath (Join-Path $held 'vofa+.cmds.json') -Destination $current
    $tabs = Get-Content -LiteralPath (Join-Path $held 'vofa+.tabviews.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($group in $tabs.ctx) {
        foreach ($tab in $group.tabs) {
            if ($IncludeTabs.Count -and $tab.name -notin $IncludeTabs) { $tab.widgets = @(); continue }
            $tab.widgets = @($tab.widgets | Where-Object { (-not $IncludePaths.Count -or $_.path -in $IncludePaths) -and $_.path -notin $ExcludePaths })
        }
    }
    $json = $tabs | ConvertTo-Json -Depth 100 -Compress
    [IO.File]::WriteAllText((Join-Path $current 'vofa+.tabviews.json'), $json, (New-Object Text.UTF8Encoding($false)))
    $env:QT_FORCE_STDERR_LOGGING = '1'
    $p = Start-Process -FilePath 'D:\VOFA+\x64\vofa+.exe' -WorkingDirectory 'D:\VOFA+\x64' -PassThru -RedirectStandardError (Join-Path $repairRoot ($Scenario + '-stderr.log')) -RedirectStandardOutput (Join-Path $repairRoot ($Scenario + '-stdout.log'))
    Start-Sleep -Seconds $Seconds
    $p.Refresh()
    $result = [pscustomobject]@{ scenario=$Scenario; exited=$p.HasExited; responding=$(if(-not $p.HasExited){$p.Responding}else{$false}); memoryBytes=$(if(-not $p.HasExited){$p.WorkingSet64}else{0}); elapsedSeconds=$Seconds }
    $result | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $repairRoot ($Scenario + '-result.json')) -Encoding UTF8
} finally {
    if ($p) { $p.Refresh(); if (-not $p.HasExited) { if ($p.Path -ne 'D:\VOFA+\x64\vofa+.exe') { throw 'Unexpected diagnostic process path.' }; Stop-Process -Id $p.Id -Force; $p.WaitForExit(5000) | Out-Null } }
    if (Test-Path -LiteralPath $current) { Move-Item -LiteralPath $current -Destination $completed }
    Move-Item -LiteralPath $held -Destination $current
}
$result | ConvertTo-Json
if ((Get-FileHash -LiteralPath (Join-Path $current 'vofa+.tabviews.json')).Hash -ne (Get-FileHash -LiteralPath (Join-Path $repairRoot 'backup\vofa-profile\100\context\vofa+.tabviews.json')).Hash) { throw 'Original layout was not restored.' }
