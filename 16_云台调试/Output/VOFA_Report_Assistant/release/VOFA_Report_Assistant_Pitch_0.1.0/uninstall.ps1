[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'Low')]
param(
    [string]$VofaRoot = 'D:\VOFA+\x64'
)

# Archive the one recognized widget directory; never delete host state or history.
$ErrorActionPreference = 'Stop'
$packageId = 'vofa.report-assistant.pitch'
function Full-Path([string]$Path) {
    if (-not [IO.Path]::IsPathRooted($Path)) { throw "Absolute path required: $Path" }
    return [IO.Path]::GetFullPath($Path).TrimEnd('\', '/')
}
function Assert-PlainDirectory([string]$Path) {
    if (Test-Path -LiteralPath $Path) {
        $item = Get-Item -LiteralPath $Path -Force
        if (-not $item.PSIsContainer -or ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw "Not an ordinary directory (links are refused): $Path"
        }
    }
}
function Assert-PlainTree([string]$Root) {
    $queue = New-Object 'System.Collections.Generic.Queue[string]'
    $queue.Enqueue($Root)
    while ($queue.Count -gt 0) {
        foreach ($item in Get-ChildItem -LiteralPath $queue.Dequeue() -Force) {
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Links are refused: $($item.FullName)" }
            if ($item.PSIsContainer) { $queue.Enqueue($item.FullName) }
        }
    }
}

$hostRoot = Full-Path $VofaRoot
$pluginsRoot = Full-Path (Join-Path $hostRoot 'plugins')
$widgetsRoot = Full-Path (Join-Path $pluginsRoot 'widgets')
$destination = Full-Path (Join-Path $widgetsRoot 'report_assistant')
if (-not $destination.Equals(($hostRoot + '\plugins\widgets\report_assistant'), [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Destination must be the exact plugins/widgets/report_assistant directory.'
}
foreach ($path in @($hostRoot, $pluginsRoot, $widgetsRoot, $destination)) { Assert-PlainDirectory $path }
if (-not (Test-Path -LiteralPath $destination -PathType Container)) { Write-Host 'report_assistant is not installed.'; return }
Assert-PlainTree $destination
$markerPath = Join-Path $destination '.report-assistant-package.json'
$manifestPath = Join-Path $destination '.report-assistant-install.json'
if (-not (Test-Path -LiteralPath $markerPath -PathType Leaf) -or -not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
    throw "Existing directory is not recognized; no files moved: $destination"
}
$marker = Get-Content -LiteralPath $markerPath -Raw -Encoding UTF8 | ConvertFrom-Json
$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($marker.packageId -ne $packageId -or $manifest.packageId -ne $packageId -or $manifest.schemaVersion -ne 1) {
    throw 'Ownership marker/manifest mismatch; no files moved.'
}
$seen = @{}
foreach ($entry in @($manifest.files)) {
    $relative = [string]$entry.path
    if ([string]::IsNullOrWhiteSpace($relative) -or [IO.Path]::IsPathRooted($relative) -or
        $relative.Contains(':') -or $relative -match '(^|[\\/])\.\.([\\/]|$)') { throw "Invalid manifest path: $relative" }
    $file = Full-Path (Join-Path $destination $relative)
    if (-not $file.StartsWith($destination + '\', [StringComparison]::OrdinalIgnoreCase) -or $seen.ContainsKey($file)) {
        throw "Invalid/duplicate manifest target: $file"
    }
    $seen[$file] = $true
    if (-not (Test-Path -LiteralPath $file -PathType Leaf) -or
        (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -ne $entry.sha256) {
        throw "Installed file was changed or removed; manual inspection required, no files moved: $file"
    }
}
if (-not $seen.ContainsKey($markerPath)) { throw 'Ownership marker missing from manifest.' }
$backupRoot = Full-Path (Join-Path $PSScriptRoot 'backups')
Assert-PlainDirectory $backupRoot
$backup = Full-Path (Join-Path $backupRoot ('uninstalled_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0, 8)))
if (-not $backup.StartsWith($backupRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid backup path.' }
Write-Host "Only VOFA directory to archive: $destination"
Write-Host "Archive destination: $backup\report_assistant"
Write-Host 'Plugin history and extra plugin-local files will be retained in that archive. No host settings will change.'
if (-not $PSCmdlet.ShouldProcess($destination, 'Move recognized report_assistant directory to package backup')) { return }
$null = New-Item -ItemType Directory -Path $backup -Force
Move-Item -LiteralPath $destination -Destination (Join-Path $backup 'report_assistant')
Write-Host "Uninstalled by archiving. User history remains in: $backup\report_assistant"
