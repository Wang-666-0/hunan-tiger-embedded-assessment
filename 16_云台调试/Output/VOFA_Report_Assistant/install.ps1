[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'Low')]
param(
    [string]$VofaRoot = 'D:\VOFA+\x64'
)

# Only this widget's new directory is writable. No host configuration is edited.
$ErrorActionPreference = 'Stop'
$packageId = 'vofa.report-assistant.pitch'
$markerName = '.report-assistant-package.json'
$manifestName = '.report-assistant-install.json'

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
function Safe-Files([string]$Root) {
    Assert-PlainDirectory $Root
    $queue = New-Object 'System.Collections.Generic.Queue[string]'
    $queue.Enqueue($Root)
    while ($queue.Count -gt 0) {
        foreach ($item in Get-ChildItem -LiteralPath $queue.Dequeue() -Force) {
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Links are refused: $($item.FullName)" }
            if ($item.PSIsContainer) { $queue.Enqueue($item.FullName) } else { $item }
        }
    }
}
function Owned-File([string]$Root, [string]$Relative) {
    if ([string]::IsNullOrWhiteSpace($Relative) -or [IO.Path]::IsPathRooted($Relative) -or
        $Relative.Contains(':') -or $Relative -match '(^|[\\/])\.\.([\\/]|$)') {
        throw "Invalid manifest path: $Relative"
    }
    $candidate = Full-Path (Join-Path $Root $Relative)
    if (-not $candidate.StartsWith($Root + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Manifest path escaped plugin directory: $Relative"
    }
    return $candidate
}
function Read-Installed([string]$Root) {
    $markerPath = Join-Path $Root $markerName
    $manifestPath = Join-Path $Root $manifestName
    if (-not (Test-Path -LiteralPath $markerPath -PathType Leaf) -or
        -not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
        throw "Existing directory is not a recognized installed report_assistant; no files changed: $Root"
    }
    $marker = Get-Content -LiteralPath $markerPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($marker.packageId -ne $packageId -or $manifest.packageId -ne $packageId -or $manifest.schemaVersion -ne 1) {
        throw "Plugin ownership marker/manifest mismatch: $Root"
    }
    $seen = @{}
    foreach ($entry in @($manifest.files)) {
        $path = Owned-File $Root ([string]$entry.path)
        if ($seen.ContainsKey($path)) { throw "Duplicate manifest entry: $path" }
        $seen[$path] = $true
        if (-not (Test-Path -LiteralPath $path -PathType Leaf) -or
            (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $entry.sha256) {
            throw "Installed plugin file has been changed or removed; refusing to overwrite it: $path"
        }
    }
    if (-not $seen.ContainsKey((Join-Path $Root $markerName))) { throw 'Ownership marker missing from manifest.' }
    return $manifest
}

$hostRoot = Full-Path $VofaRoot
if (-not (Test-Path -LiteralPath $hostRoot -PathType Container)) { throw "VOFA directory does not exist: $hostRoot" }
$pluginsRoot = Full-Path (Join-Path $hostRoot 'plugins')
$widgetsRoot = Full-Path (Join-Path $pluginsRoot 'widgets')
$destination = Full-Path (Join-Path $widgetsRoot 'report_assistant')
if (-not $destination.Equals(($hostRoot + '\plugins\widgets\report_assistant'), [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Destination must be the exact plugins/widgets/report_assistant directory.'
}
foreach ($path in @($hostRoot, $pluginsRoot, $widgetsRoot, $destination)) { Assert-PlainDirectory $path }
if (-not (Test-Path -LiteralPath $widgetsRoot -PathType Container)) { throw "VOFA widgets directory not found: $widgetsRoot" }

$source = Full-Path (Join-Path $PSScriptRoot 'plugins\widgets\report_assistant')
if ($destination.Equals($source, [StringComparison]::OrdinalIgnoreCase)) { throw 'Source and destination must differ.' }
if (-not (Test-Path -LiteralPath $source -PathType Container)) { throw "Package payload not found: $source" }
$sourceFiles = @(Safe-Files $source)
$sourceMarker = Get-Content -LiteralPath (Join-Path $source $markerName) -Raw -Encoding UTF8 | ConvertFrom-Json
if ($sourceMarker.packageId -ne $packageId) { throw 'Source package marker is not recognized.' }
if (Test-Path -LiteralPath (Join-Path $source $manifestName)) { throw 'Source must not include an installed manifest.' }

$oldManifest = $null
if (Test-Path -LiteralPath $destination) {
    $null = @(Safe-Files $destination)
    $oldManifest = Read-Installed $destination
}
$oldOwned = @{}
if ($null -ne $oldManifest) {
    foreach ($entry in @($oldManifest.files)) { $oldOwned[(Owned-File $destination ([string]$entry.path))] = $true }
}
$newEntries = @()
foreach ($file in $sourceFiles) {
    $relative = $file.FullName.Substring($source.Length + 1).Replace('\', '/')
    $targetFile = Owned-File $destination $relative
    if ((Test-Path -LiteralPath $targetFile) -and -not $oldOwned.ContainsKey($targetFile)) {
        throw "A file not owned by the old plugin would be overwritten; no files changed: $targetFile"
    }
    $newEntries += [pscustomobject]@{ path = $relative; sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash }
}

Write-Host "Payload: $source"
Write-Host "Only VOFA destination: $destination"
Write-Host 'No layout, channel, command, serial, key, context or tabview settings will be written.'
if (-not $PSCmdlet.ShouldProcess($destination, 'Install report_assistant widget files only')) { return }

$backup = $null
if ($null -ne $oldManifest) {
    $backupRoot = Full-Path (Join-Path $PSScriptRoot 'backups')
    Assert-PlainDirectory $backupRoot
    $backup = Full-Path (Join-Path $backupRoot ('upgrade_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0, 8)))
    if (-not $backup.StartsWith($backupRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid backup path.' }
    $null = New-Item -ItemType Directory -Path $backup -Force
    Copy-Item -LiteralPath $destination -Destination (Join-Path $backup 'report_assistant') -Recurse
    Write-Host "Previous plugin only backed up to: $backup"
}
if (-not (Test-Path -LiteralPath $destination)) { $null = New-Item -ItemType Directory -Path $destination }
foreach ($file in $sourceFiles) {
    $relative = $file.FullName.Substring($source.Length + 1)
    $targetFile = Owned-File $destination $relative
    $parent = [IO.Path]::GetDirectoryName($targetFile)
    if (-not (Test-Path -LiteralPath $parent)) { $null = New-Item -ItemType Directory -Path $parent -Force }
    Copy-Item -LiteralPath $file.FullName -Destination $targetFile -Force
}
# Remove only previously owned, unmodified obsolete files; never delete a directory.
$newOwned = @{}
foreach ($entry in $newEntries) { $newOwned[(Owned-File $destination $entry.path)] = $true }
if ($null -ne $oldManifest) {
    foreach ($entry in @($oldManifest.files)) {
        $path = Owned-File $destination ([string]$entry.path)
        if (-not $newOwned.ContainsKey($path)) { Remove-Item -LiteralPath $path }
    }
}
$manifest = [pscustomobject]@{
    schemaVersion = 1; packageId = $packageId; version = $sourceMarker.version
    installedAt = [DateTimeOffset]::Now.ToString('o'); files = @($newEntries)
}
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $destination $manifestName) -Encoding UTF8
$null = Read-Installed $destination
Write-Host "Installed $($sourceMarker.version). VOFA was not launched or reconnected."
