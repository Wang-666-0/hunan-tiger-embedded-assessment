param(
    [ValidateSet('S', 'P', 'I', 'D', 'STOP')]
    [string]$Command = 'P'
)

# Copy a VOFA Str template with a real LF. Never open a serial port.
$commandTemplate = if ($Command -eq 'STOP') { 'STOP' } else { $Command + '%f' }
$commandWithLf = $commandTemplate + [char]10
Set-Clipboard -Value $commandWithLf
Write-Host ('Copied: {0} + LF (0A). Paste into the VOFA Str command editor.' -f $commandTemplate)
