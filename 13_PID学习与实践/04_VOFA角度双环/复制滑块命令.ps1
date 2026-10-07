param(
    [ValidateSet('A', 'AP', 'AI', 'AD', 'P', 'I', 'D', 'V', 'S', 'STOP', 'Z', 'T1')]
    [string]$Command = 'AP'
)

# Only copy a VOFA Str template with a real LF; never open a serial port.
$angleCommandTemplate = if ($Command -in @('STOP', 'Z', 'T1')) { $Command } else { $Command + '%f' }
$angleCommandWithLf = $angleCommandTemplate + [char]10
Set-Clipboard -Value $angleCommandWithLf
Write-Host ('Copied: {0} + LF (0A). Paste into the VOFA Str editor.' -f $angleCommandTemplate)
