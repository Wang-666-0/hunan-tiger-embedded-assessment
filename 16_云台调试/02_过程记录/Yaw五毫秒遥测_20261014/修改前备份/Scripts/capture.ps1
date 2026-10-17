<#
通过已有 USART1 USB 转串口读取参数，不烧录，不改电调配置，不发送运动指令。
示例：.\Scripts\capture.ps1 -List
      .\Scripts\capture.ps1 -Port COM5
没有指定 Port 时，列出串口并让用户输入。
#>
param(
    [string]$Port = '',
    [string]$OutputDirectory = '',
    [string]$PythonPath = '',
    [switch]$List
)
$ErrorActionPreference = 'Stop'
# Windows PowerShell 5 and Python otherwise may disagree about Chinese text encoding.
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$collector = Join-Path $projectRoot 'Tools\motor_capture.py'
$localDependencies = Join-Path $projectRoot 'Tools\.deps'

# 使用明确找到的 Python；不自动安装依赖或打开 VOFA。
$pythonCandidates = @()
if ($PythonPath) {
    $pythonCandidates += $PythonPath
} else {
    foreach ($commandName in @('python', 'python3', 'py')) {
        $command = Get-Command $commandName -ErrorAction SilentlyContinue
        if ($command) {
            try { $resolvedPython = & $command.Source -X utf8 -c 'import sys; assert sys.version_info >= (3, 10); print(sys.executable)' 2>$null }
            catch { continue }
            if ($LASTEXITCODE -eq 0 -and $resolvedPython) {
                $pythonCandidates += [string]($resolvedPython | Select-Object -Last 1)
            }
        }
    }
    $pythonCandidates += 'D:\Codex\Cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
}
$selectedPython = ''
$fallbackPython = ''
foreach ($candidate in ($pythonCandidates | Select-Object -Unique)) {
    if (!(Test-Path -LiteralPath $candidate -PathType Leaf)) { continue }
    try {
        & $candidate -X utf8 -c 'import sys; assert sys.version_info >= (3, 10)' 2>$null
        if ($LASTEXITCODE -ne 0) { continue }
    } catch { continue }
    if (!$fallbackPython) { $fallbackPython = $candidate }
    try {
        & $candidate -X utf8 -c 'import sys; assert sys.version_info >= (3, 10); sys.path.insert(0, sys.argv[1]); import serial' $localDependencies 2>$null
        if ($LASTEXITCODE -eq 0) { $selectedPython = $candidate; break }
    } catch { continue }
}
if (!$selectedPython) {
    if (!$fallbackPython) {
        throw '未找到 Python 3.10 或更新版本，请安装或用 -PythonPath 指定 python.exe。'
    }
    Write-Host '缺少 pyserial。请先执行下面这一条命令，完成后重新运行采集：'
    Write-Host ('& "' + $fallbackPython + '" -m pip install --target "' + $localDependencies + '" pyserial==3.5')
    exit 2
}
$arguments = @($collector)
if ($List) {
    $arguments += '--list'
} else {
    if ($Port) { $arguments += @('--port', $Port) }
    if ($OutputDirectory) { $arguments += @('--output', $OutputDirectory) }
}
& $selectedPython -X utf8 @arguments
exit $LASTEXITCODE
