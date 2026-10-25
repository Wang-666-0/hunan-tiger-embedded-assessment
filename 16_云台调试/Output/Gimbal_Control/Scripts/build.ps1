<#
从持久的 Keil 工程取得源文件列表，生成临时 EIDE builder 参数并完整构建。
只编译：不打开串口、不连接 ST-Link、不烧录。VS Code 中也可直接打开 MDK-ARM 工作区构建。
#>
param(
    [string]$ToolchainRoot = (Join-Path $env:USERPROFILE '.eide\tools\armcc_v5_cracked'),
    [string]$BuilderPath = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$mdkRoot = Join-Path $projectRoot 'MDK-ARM'
$projectName = 'Gimbal_Control'
[xml]$keilProject = Get-Content -LiteralPath (Join-Path $mdkRoot ($projectName + '.uvprojx')) -Raw
$target = $keilProject.Project.Targets.Target
$sources = @($target.Groups.Group.Files.File |
    Where-Object { [int]$_.FileType -in 1, 2 } |
    ForEach-Object { [string]$_.FilePath })
$includes = @(([string]$target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath).Split(';'))
if ($sources.Count -lt 40) { throw '工程源文件列表不完整，请检查 App 分组。' }
foreach ($source in $sources) {
    if (!(Test-Path -LiteralPath (Join-Path $mdkRoot $source))) { throw "源文件不存在：$source" }
}
if (!$BuilderPath) {
    $extensionRoot = Join-Path $env:USERPROFILE '.vscode\extensions'
    $candidates = Get-ChildItem -LiteralPath $extensionRoot -Directory -Filter 'cl.eide-*' |
        Sort-Object LastWriteTime -Descending
    foreach ($candidate in $candidates) {
        $path = Join-Path $candidate.FullName 'res\tools\win32\unify_builder\unify_builder.exe'
        if (Test-Path -LiteralPath $path) { $BuilderPath = $path; break }
    }
}
if (!(Test-Path -LiteralPath $BuilderPath)) { throw '未找到 EIDE unify_builder.exe，可通过 -BuilderPath 指定。' }
if (!(Test-Path -LiteralPath (Join-Path $ToolchainRoot 'bin\armcc.exe'))) { throw '未找到 ARM Compiler 5，可通过 -ToolchainRoot 指定。' }
$extensionPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path $BuilderPath -Parent) '..\..\..\..')).Path
$output = Join-Path $mdkRoot ('build\' + $projectName)
New-Item -ItemType Directory -Path $output -Force | Out-Null
$scatterPath = Join-Path $output ($projectName + '.sct')
# 链接布局：Flash 1 MB，SRAM1 112 KB + SRAM2 16 KB；不把 CCM 用作 UART 缓冲。
$scatter = @'
LR_IROM1 0x08000000 0x00100000 {
  ER_IROM1 0x08000000 0x00100000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x0001C000 {
    .ANY (+RW +ZI)
  }
  RW_IRAM2 0x2001C000 0x00004000 {
    .ANY (+RW +ZI)
  }
}
'@
[System.IO.File]::WriteAllText($scatterPath, $scatter, [System.Text.Encoding]::ASCII)
$options = @{
    version = 4
    afterBuildTasks = @(); beforeBuildTasks = @()
    'asm-compiler' = @{ defines = @() }
    'c/cpp-compiler' = @{
        CXX_FLAGS = '--diag_suppress=1 --diag_suppress=1295'
        C_FLAGS = '--diag_suppress=1 --diag_suppress=1295'
        'c99-mode' = $true; 'one-elf-section-per-function' = $true
        optimization = 'level-0'; warnings = 'all-warnings'
    }
    global = @{
        'output-debug-info' = 'enable'; 'use-microLIB' = $false
        'microcontroller-cpu' = 'cortex-m4-sp'
        'microcontroller-fpu' = 'cortex-m4-sp'
        'microcontroller-float' = 'cortex-m4-sp'
        '$arch-extensions' = ''; '$clang-arch-extensions' = ''; '$armlink-arch-extensions' = ''
    }
    linker = @{
        '$outputTaskExcludes' = @('.bin'); 'output-format' = 'elf'
        'ro-base' = ''; 'rw-base' = ''; 'xo-base' = ''
        'link-scatter' = @($scatterPath.Replace('\', '/'))
    }
}
$parameters = @{
    name = $projectName; target = $projectName; toolchain = 'AC5'
    toolchainLocation = $ToolchainRoot
    toolchainCfgFile = (Join-Path $extensionPath 'res\data\models\arm.v5.model.json')
    buildMode = 'fast|multhread'; showRepathOnLog = $true; threadNum = 8
    rootDir = $mdkRoot; dumpPath = ('build\' + $projectName); outDir = ('build\' + $projectName)
    ram = 131072; rom = 1048576
    incDirs = $includes; libDirs = @(); defines = @('USE_HAL_DRIVER', 'STM32F405xx')
    sourceList = $sources; alwaysInBuildSources = @(); sourceParams = @{}
    options = $options; sysPaths = @()
    env = @{
        ProjectName = $projectName; ConfigName = $projectName; ProjectRoot = $mdkRoot
        workspaceFolder = $mdkRoot; OutDir = $output; OutDirRoot = 'build'
        OutDirBase = ('build\' + $projectName); ToolchainRoot = $ToolchainRoot
        ExecutableName = (Join-Path $output $projectName); KEIL_OUTPUT_DIR = $projectName
    }
}
$paramsPath = Join-Path $output 'builder.params'
[System.IO.File]::WriteAllText($paramsPath, ($parameters | ConvertTo-Json -Depth 12), (New-Object System.Text.UTF8Encoding($false)))
$previousRollForward = $env:DOTNET_ROLL_FORWARD
try {
    $env:DOTNET_ROLL_FORWARD = 'Major' # 允许已有 .NET 7/8 运行 EIDE 的 .NET 6 构建器。
    Push-Location -LiteralPath $mdkRoot
    & $BuilderPath -p $paramsPath --rebuild
    if ($LASTEXITCODE -ne 0) { throw "构建失败，退出码 $LASTEXITCODE" }
    Write-Host ('HEX: ' + (Join-Path $output ($projectName + '.hex')))
}
finally {
    Pop-Location
    $env:DOTNET_ROLL_FORWARD = $previousRollForward
}
