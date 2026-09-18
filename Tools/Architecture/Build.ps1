param(
    [string]$ArchifyRoot = $env:ARCHIFY_ROOT
)

$ErrorActionPreference = 'Stop'
if (-not $ArchifyRoot) {
    throw '请通过 -ArchifyRoot 指定包含 bin/archify.mjs 的 Archify 技能目录。'
}

$ArchifyRoot = (Resolve-Path -LiteralPath $ArchifyRoot).Path
$lock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'Archify.lock.json') -Raw | ConvertFrom-Json
$package = Get-Content -LiteralPath (Join-Path $ArchifyRoot 'package.json') -Raw | ConvertFrom-Json
if ($package.version -ne $lock.version) {
    throw "请使用 Archify $($lock.version)，工具版本与来源见 Archify.lock.json。"
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$assets = Join-Path $projectRoot 'Assets/Architecture'
$source = Join-Path $PSScriptRoot 'H7_BSP.architecture.json'
$html = Join-Path $assets 'H7_BSP.html'
$svg = Join-Path $assets 'H7_BSP.svg'
$cli = Join-Path $ArchifyRoot 'bin/archify.mjs'
New-Item -ItemType Directory -Force -Path $assets | Out-Null

# deliver 完成结构与布局校验后，才替换已有 HTML。
& node $cli deliver architecture $source $html --quality showcase --json
if ($LASTEXITCODE -ne 0) { throw 'Archify 架构图生成或校验失败。' }

& node (Join-Path $PSScriptRoot 'Export_Svg.mjs') $ArchifyRoot $html $svg
if ($LASTEXITCODE -ne 0) { throw 'Archify SVG 导出失败。' }

Write-Output "架构图已生成: $assets"
