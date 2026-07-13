[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('Select', 'Flash', 'Get')]
    [string]$Action,

    [ValidateSet('DAPLink', 'ST-Link', 'J-Link')]
    [string]$Probe
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$workspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$selectionPath = Join-Path $workspaceRoot '.vscode\selected-flash-probe.json'
$supportedProbes = @('DAPLink', 'ST-Link', 'J-Link')

function Get-SelectedFlashProbe {
    if (Test-Path -LiteralPath $selectionPath) {
        try {
            $selection = Get-Content -LiteralPath $selectionPath -Raw -Encoding UTF8 |
                ConvertFrom-Json -ErrorAction Stop
            $selectedProbe = [string]$selection.probe
            if ($supportedProbes -contains $selectedProbe) {
                return $selectedProbe
            }
        }
        catch {
            Write-Warning 'Unable to read the saved flash probe. Using DAPLink.'
        }
    }

    return 'DAPLink'
}

function Set-SelectedFlashProbe {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('DAPLink', 'ST-Link', 'J-Link')]
        [string]$SelectedProbe
    )

    $selectionDirectory = Split-Path -Parent $selectionPath
    New-Item -ItemType Directory -Path $selectionDirectory -Force | Out-Null

    $selection = [ordered]@{
        probe = $SelectedProbe
        selectedAt = [DateTime]::UtcNow.ToString('o')
    }
    $selectionJson = ($selection | ConvertTo-Json) + [Environment]::NewLine
    $utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($selectionPath, $selectionJson, $utf8WithoutBom)

    Write-Host "[H7_BSP] Flash probe selected: $SelectedProbe"
}

function Assert-RequiredFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "$Description not found: $Path"
    }
}

function Invoke-OpenOcdFlash {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ConfigurationFile,

        [Parameter(Mandatory = $true)]
        [string]$ProbeName
    )

    $openOcdPath = 'C:\DevEnv\DevEnv\OpenOCD\bin\openocd.exe'
    $elfPath = Join-Path $workspaceRoot 'build\Debug\H7_BSP.elf'
    $configurationPath = Join-Path $workspaceRoot $ConfigurationFile

    Assert-RequiredFile -Path $openOcdPath -Description 'OpenOCD'
    Assert-RequiredFile -Path $configurationPath -Description "$ProbeName configuration"
    Assert-RequiredFile -Path $elfPath -Description 'ELF image'

    Write-Host "[H7_BSP] Flashing with ${ProbeName}: $elfPath"
    Push-Location $workspaceRoot
    try {
        & $openOcdPath '-f' $ConfigurationFile '-c' 'program build/Debug/H7_BSP.elf verify reset exit'
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
    }
    finally {
        Pop-Location
    }
}

function Invoke-JLinkFlash {
    $jLinkPath = 'C:\Program Files\SEGGER\JLink_V798a\JLink.exe'
    $elfPath = Join-Path $workspaceRoot 'build\Debug\H7_BSP.elf'
    $jLinkScriptPath = Join-Path $workspaceRoot '.vscode\jlink_flash.jlink'

    Assert-RequiredFile -Path $jLinkPath -Description 'J-Link CLI'
    Assert-RequiredFile -Path $elfPath -Description 'ELF image'

    $jLinkCommands = @(
        'r',
        "loadfile $elfPath",
        'r',
        'g',
        'qc'
    )
    $asciiEncoding = New-Object System.Text.ASCIIEncoding
    [System.IO.File]::WriteAllLines($jLinkScriptPath, $jLinkCommands, $asciiEncoding)

    Write-Host "[H7_BSP] Flashing with J-Link: $elfPath"
    & $jLinkPath '-device' 'STM32H723ZG' '-if' 'SWD' '-speed' '4000' '-autoconnect' '1' '-CommandFile' $jLinkScriptPath
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

switch ($Action) {
    'Select' {
        if ([string]::IsNullOrWhiteSpace($Probe)) {
            throw 'The Select action requires -Probe.'
        }
        Set-SelectedFlashProbe -SelectedProbe $Probe
    }
    'Get' {
        Write-Output (Get-SelectedFlashProbe)
    }
    'Flash' {
        $selectedProbe = Get-SelectedFlashProbe
        switch ($selectedProbe) {
            'DAPLink' {
                Invoke-OpenOcdFlash -ConfigurationFile 'User_Config\daplink.cfg' -ProbeName 'DAPLink / OpenOCD'
            }
            'ST-Link' {
                Invoke-OpenOcdFlash -ConfigurationFile 'User_Config\stlink.cfg' -ProbeName 'ST-Link / OpenOCD'
            }
            'J-Link' {
                Invoke-JLinkFlash
            }
            default {
                throw "Unsupported flash probe: $selectedProbe"
            }
        }
    }
}
