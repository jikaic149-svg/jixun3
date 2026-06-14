param(
    [Parameter(Mandatory = $true)]
    [string]$Action
)

$ErrorActionPreference = "Stop"

$make = "D:/MAKE/make.exe"
$openocd = "D:/OPENOCD/xpack-openocd-0.12.0-7/bin/openocd.exe"
$elf = "build/spi_bmi088.elf"
$openocdArgs = @(
    "-f", "interface/stlink.cfg",
    "-f", "target/stm32f4x.cfg",
    "-c", "adapter speed 1000"
)

function Invoke-Make {
    param([string[]]$ArgsList = @())
    New-Item -ItemType Directory -Force -Path "build" | Out-Null
    & $make @ArgsList
    if ($LASTEXITCODE -ne 0) {
        throw "make failed with exit code $LASTEXITCODE"
    }
}

function Remove-BuildDirectory {
    Remove-Item -Recurse -Force "build" -ErrorAction SilentlyContinue
}

function Invoke-OpenOCD {
    param([string[]]$ExtraArgs = @())
    & $openocd @openocdArgs @ExtraArgs
    if ($LASTEXITCODE -ne 0) {
        throw "openocd failed with exit code $LASTEXITCODE"
    }
}

switch ($Action) {
    "STM32: build" {
        Invoke-Make
    }
    "STM32: clean" {
        Remove-BuildDirectory
    }
    "STM32: rebuild" {
        Remove-BuildDirectory
        Invoke-Make
    }
    "STM32: flash" {
        if (-not (Test-Path $elf)) {
            Invoke-Make
        }
        Invoke-OpenOCD @("-c", "program $elf verify reset exit")
    }
    "STM32: build and flash" {
        Invoke-Make
        Invoke-OpenOCD @("-c", "program $elf verify reset exit")
    }
    "STM32: openocd server" {
        Invoke-OpenOCD
    }
    default {
        throw "Unknown STM32 action: $Action"
    }
}
