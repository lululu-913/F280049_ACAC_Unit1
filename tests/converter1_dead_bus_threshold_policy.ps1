$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$deadBus = [regex]::Match(
    $source,
    '(?s)Uint16 dead_bus\(void\)\s*\{(.*?)\n\}')

if (-not $deadBus.Success) {
    throw 'Unit1 dead-bus policy: dead_bus body not found'
}

$body = $deadBus.Groups[1].Value
foreach ($required in @(
    'meas.uo_rms >= 2.0f',
    'meas.il_rms >= 0.30f',
    'meas.io_rms >= 0.30f',
    'debug_dead_bus_block = 1U;',
    'debug_dead_bus_block = 2U;',
    'debug_dead_bus_block = 3U;',
    'return (count >= 100U) ? 1U : 0U;'
)) {
    if (-not $body.Contains($required)) {
        throw "Unit1 dead-bus policy missing: $required"
    }
}

if (-not $source.Contains('volatile Uint16 debug_dead_bus_limit_mA = 300U;')) {
    throw 'Unit1 dead-bus policy: debugger-visible threshold is not 300 mA'
}

'converter1 dead-bus threshold policy: PASS'
