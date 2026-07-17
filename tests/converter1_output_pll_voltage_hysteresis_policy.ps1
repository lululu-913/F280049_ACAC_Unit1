$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*)$')

if (-not $stateMachine.Success) {
    throw 'Unit1 output-PLL hysteresis policy: slow state machine not found'
}

$stateSource = $stateMachine.Groups[1].Value

$waitBlock = [regex]::Match(
    $stateSource,
    '(?s)case ST_WAIT_UO:(.*?)case ST_OUTPUT_PLL:')

if (-not $waitBlock.Success) {
    throw 'Unit1 output-PLL hysteresis policy: ST_WAIT_UO block not found'
}

$waitBody = $waitBlock.Groups[1].Value
if (-not $waitBody.Contains('meas.uo_rms >= 1.0f')) {
    throw 'Unit1 output-PLL hysteresis policy: WAIT must require Uo >= 1.0 V'
}
if ($waitBody.Contains('meas.uo_rms > 3.5f')) {
    throw 'Unit1 output-PLL hysteresis policy: obsolete 3.5 V entry threshold remains'
}

$outputPllBlock = [regex]::Match(
    $stateSource,
    '(?s)case ST_OUTPUT_PLL:(.*?)case ST_SS5_RAMP:')

if (-not $outputPllBlock.Success) {
    throw 'Unit1 output-PLL hysteresis policy: ST_OUTPUT_PLL block not found'
}

$outputPllBody = $outputPllBlock.Groups[1].Value
foreach ($required in @(
    'if (meas.uo_rms < 0.5f)',
    'run_state = ST_WAIT_UO;',
    'reset_share_signal_processing();',
    '(meas.uo_rms >= 3.0f) && (meas.uo_rms <= qualify_uo_max)'
)) {
    if (-not $outputPllBody.Contains($required)) {
        throw "Unit1 output-PLL hysteresis policy missing: $required"
    }
}

if ($outputPllBody.Contains('meas.uo_rms < 4.0f')) {
    throw 'Unit1 output-PLL hysteresis policy: obsolete 4.0 V return threshold remains'
}

'converter1 output-PLL voltage hysteresis policy: PASS'
