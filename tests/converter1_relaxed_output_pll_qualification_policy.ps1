$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

if (-not $source.Contains('Uint16 output_pll_stable_ms = 0U;')) {
    throw 'Unit1 output-PLL qualification: debugger-visible stable counter missing'
}

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*)$')

if (-not $stateMachine.Success) {
    throw 'Unit1 output-PLL qualification: slow state machine not found'
}

$outputPllBlock = [regex]::Match(
    $stateMachine.Groups[1].Value,
    '(?s)case ST_OUTPUT_PLL:(.*?)case ST_SS5_RAMP:')

if (-not $outputPllBlock.Success) {
    throw 'Unit1 output-PLL qualification: ST_OUTPUT_PLL block not found'
}

$body = $outputPllBlock.Groups[1].Value
foreach ($required in @(
    'float qualify_uo_max = effective_u_ref() + 2.0f;',
    '(meas.uo_rms >= 3.0f) && (meas.uo_rms <= qualify_uo_max)',
    '(pll_uo.omega >= TWO_PI_F * 48.0f)',
    '(pll_uo.omega <= TWO_PI_F * 52.0f)',
    '(20.0f * PI_F / 180.0f)',
    'if (output_pll_stable_ms < 50U) output_pll_stable_ms++;',
    'if (output_pll_stable_ms >= 50U)',
    'else output_pll_stable_ms = 0U;'
)) {
    if (-not $body.Contains($required)) {
        throw "Unit1 output-PLL qualification missing: $required"
    }
}

foreach ($obsolete in @(
    'state_ms > 1000UL',
    'latch_fault(FAULT_START);',
    '(meas.uo_rms >= 4.5f) && (meas.uo_rms <= 5.5f)',
    '(10.0f * PI_F / 180.0f)'
)) {
    if ($body.Contains($obsolete)) {
        throw "Unit1 relaxed output-PLL qualification: obsolete rule remains: $obsolete"
    }
}

'converter1 relaxed output-PLL qualification policy: PASS'
