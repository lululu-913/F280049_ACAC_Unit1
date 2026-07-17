$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*?)void keys_update_5ms').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($stateMachine)) {
    throw 'In-phase policy: slow state machine body not found'
}

$outputPll = [regex]::Match(
    $stateMachine,
    '(?s)case ST_OUTPUT_PLL:(.*?)(?=case ST_SS5_RAMP:)').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($outputPll)) {
    throw 'In-phase policy: ST_OUTPUT_PLL body not found'
}

if ([regex]::Matches(
        $outputPll,
        'fabsf\(wrap_pi\(pll_uo\.theta - pll_ui\.theta\)\)').Count -ne 1) {
    throw 'In-phase policy: qualification must use theta_o-theta_i exactly once'
}

if ($outputPll -match 'pll_uo\.theta - pll_ui\.theta - PI_F') {
    throw 'In-phase policy: old 180-degree phase contract remains'
}

if ($outputPll -notmatch '20\.0f \* PI_F / 180\.0f') {
    throw 'In-phase policy: the relaxed 20-degree qualification window is missing'
}

if ($outputPll -match 'latch_fault\(FAULT_START\)|latch_fault\(FAULT_PHASE\)') {
    throw 'In-phase policy: blocked OPLL waiting must not latch F10 or F13'
}

if ($source -notmatch '#define IL_TO_OUTPUT_SIGN\s+\(1\.0f\)') {
    throw 'In-phase policy: retain the hardware-validated positive current direction'
}

'converter1 in-phase contract policy: PASS'
