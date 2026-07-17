$ErrorActionPreference = 'Stop'

$path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'
$source = Get-Content -Raw -LiteralPath $path

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*)$')

if (-not $stateMachine.Success) {
    throw 'Unit2 fast SS5/HOLD policy: state machine not found'
}

$stateSource = $stateMachine.Groups[1].Value

$ss5 = [regex]::Match(
    $stateSource,
    '(?s)case ST_SS5_RAMP:(.*?)case ST_SS5_HOLD:')
$hold = [regex]::Match(
    $stateSource,
    '(?s)case ST_SS5_HOLD:(.*?)case ST_VOLT_RAMP:')

if (-not $ss5.Success -or -not $hold.Success) {
    throw 'Unit2 fast SS5/HOLD policy: state blocks not found'
}

if (-not $ss5.Groups[1].Value.Contains(
    'u_ref_active = slew(u_ref_active, 5.0f, 0.150f);')) {
    throw 'Unit2 fast SS5/HOLD policy: 5 V ramp is not 0.150 V/ms'
}

foreach ($required in @(
    'if (ss5_stable_ms < 300U) ss5_stable_ms++;',
    'if (ss5_stable_ms >= 300U)'
)) {
    if (-not $hold.Groups[1].Value.Contains($required)) {
        throw "Unit2 fast SS5/HOLD policy missing: $required"
    }
}

foreach ($obsolete in @(
    'ss5_stable_ms < 125U',
    'ss5_stable_ms >= 125U',
    'state_ms > 1500UL',
    'latch_fault(FAULT_START)'
)) {
    if ($hold.Groups[1].Value.Contains($obsolete)) {
        throw "Unit2 fast SS5/HOLD policy: obsolete HOLD rule remains: $obsolete"
    }
}

if (-not $stateSource.Contains('ramp_span / 500.0f')) {
    throw 'Unit2 fast SS5/HOLD policy: 5 V to target ramp is not approximately 500 ms'
}

'converter2 fast SS5/HOLD policy: PASS'
