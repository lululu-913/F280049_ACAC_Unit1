$ErrorActionPreference = 'Stop'

$path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'
$source = Get-Content -Raw -LiteralPath $path
$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*)$')

if (-not $stateMachine.Success) {
    throw 'Unit2 relaxed voltage ramp policy: state machine not found'
}

$stateSource = $stateMachine.Groups[1].Value
$ss5 = [regex]::Match(
    $stateSource,
    '(?s)case ST_SS5_RAMP:(.*?)case ST_SS5_HOLD:')
$ramp = [regex]::Match(
    $stateSource,
    '(?s)case ST_VOLT_RAMP:(.*?)case ST_SHARE_RAMP:')
$run = [regex]::Match(
    $stateSource,
    '(?s)case ST_RUN:(.*?)case ST_STOP_RAMP:')

if (-not $ss5.Success -or -not $ramp.Success -or -not $run.Success) {
    throw 'Unit2 relaxed voltage ramp policy: state blocks not found'
}

foreach ($required in @(
    'u_ref_active >= 4.80f',
    'if (ss5_stable_ms < 30U) ss5_stable_ms++;',
    'if (ss5_stable_ms >= 30U)'
)) {
    if (-not $ss5.Groups[1].Value.Contains($required)) {
        throw "Unit2 relaxed voltage ramp policy missing SS5 rule: $required"
    }
}

foreach ($required in @(
    'fabsf(u_ref_active - target) < 0.05f',
    'fabsf(meas.uo_rms - target) <= 3.0f',
    'current_limit_active == 0U',
    'if (voltage_stable_ms < 100U) voltage_stable_ms++;',
    'if (voltage_stable_ms >= 100U)'
)) {
    if (-not $ramp.Groups[1].Value.Contains($required)) {
        throw "Unit2 relaxed voltage ramp policy missing ramp rule: $required"
    }
}

foreach ($block in @(
    @{ Name = 'SS5'; Body = $ss5.Groups[1].Value },
    @{ Name = 'RAMP'; Body = $ramp.Groups[1].Value }
)) {
    foreach ($obsolete in @(
        'latch_fault(FAULT_START)',
        'state_ms > 1500UL',
        'state_ms > 2000UL'
    )) {
        if ($block.Body.Contains($obsolete)) {
            throw "Unit2 relaxed voltage ramp policy: $($block.Name) timeout remains: $obsolete"
        }
    }
}

foreach ($required in @(
    'fmaxf(2.0f, 0.15f * u_ref_active)',
    'if (voltage_error_ms < 5000U) voltage_error_ms++;',
    'if (voltage_error_ms >= 5000U) latch_fault(FAULT_START);'
)) {
    if (-not $run.Groups[1].Value.Contains($required)) {
        throw "Unit2 relaxed voltage ramp policy missing RUN rule: $required"
    }
}

'converter2 relaxed voltage ramp policy: PASS'
