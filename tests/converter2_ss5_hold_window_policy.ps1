$ErrorActionPreference = 'Stop'

$path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'
$source = Get-Content -Raw -LiteralPath $path

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*)$')

if (-not $stateMachine.Success) {
    throw 'Unit2 SS5/HOLD window policy: state machine not found'
}

$stateSource = $stateMachine.Groups[1].Value

$ss5 = [regex]::Match(
    $stateSource,
    '(?s)case ST_SS5_RAMP:(.*?)case ST_SS5_HOLD:')
$hold = [regex]::Match(
    $stateSource,
    '(?s)case ST_SS5_HOLD:(.*?)case ST_VOLT_RAMP:')

if (-not $ss5.Success -or -not $hold.Success) {
    throw 'Unit2 SS5/HOLD window policy: state blocks not found'
}

foreach ($block in @(
    @{ Name = 'SS5'; Body = $ss5.Groups[1].Value },
    @{ Name = 'HOLD'; Body = $hold.Groups[1].Value }
)) {
    foreach ($required in @(
        'meas.uo_rms >= 3.5f',
        'meas.uo_rms <= 6.5f',
        'current_limit_active == 0U'
    )) {
        if (-not $block.Body.Contains($required)) {
            throw "Unit2 $($block.Name) window policy missing: $required"
        }
    }
}

foreach ($obsolete in @(
    'meas.uo_rms >= 4.3f',
    'meas.uo_rms <= 5.7f'
)) {
    if ($ss5.Groups[1].Value.Contains($obsolete) -or
        $hold.Groups[1].Value.Contains($obsolete)) {
        throw "Unit2 SS5/HOLD window policy: obsolete window remains: $obsolete"
    }
}

'converter2 SS5/HOLD window policy: PASS'
