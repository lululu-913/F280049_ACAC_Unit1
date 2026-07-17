$ErrorActionPreference = 'Stop'

$unit1Path = Join-Path $PSScriptRoot '..\converter1.c'
$unit2Path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'
$unit1 = Get-Content -Raw -LiteralPath $unit1Path
$unit2 = Get-Content -Raw -LiteralPath $unit2Path

foreach ($item in @(
    @{ Name = 'Unit1'; Source = $unit1 },
    @{ Name = 'Unit2'; Source = $unit2 }
)) {
    if ([regex]::Matches(
        $item.Source,
        '(?m)^#define UO_DEFAULT\s+30\.0f\s*$').Count -ne 2) {
        throw "$($item.Name) 30 V/2 A policy: HALF and FULL defaults must both be 30.0 V"
    }

    foreach ($protection in @(
        '#define I_BRANCH_CMD_MAX                 2.05f',
        '#define IO_PK_TRIP                       3.50f',
        '#define IO_RMS_TRIP                      2.25f',
        '#define IT_PK_TRIP                       7.00f',
        '#define IT_RMS_TRIP                      4.50f',
        '#define IL_REF_PK_MAX                    5.80f',
        '#define IL_SW_FAST_LIMIT                 6.00f',
        '#define IL_CMPSS_TRIP                    6.50f',
        '#define IL_RMS_CONT                      4.00f',
        '#define IL_RMS_TRIP                      4.20f',
        '#define UO_RMS_HARD_MAX                  37.0f',
        '#define UO_ABS_PK_TRIP                   53.0f'
    )) {
        if (-not $item.Source.Contains($protection)) {
            throw "$($item.Name) 30 V/2 A policy changed required protection: $protection"
        }
    }

    $slowProtection = [regex]::Match(
        $item.Source,
        '(?s)void software_protection_slow\(void\)\s*\{(.*?)// 外环')
    if (-not $slowProtection.Success) {
        throw "$($item.Name) 30 V/2 A policy: slow protection body not found"
    }
    $body = $slowProtection.Groups[1].Value
    $rampRaise = $body.LastIndexOf(
        'dynamic_ov = fmaxf(dynamic_ov, ramp_ov_min);')
    $finalHardClamp = $body.LastIndexOf(
        'dynamic_ov = fminf(dynamic_ov, UO_RMS_HARD_MAX);')
    if (($rampRaise -lt 0) -or ($finalHardClamp -le $rampRaise)) {
        throw "$($item.Name) 30 V/2 A policy: final RMS overvoltage threshold can exceed the hard limit"
    }
}

foreach ($required in @(
    '#define IL_TO_OUTPUT_SIGN              (1.0f)',
    '#define CURRENT_KP                       6.0f',
    '#define CURRENT_KI                       160.0f',
    '#define SHARE_KP                         0.70f',
    '#define SHARE_KI                         30.0f'
)) {
    if (-not $unit1.Contains($required)) {
        throw "Unit1 30 V/2 A policy changed the validated sharing loop: $required"
    }
}

foreach ($required in @(
    '#define CURRENT_LOOP_INTEGRAL_ENABLE   0U',
    '#define CURRENT_KP                       0.25f',
    '#define CURRENT_KI                       0.001f',
    '#define VOLTAGE_KP                       1.0e-2f',
    '#define VOLTAGE_KI                       2.5e-1f',
    'u_ref_active = slew(u_ref_active, 5.0f, 0.150f);',
    'if (ss5_stable_ms < 300U) ss5_stable_ms++;',
    'if (ss5_stable_ms >= 300U)',
    'float ramp_span = fmaxf(target - 5.0f, 1.0f);',
    'ramp_span / 500.0f',
    'fabsf(meas.uo_rms - target) <= 3.0f',
    'if (voltage_stable_ms < 100U) voltage_stable_ms++;',
    'if (voltage_stable_ms >= 100U)',
    'fmaxf(2.0f, 0.15f * u_ref_active)',
    'if (voltage_error_ms >= 5000U) latch_fault(FAULT_START);'
)) {
    if (-not $unit2.Contains($required)) {
        throw "Unit2 30 V/2 A policy missing: $required"
    }
}

'dual-unit 30 V/2 A policy: PASS'
