$ErrorActionPreference = 'Stop'

$unit1Path = Join-Path $PSScriptRoot '..\converter1.c'
$unit2Path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'
$unit1 = Get-Content -Raw -LiteralPath $unit1Path
$unit2 = Get-Content -Raw -LiteralPath $unit2Path

foreach ($required in @(
    '#define CURRENT_KP                       4.5f',
    '#define CURRENT_KI                       160.0f',
    '#define SHARE_KP                         0.50f',
    '#define SHARE_KI                         157.0f',
    'pi_id.out_min = -7.0f; pi_id.out_max = 7.0f;'
)) {
    if (-not $unit1.Contains($required)) {
        throw "Dual-unit loop strength policy missing Unit1 rule: $required"
    }
}

foreach ($required in @(
    '#define VOLTAGE_KP                       5.0e-3f',
    '#define VOLTAGE_KI                       2.5e-1f',
    'pi_vd.out_min = -0.80f;',
    'pi_vd.out_max =  1.40f;'
)) {
    if (-not $unit2.Contains($required)) {
        throw "Dual-unit loop strength policy missing Unit2 rule: $required"
    }
}

if ([regex]::Matches($unit2, 'pi_vd\.integral = 0\.35f;').Count -ne 2) {
    throw 'Dual-unit loop strength policy: both Unit2 voltage-ramp entry paths must preload 0.35 A'
}

foreach ($required in @(
    '#define CURRENT_LOOP_INTEGRAL_ENABLE   0U',
    '#define CURRENT_KP                       0.5f',
    '#define CURRENT_KI                       0.0f',
    'pi_id.ki = CURRENT_LOOP_INTEGRAL_ENABLE ? CURRENT_KI : 0.0f;',
    'pi_id.out_min = -2.0f; pi_id.out_max = 2.0f;'
)) {
    if (-not $unit2.Contains($required)) {
        throw "Dual-unit loop strength policy missing Unit2 weak-P/no-I rule: $required"
    }
}

'dual-unit 12 V loop strength policy: PASS'
