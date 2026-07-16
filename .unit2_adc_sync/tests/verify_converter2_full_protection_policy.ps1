$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter2.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($token in @(
    '#define DIAG_DISABLE_IL_SW_TRIP         0U',
    '#define DIAG_DISABLE_CMPSS_TRIP         0U',
    '#define I_BRANCH_CMD_MAX                 2.05f',
    '#define IO_PK_TRIP                       3.50f',
    '#define IO_RMS_TRIP                      2.25f',
    '#define IL_REF_PK_MAX                    5.80f',
    '#define IL_SW_FAST_LIMIT                 6.00f',
    '#define IL_CMPSS_TRIP                    6.50f',
    '#define IL_RMS_CONT                      4.00f',
    '#define IL_RMS_TRIP                      4.20f',
    '#define ADC_RAW_RAIL_MIN                  8U',
    '#define ADC_RAW_RAIL_MAX                  4087U',
    '#define ADC_RAIL_BAD_LIMIT                20U',
    'XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX12);',
    'if (++adc_bad_count >= ADC_RAIL_BAD_LIMIT) latch_fault(FAULT_ADC);',
    'if (++il_fast_count >= 1U) latch_fault(FAULT_IL_PK);',
    'if (++io_pk_count >= 5U) latch_fault(FAULT_IO);',
    'if (++uo_pk_count >= 3U) latch_fault(FAULT_UO_OV);',
    'if (++uo_rms_ov_count >= 10U)',
    'if (fault_clear_safe_ms == 0U)',
    'CMPSS_clearFilterLatchHigh(CMPSS7_BASE);',
    'CMPSS_clearFilterLatchLow(CMPSS7_BASE);'
)) {
    if (-not $source.Contains($token)) {
        throw "converter2 full protection policy missing: $token"
    }
}

$faultClear = [regex]::Match(
    $source,
    '(?s)Uint16 fault_clear_conditions\(void\)\s*\{.*?\n\}\s*\n\s*Uint16 is_voltage_role\(void\)')
if (-not $faultClear.Success -or
    $faultClear.Value -notmatch 'meas\.raw_ui >= ADC_RAW_RAIL_MIN' -or
    $faultClear.Value -notmatch 'meas\.raw_io <= ADC_RAW_RAIL_MAX') {
    throw 'converter2 full protection policy: fault-clear ADC window is not using the common rail limits'
}

$tripClear = [regex]::Match(
    $source,
    '(?s)Uint16 trip_clear_qualify_fast\(void\)\s*\{.*?\n\}\s*\n\s*Uint16 pwm_clear_ost\(void\)')
if (-not $tripClear.Success -or
    $tripClear.Value -notmatch 'meas\.raw_ui >= ADC_RAW_RAIL_MIN' -or
    $tripClear.Value -notmatch 'meas\.raw_io <= ADC_RAW_RAIL_MAX') {
    throw 'converter2 full protection policy: trip-clear ADC window is not using the common rail limits'
}

$fastProtection = [regex]::Match(
    $source,
    '(?s)void software_protection_fast\(void\)\s*\{.*?\n\}\s*\n\s*void software_protection_slow\(void\)')
if (-not $fastProtection.Success -or
    $fastProtection.Value -notmatch 'meas\.raw_ui < ADC_RAW_RAIL_MIN' -or
    $fastProtection.Value -notmatch 'meas\.raw_io > ADC_RAW_RAIL_MAX') {
    throw 'converter2 full protection policy: running ADC protection is not using the common rail limits'
}

if ($source -match 'meas\.raw_(?:ui|uo|il|io|it)\s*(?:<|>|<=|>=)\s*(?:32U|64U|4031U|4063U)') {
    throw 'converter2 full protection policy: legacy ADC rail thresholds have returned'
}

foreach ($forbidden in @(
    '#define IL_REF_PK_MAX                   99.00f',
    '#define IL_SW_FAST_LIMIT                99.00f',
    '#define IL_CMPSS_TRIP                   99.00f',
    'Io/It/Uo 峰值已删除'
)) {
    if ($source.Contains($forbidden)) {
        throw "converter2 full protection policy still contains disabled path: $forbidden"
    }
}

if (([regex]::Matches($source, 'CMPSS_clearFilterLatchHigh\(CMPSS7_BASE\);').Count -lt 2) -or
    ([regex]::Matches($source, 'CMPSS_clearFilterLatchLow\(CMPSS7_BASE\);').Count -lt 2)) {
    throw 'converter2 full protection policy: CMPSS filter latches lack safe re-arm'
}

$cmpTrip = 6.5
$highCode = 2062.3 + 163.03 * $cmpTrip
$lowCode = 2062.3 - 163.03 * $cmpTrip
if (($highCode -ge 4094.0) -or ($lowCode -le 1.0)) {
    throw "converter2 full protection policy: CMPSS codes out of range: low=$lowCode high=$highCode"
}

'converter2 full protection policy: PASS'
