$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($token in @(
    '#define DIAG_DISABLE_IL_SW_TRIP         0U',
    '#define DIAG_DISABLE_CMPSS_TRIP         0U',
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
    'XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX12);',
    'if (++il_fast_count >= 1U) latch_fault(FAULT_IL_PK);',
    'if (++io_pk_count >= 5U) latch_fault(FAULT_IO);',
    'if (++it_pk_count >= 5U) latch_fault(FAULT_IT);',
    'if (++uo_pk_count >= 3U) latch_fault(FAULT_UO_OV);',
    'if (++uo_rms_ov_count >= 10U)',
    'CMPSS_clearFilterLatchHigh(CMPSS7_BASE);',
    'CMPSS_clearFilterLatchLow(CMPSS7_BASE);'
)) {
    if (-not $source.Contains($token)) {
        throw "converter1 full protection policy missing: $token"
    }
}

if ($source -notmatch '(?m)^\s*if \(\+\+adc_bad_count >= 3U\) latch_fault\(FAULT_ADC\);') {
    throw 'converter1 full protection policy: ADC out-of-range fault is not executable'
}

foreach ($forbidden in @(
    '#define IL_REF_PK_MAX                   99.00f',
    '#define IL_SW_FAST_LIMIT                99.00f',
    '#define IL_CMPSS_TRIP                   99.00f',
    'TODO: 临时关闭ADC保护',
    '// if (++adc_bad_count >= 3U) latch_fault(FAULT_ADC);',
    'Io/It/Uo 峰值已删除'
)) {
    if ($source.Contains($forbidden)) {
        throw "converter1 full protection policy still contains disabled path: $forbidden"
    }
}

if (([regex]::Matches($source, 'CMPSS_clearFilterLatchHigh\(CMPSS7_BASE\);').Count -lt 2) -or
    ([regex]::Matches($source, 'CMPSS_clearFilterLatchLow\(CMPSS7_BASE\);').Count -lt 2)) {
    throw 'converter1 full protection policy: CMPSS filter latches lack safe re-arm'
}

$clearMatch = [regex]::Match(
    $source,
    '(?s)Uint16 fault_clear_conditions\(void\).*?if \(fault_clear_safe_ms == 0U\)\s*\{\s*CMPSS_clearFilterLatchHigh\(CMPSS7_BASE\);\s*CMPSS_clearFilterLatchLow\(CMPSS7_BASE\);')
if (-not $clearMatch.Success) {
    throw 'converter1 full protection policy: CMPSS re-arm must clear once at the qualification-window start'
}

'converter1 full protection policy: PASS'
