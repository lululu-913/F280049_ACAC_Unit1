$ErrorActionPreference = 'Stop'
$path = Join-Path $PSScriptRoot '..\converter1.c'
$text = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    'void pll_slow_amplitude_update(SinglePhasePll *pll, float amp_min);',
    'void pll_slow_normalize_update(SinglePhasePll *pll);',
    'void pll_slow_theta_update(SinglePhasePll *pll);',
    'void pll_slow_sin_dtheta_update(SinglePhasePll *pll);',
    'void pll_slow_cos_dtheta_update(SinglePhasePll *pll);',
    'void pll_slow_commit_dtheta_update(SinglePhasePll *pll);',
    'void signal_processing_it_dq_publish_slow(void);',
    'void signal_processing_io_dq_publish_slow(void);',
    'void signal_processing_io2_dq_publish_slow(void);',
    'void signal_processing_il_publish_slow(void);',
    'void signal_processing_io_publish_slow(void);',
    'void signal_processing_it_publish_slow(void);',
    'void signal_processing_voltage_publish_slow(void);',
    'void software_protection_fast(void);',
    'void software_protection_slow(void);',
    'void control_update_slow(void);',
    'pll->sin_theta',
    'pll->cos_theta'
)) {
    if ($text -notmatch [regex]::Escape($required)) {
        throw "missing multirate ISR contract: $required"
    }
}

$isr = [regex]::Match($text, '(?s)__interrupt void adcB1ISR\(void\)\s*\{(.+?)\n\}').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($isr)) {
    throw 'multirate ISR contract: adcB1ISR body not found'
}
if ($isr -match 'sinf\(|cosf\(|sqrtf\(') {
    throw 'runtime trig/sqrt remains directly in adcA1ISR'
}
if ($text -notmatch '#define POWER_STAGE_ENABLE\s+1U') {
    throw 'restored power-stage setting changed'
}

'converter1 ISR multirate policy: PASS'
