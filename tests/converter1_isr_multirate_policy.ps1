$ErrorActionPreference = 'Stop'
$path = Join-Path $PSScriptRoot '..\converter1.c'
$text = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    '#define CONTROL_SLOW_DIV',
    'void rms_publish(',
    'void software_protection_fast_update(',
    'void software_protection_slow_update(',
    'void control_outer_update_1khz(',
    'pll->sin_theta',
    'pll->cos_theta'
)) {
    if ($text -notmatch [regex]::Escape($required)) {
        throw "missing multirate ISR contract: $required"
    }
}

$isr = [regex]::Match($text, '(?s)__interrupt void adcA1ISR\(void\)\s*\{(.+?)\n\}').Groups[1].Value
if ($isr -match 'sinf\(|cosf\(|sqrtf\(') {
    throw 'runtime trig/sqrt remains directly in adcA1ISR'
}
if ($text -notmatch '#define POWER_STAGE_ENABLE\s+0U') {
    throw 'gate lock changed'
}

'converter1 ISR multirate policy: PASS'
