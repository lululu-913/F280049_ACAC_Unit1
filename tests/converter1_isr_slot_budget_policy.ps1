$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    'Uint16 share_signal_processing_active(void);',
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
    'void signal_processing_voltage_publish_slow(void);'
)) {
    if (-not $source.Contains($required)) {
        throw "ISR slot budget policy missing declaration: $required"
    }
}

if ($source -match 'void signal_processing_slow\(void\)') {
    throw 'ISR slot budget policy: monolithic signal_processing_slow still exists'
}

$shareActive = [regex]::Match(
    $source,
    '(?s)Uint16 share_signal_processing_active\(void\)\s*\{(.*?)\n\}')
if (-not $shareActive.Success) {
    throw 'ISR slot budget policy: share signal-processing state gate not found'
}
foreach ($state in @('ST_OUTPUT_PLL', 'ST_SHARE_RAMP', 'ST_RUN', 'ST_STOP_RAMP')) {
    if (-not $shareActive.Groups[1].Value.Contains($state)) {
        throw "ISR slot budget policy: share processing does not include $state"
    }
}
foreach ($state in @('ST_PRECHECK', 'ST_INPUT_PLL', 'ST_WAIT_UO', 'ST_FAULT')) {
    if ($shareActive.Groups[1].Value.Contains($state)) {
        throw "ISR slot budget policy: unnecessary share processing enabled in $state"
    }
}

$fast = [regex]::Match(
    $source,
    '(?s)void signal_processing_fast\(void\)\s*\{(.*?)\n\}')
if (-not $fast.Success) {
    throw 'ISR slot budget policy: signal_processing_fast body not found'
}
if ($fast.Groups[1].Value -notmatch 'Uint16 share_active\s*=\s*share_signal_processing_active\(\);') {
    throw 'ISR slot budget policy: fast path does not cache the share-processing decision'
}
if (($fast.Groups[1].Value | Select-String -Pattern 'is_share_role\(\)' -AllMatches).Matches.Count -ne 0) {
    throw 'ISR slot budget policy: repeated is_share_role calls remain in fast path'
}

$isr = [regex]::Match(
    $source,
    '(?s)__interrupt void adcB1ISR\(void\)\s*\{(.*?)\n\}')
if (-not $isr.Success) {
    throw 'ISR slot budget policy: adcB1ISR body not found'
}

$isrBody = $isr.Groups[1].Value
foreach ($slotCall in @(
    'case 0U:',
    'pll_slow_amplitude_update(&pll_ui,',
    'case 1U:',
    'pll_slow_amplitude_update(&pll_uo, 2.0f);',
    'case 2U:',
    'pll_slow_normalize_update(&pll_ui);',
    'case 3U:',
    'pll_slow_normalize_update(&pll_uo);',
    'case 4U:',
    'pll_slow_theta_update(&pll_ui);',
    'case 5U:',
    'pll_slow_theta_update(&pll_uo);',
    'case 6U:',
    'software_protection_slow();',
    'case 7U:',
    'pll_slow_sin_dtheta_update(&pll_ui);',
    'case 8U:',
    'pll_slow_sin_dtheta_update(&pll_uo);',
    'case 9U:',
    'pll_slow_cos_dtheta_update(&pll_ui);',
    'case 10U:',
    'pll_slow_cos_dtheta_update(&pll_uo);',
    'case 11U:',
    'signal_processing_it_dq_publish_slow();',
    'case 12U:',
    'signal_processing_io_dq_publish_slow();',
    'case 13U:',
    'signal_processing_io2_dq_publish_slow();',
    'case 14U:',
    'control_update_slow();',
    'case 15U:',
    'signal_processing_il_publish_slow();',
    'case 16U:',
    'signal_processing_io_publish_slow();',
    'case 17U:',
    'signal_processing_it_publish_slow();',
    'case 18U:',
    'signal_processing_voltage_publish_slow();',
    'case 19U:',
    'pll_slow_commit_dtheta_update(&pll_ui);',
    'pll_slow_commit_dtheta_update(&pll_uo);'
)) {
    if (-not $isrBody.Contains($slotCall)) {
        throw "ISR slot budget policy missing staggered slot: $slotCall"
    }
}

if ($isrBody -notmatch '(?s)if \(run_state != ST_FAULT\)\s*\{\s*control_update_fast\(\);\s*gate_sequence_update\(\);\s*\}') {
    throw 'ISR slot budget policy: fault state still executes fast control and gate sequencing'
}

$slowControl = [regex]::Match(
    $source,
    '(?s)void control_update_slow\(void\)\s*\{(.*?)// 内环：电感电流 PI')
if (-not $slowControl.Success) {
    throw 'ISR slot budget policy: control_update_slow body not found'
}
if ($slowControl.Groups[1].Value -match 'sqrtf\(') {
    throw 'ISR slot budget policy: control slot still duplicates Uo amplitude sqrt'
}

if ([regex]::Matches($source, 'return \(count >= 100U\) \? 1U : 0U;').Count -ne 1) {
    throw 'ISR slot budget policy: dead_bus contains duplicate/unreachable return'
}

'converter1 ISR slot budget policy: PASS'
