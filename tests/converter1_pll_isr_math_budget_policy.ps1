$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

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
    'void reset_share_signal_processing(void);',
    'volatile Uint16 debug_adc_ovf_slot'
)) {
    if (-not $source.Contains($required)) {
        throw "ISR math budget policy missing: $required"
    }
}

if ($source -match 'void pll_slow_update\(SinglePhasePll \*pll, float amp_min\)') {
    throw 'ISR math budget policy: monolithic PLL slow update still exists'
}
if ($source -match 'void signal_processing_dq_publish_slow\(void\)') {
    throw 'ISR math budget policy: three dq square roots still share one slot'
}
if ($source -match 'void signal_processing_measurement_publish_slow\(void\)') {
    throw 'ISR math budget policy: measurement square roots still share one slot'
}

$phaseRules = @{
    'pll_slow_amplitude_update' = @('sqrtf(', 'atan2f(', 'sinf(', 'cosf(')
    'pll_slow_normalize_update' = @('sqrtf(', 'atan2f(', 'sinf(', 'cosf(')
    'pll_slow_theta_update' = @('atan2f(', 'sqrtf(', 'sinf(', 'cosf(')
    'pll_slow_sin_dtheta_update' = @('sinf(', 'sqrtf(', 'atan2f(', 'cosf(')
    'pll_slow_cos_dtheta_update' = @('cosf(', 'sqrtf(', 'atan2f(', 'sinf(')
}

foreach ($name in $phaseRules.Keys) {
    $body = [regex]::Match(
        $source,
        "(?s)void $name\([^\)]*\)\s*\{(.*?)\n\}").Groups[1].Value
    if ([string]::IsNullOrWhiteSpace($body)) {
        throw "ISR math budget policy: body not found for $name"
    }
    $calls = $phaseRules[$name]
    if (-not $body.Contains($calls[0])) {
        throw "ISR math budget policy: $name lacks $($calls[0])"
    }
    foreach ($forbidden in $calls[1..3]) {
        if ($body.Contains($forbidden)) {
            throw "ISR math budget policy: $name also contains $forbidden"
        }
    }
}

$isr = [regex]::Match(
    $source,
    '(?s)__interrupt void adcB1ISR\(void\)\s*\{(.*?)\n\}').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($isr)) {
    throw 'ISR math budget policy: adcB1ISR body not found'
}

foreach ($slot in 0..18) {
    if (-not $isr.Contains("case ${slot}U:")) {
        throw "ISR math budget policy: carrier slot $slot is not explicitly assigned"
    }
}

foreach ($call in @(
    'pll_slow_amplitude_update(&pll_ui',
    'pll_slow_amplitude_update(&pll_uo',
    'pll_slow_normalize_update(&pll_ui)',
    'pll_slow_normalize_update(&pll_uo)',
    'pll_slow_theta_update(&pll_ui)',
    'pll_slow_theta_update(&pll_uo)',
    'pll_slow_sin_dtheta_update(&pll_ui)',
    'pll_slow_sin_dtheta_update(&pll_uo)',
    'pll_slow_cos_dtheta_update(&pll_ui)',
    'pll_slow_cos_dtheta_update(&pll_uo)',
    'pll_slow_commit_dtheta_update(&pll_ui)',
    'pll_slow_commit_dtheta_update(&pll_uo)',
    'signal_processing_it_dq_publish_slow();',
    'signal_processing_io_dq_publish_slow();',
    'signal_processing_io2_dq_publish_slow();',
    'control_update_slow();',
    'signal_processing_il_publish_slow();',
    'signal_processing_io_publish_slow();',
    'signal_processing_it_publish_slow();',
    'signal_processing_voltage_publish_slow();'
)) {
    if (-not $isr.Contains($call)) {
        throw "ISR math budget policy: scheduled call missing: $call"
    }
}

$slotExpected = [ordered]@{
    0  = 'pll_slow_amplitude_update(&pll_ui'
    1  = 'pll_slow_amplitude_update(&pll_uo'
    2  = 'pll_slow_normalize_update(&pll_ui)'
    3  = 'pll_slow_normalize_update(&pll_uo)'
    4  = 'pll_slow_theta_update(&pll_ui)'
    5  = 'pll_slow_theta_update(&pll_uo)'
    7  = 'pll_slow_sin_dtheta_update(&pll_ui)'
    8  = 'pll_slow_sin_dtheta_update(&pll_uo)'
    9  = 'pll_slow_cos_dtheta_update(&pll_ui)'
    10 = 'pll_slow_cos_dtheta_update(&pll_uo)'
    11 = 'signal_processing_it_dq_publish_slow();'
    12 = 'signal_processing_io_dq_publish_slow();'
    13 = 'signal_processing_io2_dq_publish_slow();'
    15 = 'signal_processing_il_publish_slow();'
    16 = 'signal_processing_io_publish_slow();'
    17 = 'signal_processing_it_publish_slow();'
    18 = 'signal_processing_voltage_publish_slow();'
}
$heavyCallPattern = 'pll_slow_(?:amplitude|normalize|theta|sin_dtheta|cos_dtheta)_update|signal_processing_(?:it_dq|io_dq|io2_dq|il|io|it|voltage)_publish_slow'
foreach ($slot in 0..19) {
    $case = [regex]::Match(
        $isr,
        "(?s)case ${slot}U:(.*?)(?=case \d+U:|default:)")
    if (-not $case.Success) {
        throw "ISR math budget policy: carrier slot $slot body not found"
    }
    if ([regex]::Matches($case.Groups[1].Value, $heavyCallPattern).Count -gt 1) {
        throw "ISR math budget policy: slot $slot contains multiple heavy helpers"
    }
}
foreach ($entry in $slotExpected.GetEnumerator()) {
    $slot = $entry.Key
    $expectedCall = $entry.Value
    $case = [regex]::Match(
        $isr,
        "(?s)case ${slot}U:(.*?)(?=case \d+U:|default:)")
    if (-not $case.Success -or
        -not $case.Groups[1].Value.Contains($expectedCall)) {
        throw "ISR math budget policy: slot $slot does not own $expectedCall"
    }
    if ([regex]::Matches($case.Groups[1].Value, $heavyCallPattern).Count -ne 1) {
        throw "ISR math budget policy: slot $slot must contain exactly one heavy helper"
    }
}

$stateMachine = [regex]::Match(
    $source,
    '(?s)void slow_state_machine_1ms\(void\)\s*\{(.*?)void keys_update_5ms').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($stateMachine)) {
    throw 'ISR math budget policy: slow state machine body not found'
}

$inputWait = [regex]::Match(
    $stateMachine,
    '(?s)case ST_INPUT_PLL:(.*?)(?=case ST_WAIT_UO:)')
if (-not $inputWait.Success -or
    -not $inputWait.Groups[1].Value.Contains('reset_share_signal_processing();') -or
    -not $inputWait.Groups[1].Value.Contains('run_state = ST_WAIT_UO;')) {
    throw 'ISR math budget policy: initial WAIT entry does not reset share histories'
}
if ($inputWait.Groups[1].Value.IndexOf('run_state = ST_WAIT_UO;') -gt
    $inputWait.Groups[1].Value.IndexOf('reset_share_signal_processing();')) {
    throw 'ISR math budget policy: initial WAIT state must be published before reset'
}

$dropoutWait = [regex]::Match(
    $stateMachine,
    '(?s)case ST_OUTPUT_PLL:(.*?)(?=case ST_SS5_RAMP:)')
if (-not $dropoutWait.Success -or
    -not $dropoutWait.Groups[1].Value.Contains('reset_share_signal_processing();') -or
    -not $dropoutWait.Groups[1].Value.Contains('run_state = ST_WAIT_UO;')) {
    throw 'ISR math budget policy: bus-drop WAIT entry does not invalidate old output PLL lock'
}
if ($dropoutWait.Groups[1].Value.IndexOf('run_state = ST_WAIT_UO;') -gt
    $dropoutWait.Groups[1].Value.IndexOf('reset_share_signal_processing();')) {
    throw 'ISR math budget policy: dropout WAIT state must be published before reset'
}

if ($isr -notmatch 'debug_adc_ovf_slot\s*=\s*debug_isr_slot;') {
    throw 'ISR math budget policy: overflow does not snapshot the active slot'
}

foreach ($hotFunction in @(
    'is_voltage_role',
    'share_signal_processing_active',
    'clampf_local',
    'slew',
    'sogi_update',
    'pll_update_fast',
    'rms_update_fast',
    'mean_update_fast',
    'pi_reset',
    'dq_lpf_update',
    'adcB1ISR',
    'sample_and_calibrate',
    'signal_processing_fast',
    'control_update_fast'
)) {
    $pragma = "#pragma CODE_SECTION($hotFunction, `".TI.ramfunc`")"
    if (-not $source.Contains($pragma)) {
        throw "ISR math budget policy: hot function remains in Flash: $hotFunction"
    }
}

'converter1 PLL/ISR math budget policy: PASS'
