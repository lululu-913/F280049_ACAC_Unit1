$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$fastMatch = [regex]::Match(
    $source,
    '(?s)void control_update_fast\(void\)\s*\{(.*?)//\*+ 运行状态机 \*+//',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

if (-not $fastMatch.Success) {
    throw 'Model2 current-loop policy: control_update_fast body not found'
}

$fast = $fastMatch.Groups[1].Value

if ($fast -notmatch '#if DIRECT_VOLTAGE_DUTY_TEST\s+if \(is_voltage_role\(\) != 0U\)') {
    throw 'Model2 current-loop policy: direct-duty bypass is not limited to the voltage role'
}

foreach ($token in @(
    'il_ref_target = IL_TO_OUTPUT_SIGN * iconv_ref / div;',
    'il_ref = slew(il_ref_prev, il_ref_target, 0.125f);',
    'error = il_ref - meas.il;',
    'v_pi = CURRENT_KP * error + pi_id.integral;',
    'd_unsat = dff + qsign * (L_HENRY * il_ref_dot + v_pi) / denom;',
    'duty = clampf_local(d_unsat, PWM_D_MIN, PWM_D_MAX);',
    'pi_id.integral += pi_id.ki * CONTROL_TS * error;'
)) {
    if (-not $fast.Contains($token)) {
        throw "Model2 current-loop policy missing: $token"
    }
}

$slowFunction = [regex]::Match(
    $source,
    '(?s)void control_update_slow\(void\)\s*\{(.*?)// 内环：电感电流 PI')

if (-not $slowFunction.Success) {
    throw 'Model2 current-loop policy: control_update_slow body not found'
}

$slow = $slowFunction.Groups[1].Value
if ($slow -notmatch 'else if \(is_share_role\(\) != 0U\)') {
    throw 'Model2 current-loop policy: share-role outer loop not found'
}

foreach ($token in @(
    'alpha = (k_active / (1.0f + k_active)) * share_alpha_ramp;',
    'target_d = alpha * it_dq.d;',
    'target_q = alpha * it_dq.q;',
    'corr_d = pi_update_dt(&pi_sd, target_d - io_dq.d, ctrl_active, 0.001f);',
    'corr_q = pi_update_dt(&pi_sq, target_q - io_dq.q, ctrl_active, 0.001f);',
    'iconv_ref = target_d * so + target_q * co +'
)) {
    if (-not $slow.Contains($token)) {
        throw "Model2 current-loop policy missing share-loop token: $token"
    }
}

'converter1 Model2 current-loop policy: PASS'
