$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$oled = [regex]::Match(
    $source,
    '(?s)void oled_update_one_line\(void\)\s*\{.*?\n\}\s*\n\s*//\*+ 安全条件')
if (-not $oled.Success) {
    throw 'converter1 Model2 OLED K policy: OLED function not found'
}

foreach ($token in @(
    'if (model == MODEL_PARALLEL)',
    'put_text(line, 0U, "K");',
    'put_fixed(line, 1U, k_active, 2U, 4U);',
    'put_text(line, 6U, "c");',
    'put_fixed(line, 7U, k_cmd, 2U, 4U);',
    'put_text(line, 12U, "w");',
    'put_u2(line, 13U, (Uint16)((fault_code != FAULT_OK) ? fault_code : warning_code));'
)) {
    if (-not $oled.Value.Contains($token)) {
        throw "converter1 Model2 OLED K policy missing: $token"
    }
}

if ($oled.Value -notmatch '(?s)else\s*\{\s*put_text\(line, 0U, "P"\);.*pll_ui\.omega / TWO_PI_F') {
    throw 'converter1 Model2 OLED K policy: non-Model2 PLL display was not preserved'
}

'converter1 Model2 OLED K policy: PASS'
