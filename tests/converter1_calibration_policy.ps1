$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($token in @(
    '#define UI_ADC_GAIN                      36.443f',
    '#define UI_ADC_OFFSET                    2066.2f',
    '#define UO_ADC_GAIN                      36.378f',
    '#define UO_ADC_OFFSET                    2067.0f',
    '#define IL_ADC_GAIN                      164.61f',
    '#define IL_ADC_OFFSET                    2077.3f',
    '#define IO_ADC_GAIN                      333.10f',
    '#define IO_ADC_OFFSET                    2097.6f',
    '#define IT_ADC_GAIN                      162.97f',
    '#define IT_ADC_OFFSET                    2081.8f'
)) {
    if (-not $source.Contains($token)) {
        throw "converter1 calibration policy missing: $token"
    }
}

$cmpTrip = 6.5
$highCode = 2077.3 + 164.61 * $cmpTrip
$lowCode = 2077.3 - 164.61 * $cmpTrip
if (($highCode -ge 4094.0) -or ($lowCode -le 1.0)) {
    throw "converter1 calibration policy: CMPSS codes out of range: low=$lowCode high=$highCode"
}

'converter1 calibration policy: PASS'
