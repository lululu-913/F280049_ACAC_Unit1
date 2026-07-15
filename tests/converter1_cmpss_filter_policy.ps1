$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

$exactTokens = @(
    '#define IL_CMPSS_FILTER_PRESCALE        9U',
    '#define IL_CMPSS_FILTER_WINDOW          5U',
    '#define IL_CMPSS_FILTER_THRESHOLD       3U',
    'CMPSS_configFilterHigh(CMPSS7_BASE, IL_CMPSS_FILTER_PRESCALE,',
    'CMPSS_configFilterLow(CMPSS7_BASE, IL_CMPSS_FILTER_PRESCALE,',
    'CMPSS_initFilterHigh(CMPSS7_BASE);',
    'CMPSS_initFilterLow(CMPSS7_BASE);',
    'XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX12_CMPSS7_CTRIPH_OR_L);',
    'EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_DCAEVT1);',
    'EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_DCAEVT1);'
)

foreach ($token in $exactTokens) {
    if (-not $source.Contains($token)) {
        throw "converter1 CMPSS filter policy missing: $token"
    }
}

$cmpssBody = [regex]::Match($source, '(?s)void cmpss_trip_init\(void\)\s*\{(.*?)\n\}').Groups[1].Value
if ([string]::IsNullOrWhiteSpace($cmpssBody)) {
    throw 'converter1 CMPSS filter policy: cmpss_trip_init body not found'
}

$filterRoutes = [regex]::Matches($cmpssBody,
    'CMPSS_configOutputs(High|Low)\(CMPSS7_BASE,\s*CMPSS_TRIP_FILTER \| CMPSS_TRIPOUT_FILTER\);')
if ($filterRoutes.Count -ne 2 -or
    -not ($filterRoutes.Value -match 'OutputsHigh') -or
    -not ($filterRoutes.Value -match 'OutputsLow')) {
    throw 'converter1 CMPSS filter policy: both high and low outputs must use FILTER'
}

if ($cmpssBody -match 'ASYNC_COMP') {
    throw 'converter1 CMPSS filter policy: asynchronous comparator trip path is enabled'
}

foreach ($pwm in @('EPWM1_BASE', 'EPWM2_BASE')) {
    if ($source -notmatch "EPWM_selectDigitalCompareTripInput\($pwm,\s*EPWM_DC_TRIP_TRIPIN7,\s*EPWM_DC_TYPE_DCAH\);") {
        throw "converter1 CMPSS filter policy: $pwm TRIP7-to-DCAH route missing"
    }
}

if (([regex]::Matches($source, 'CMPSS_clearFilterLatchHigh\(CMPSS7_BASE\);').Count -lt 2) -or
    ([regex]::Matches($source, 'CMPSS_clearFilterLatchLow\(CMPSS7_BASE\);').Count -lt 2)) {
    throw 'converter1 CMPSS filter policy: CMPSS latches are not cleared at init and acknowledged re-arm'
}

'converter1 CMPSS filter policy: PASS'