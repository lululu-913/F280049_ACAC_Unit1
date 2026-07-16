$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\converter1.c'
$source = Get-Content -Raw -LiteralPath $path

if ($source -notmatch '(?s)\(i == 3U\) && \(run_state == ST_SAFE\) &&\s*\(k->held_ms >= 100U\) && \(k->long_fired == 0U\)') {
    throw 'KEY4 start hold policy: SAFE start threshold is not 100'
}

if ($source -notmatch '(?s)\(i == 3U\) && \(run_state == ST_FAULT\) &&\s*\(k->held_ms >= 200U\) && \(k->long_fired == 0U\)') {
    throw 'KEY4 start hold policy: FAULT clear threshold is not 200'
}

'converter1 KEY4 start hold policy: PASS'
