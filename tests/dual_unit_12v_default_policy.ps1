$ErrorActionPreference = 'Stop'

$unit1Path = Join-Path $PSScriptRoot '..\converter1.c'
$unit2Path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\converter2.c'

foreach ($item in @(
    @{ Name = 'Unit1'; Path = $unit1Path },
    @{ Name = 'Unit2'; Path = $unit2Path }
)) {
    $source = Get-Content -Raw -LiteralPath $item.Path
    $productionDefaults = [regex]::Matches(
        $source,
        '(?m)^#define UO_DEFAULT\s+12\.0f\s*$')

    if ($productionDefaults.Count -ne 2) {
        throw "$($item.Name) 12 V policy: HALF and FULL UO_DEFAULT must both be 12.0f"
    }

    if (-not $source.Contains('#define UO_DEFAULT                       1.0f')) {
        throw "$($item.Name) 12 V policy: isolated 8 V diagnostic default must remain 1.0 V"
    }
}

$unit2Source = Get-Content -Raw -LiteralPath $unit2Path
foreach ($platformRule in @(
    'u_ref_active = slew(u_ref_active, 5.0f, 0.150f);',
    'u_ref_active = slew(u_ref_active, 5.0f, 0.005f);',
    'float ramp_span = fmaxf(target - 5.0f, 1.0f);',
    'ramp_span / 333.333f'
)) {
    if (-not $unit2Source.Contains($platformRule)) {
        throw "Unit2 12 V policy: 5 V platform sequence changed unexpectedly: $platformRule"
    }
}

'dual-unit 12 V default policy: PASS'
