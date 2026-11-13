param([string]$OutputFile = (Join-Path (Split-Path $PSScriptRoot -Parent) 'examples/data/demo.pcd'))
$ErrorActionPreference = 'Stop'
$points = [Collections.Generic.List[string]]::new()
$format = [Globalization.CultureInfo]::InvariantCulture
for ($x = -20; $x -le 20; $x++) {
    for ($y = -20; $y -le 20; $y++) {
        $z = (($x * $y) % 3) * 0.0002
        $points.Add([string]::Format($format, '{0:F4} {1:F4} {2:F4} {3:F1}', $x * 0.03, $y * 0.03, $z, 100 + $x))
    }
}
for ($i = 0; $i -lt 12; $i++) {
    $points.Add([string]::Format($format, '{0:F4} {1:F4} {2:F4} 20', $i * 0.07, $i * 0.04, 0.15 + $i * 0.02))
}
$header = @('# .PCD v0.7 - example coordinates in metres', 'VERSION 0.7', 'FIELDS x y z intensity', 'SIZE 4 4 4 4', 'TYPE F F F F', 'COUNT 1 1 1 1', "WIDTH $($points.Count)", 'HEIGHT 1', 'VIEWPOINT 0 0 0 1 0 0 0', "POINTS $($points.Count)", 'DATA ascii')
New-Item -ItemType Directory -Path (Split-Path $OutputFile -Parent) -Force | Out-Null
[IO.File]::WriteAllLines($OutputFile, [string[]]($header + $points), [Text.UTF8Encoding]::new($false))
Write-Output "Generated $($points.Count) points: $OutputFile"
