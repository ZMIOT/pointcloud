param([Parameter(Mandatory=$true)][string]$Expected,[Parameter(Mandatory=$true)][string]$Actual)
$ErrorActionPreference='Stop'
function Read-BinaryPcd([string]$Path) {
    $bytes=[IO.File]::ReadAllBytes($Path)
    $text=[Text.Encoding]::ASCII.GetString($bytes)
    $data=[regex]::Match($text,'(?m)^DATA binary\r?\n')
    if(!$data.Success) {throw 'Comparison requires uncompressed binary PCD'}
    $header=$text.Substring(0,$data.Index)
    $names=[regex]::Match($header,'(?m)^FIELDS (.+)\r?$').Groups[1].Value.Trim().Split(' ',[StringSplitOptions]::RemoveEmptyEntries)
    $sizes=[regex]::Match($header,'(?m)^SIZE (.+)\r?$').Groups[1].Value.Trim().Split(' ',[StringSplitOptions]::RemoveEmptyEntries)
    $counts=[regex]::Match($header,'(?m)^COUNT (.+)\r?$').Groups[1].Value.Trim().Split(' ',[StringSplitOptions]::RemoveEmptyEntries)
    $types=[regex]::Match($header,'(?m)^TYPE (.+)\r?$').Groups[1].Value.Trim().Split(' ',[StringSplitOptions]::RemoveEmptyEntries)
    $offsets=@{}; $stride=0
    for($i=0;$i -lt $names.Count;$i++) {
        $offsets[$names[$i]]=$stride
        if($names[$i] -in @('x','y','z') -and ($sizes[$i] -ne '4' -or $types[$i] -ne 'F')) {throw 'XYZ must be float32'}
        $stride+=[int]$sizes[$i]*[int]$counts[$i]
    }
    $points=[int][regex]::Match($header,'(?m)^POINTS (\d+)').Groups[1].Value
    return @{Bytes=$bytes;Start=$data.Index+$data.Length;Stride=$stride;Offsets=$offsets;Count=$points}
}
$a=Read-BinaryPcd $Expected; $b=Read-BinaryPcd $Actual
if($a.Count -ne $b.Count) {throw 'Point count changed'}
$maximum=0.0
for($i=0;$i -lt $a.Count;$i++) {
    foreach($axis in @('x','y','z')) {
        $x=[BitConverter]::ToSingle($a.Bytes,$a.Start+$i*$a.Stride+$a.Offsets[$axis])
        $y=[BitConverter]::ToSingle($b.Bytes,$b.Start+$i*$b.Stride+$b.Offsets[$axis])
        if([float]::IsNaN($x) -or [float]::IsNaN($y) -or [float]::IsInfinity($x) -or [float]::IsInfinity($y)) {throw 'Non-finite output'}
        $maximum=[Math]::Max($maximum,[Math]::Abs([double]$x-$y))
    }
}
if($maximum -ne 0) {throw "Coordinates changed: maximum error $maximum"}
[PSCustomObject]@{points=$a.Count;maxCoordinateDifference=$maximum;exactMatch=$true}
