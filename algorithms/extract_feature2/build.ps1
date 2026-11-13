param(
    [ValidateSet('Build','Test','Deploy','Package')][string]$Action='Build',
    [string]$SdkRoot=(Join-Path $PSScriptRoot '../../install'),
    [string]$HostDirectory=(Join-Path $PSScriptRoot '../../install/bin'),
    [string]$BuildDirectory=(Join-Path $PSScriptRoot 'build'),
    [string]$DependencyCache=(Join-Path $PSScriptRoot '../../build/CMakeCache.txt'),
    [string]$CMakeExe=''
)
$ErrorActionPreference='Stop'
$SdkRoot=[IO.Path]::GetFullPath($SdkRoot)
$HostDirectory=[IO.Path]::GetFullPath($HostDirectory)
if(!$CMakeExe) {
    $command=Get-Command cmake -ErrorAction SilentlyContinue
    if($command) {$CMakeExe=$command.Source}
    else {
        $vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
        $vsRoot=& $vswhere -latest -products '*' -property installationPath
        $CMakeExe=Join-Path $vsRoot 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
    }
}
if($Action -eq 'Build') {
    $cache=@{}
    if(Test-Path -LiteralPath $DependencyCache) {Get-Content -LiteralPath $DependencyCache | ForEach-Object {if($_ -match '^([^/#][^:]*):[^=]+=(.*)$') {$cache[$matches[1]]=$matches[2]}}}
    $arguments=@('-S',$PSScriptRoot,'-B',$BuildDirectory,'-G','Visual Studio 17 2022','-A','x64','-T','v142',"-DFrameworkSDK_DIR=$SdkRoot/lib/cmake/FrameworkSDK",'-DEXTRACT_FEATURE2_GUI_SMOKE=OFF')
    foreach($key in @('Qt5_DIR','PCL_DIR','VTK_DIR','OpenCV_DIR')) {if($cache[$key]) {$arguments+="-D$key=$($cache[$key])"}}
    & $CMakeExe @arguments
    if($LASTEXITCODE) {throw 'Algorithm configure failed'}
    & $CMakeExe --build $BuildDirectory --config Release --parallel 4
    if($LASTEXITCODE) {throw 'Algorithm build failed'}
}
elseif($Action -eq 'Test') {
    $previousPath=$env:PATH
    try {
        $env:PATH="$SdkRoot/bin;$previousPath"
        & (Join-Path (Split-Path $CMakeExe) 'ctest.exe') --test-dir $BuildDirectory -C Release --output-on-failure
        if($LASTEXITCODE) {throw 'Algorithm tests failed'}
    } finally {$env:PATH=$previousPath}
}
elseif($Action -eq 'Deploy') {
    $plugin=Join-Path $BuildDirectory 'bin/Release/plugins/algorithm/ExtractFeature2Algorithm.dll'
    $target=Join-Path $HostDirectory 'plugins/algorithm'
    if(!(Test-Path -LiteralPath $plugin)) {throw 'Build the algorithm first'}
    if(!(Test-Path -LiteralPath (Join-Path $HostDirectory 'Launcher.exe'))) {throw 'HostDirectory must contain Launcher.exe'}
    New-Item -ItemType Directory -Force -Path $target | Out-Null
    Copy-Item -LiteralPath $plugin -Destination (Join-Path $target 'ExtractFeature2Algorithm.dll') -Force
    Write-Output 'Algorithm deployed. Restart the workbench to load it.'
}
else {
    $plugin=Join-Path $BuildDirectory 'bin/Release/plugins/algorithm/ExtractFeature2Algorithm.dll'
    if(!(Test-Path -LiteralPath $plugin)) {throw 'Build the algorithm first'}
    $stage=Join-Path $BuildDirectory 'package/ExtractFeature2Algorithm'
    $target=Join-Path $stage 'plugins/algorithm'
    New-Item -ItemType Directory -Force -Path $target | Out-Null
    Copy-Item -LiteralPath $plugin -Destination (Join-Path $target 'ExtractFeature2Algorithm.dll') -Force
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'DEPLOY.md') -Destination (Join-Path $stage 'ExtractFeature2Algorithm.README.md') -Force
    $archive=Join-Path $BuildDirectory 'ExtractFeature2Algorithm-1.9.0-win64.zip'
    Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $archive -Force
    Write-Output "Package: $archive"
}
