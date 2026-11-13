param(
    [ValidateSet('Configure','Build','Test','Smoke','Deploy','Install')][string]$Action = 'Build',
    [string]$BuildDirectory = 'build',
    [string]$Configuration = 'Release',
    [string[]]$SmokeArguments = @()
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
$originalPath = $env:PATH
try {
    $cmakeCommand = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmakeCommand) { $cmakeExe = $cmakeCommand.Source }
    else {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
        $vsRoot = & $vswhere -latest -products '*' -property installationPath
        $cmakeExe = Join-Path $vsRoot 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
    }
    if ($Action -eq 'Configure') { & $cmakeExe --preset local; if ($LASTEXITCODE) { throw 'Configure failed' }; return }
    $cacheFile = Join-Path $BuildDirectory 'CMakeCache.txt'
    $cache = @{}
    Get-Content -LiteralPath $cacheFile | ForEach-Object { if ($_ -match '^([^/#][^:]*):[^=]+=(.*)$') { $cache[$matches[1]] = $matches[2] } }
    $qtBin = [IO.Path]::GetFullPath((Join-Path $cache.Qt5_DIR '../../../bin'))
    $pclRoot = [IO.Path]::GetFullPath((Join-Path $cache.PCL_DIR '..'))
    $vtkBin = [IO.Path]::GetFullPath((Join-Path $cache.VTK_DIR '../../../bin'))
    $opencvBin = [IO.Path]::GetFullPath((Join-Path $cache.OpenCV_DIR '../bin'))
    $env:PATH = "$qtBin;$pclRoot/bin;$vtkBin;$opencvBin;$pclRoot/3rdParty/FLANN/bin;$pclRoot/3rdParty/OpenNI2/Redist;$originalPath"
    switch ($Action) {
        Build { & $cmakeExe --build $BuildDirectory --config $Configuration --parallel 4 }
        Test { & (Join-Path (Split-Path $cmakeExe) 'ctest.exe') --test-dir $BuildDirectory -C $Configuration --output-on-failure }
        Smoke {
            # A pipeline waits for a Windows GUI executable; the visible render surface
            # is required for this OpenGL smoke test (not a background helper).
            & (Join-Path $BuildDirectory "bin/$Configuration/Launcher.exe") --smoke-test @SmokeArguments | Out-Host
        }
        Deploy { & $cmakeExe --build $BuildDirectory --config $Configuration --target deploy }
        Install {
            & $cmakeExe --install $BuildDirectory --config $Configuration
            if ($LASTEXITCODE) { throw 'Install failed' }
            $installBin = Join-Path $cache.CMAKE_INSTALL_PREFIX 'bin'
            & $cmakeExe "-DFW_BIN=$installBin" "-DFW_CONFIG=$Configuration" -P (Join-Path $BuildDirectory 'DeployRuntime.cmake')
        }
    }
    if ($LASTEXITCODE) { throw "$Action failed with exit code $LASTEXITCODE" }
}
finally { $env:PATH = $originalPath; Pop-Location }
