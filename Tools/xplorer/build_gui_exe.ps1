<#
.SYNOPSIS
    Rebuild ArcskyRelease.exe from Tools/xplorer/release_gui.py.

.DESCRIPTION
    Run this after editing release_gui.py. Produces a single-file, windowed exe
    with no console, then copies it to the repo root so it sits next to the
    repository it drives (the GUI locates the repo by walking up from its own
    location, so keeping it there means zero configuration).

    The exe bundles Python and tkinter. It does NOT bundle the repo, git, waf or
    Cygwin -- it shells out to those, exactly as a developer would by hand.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File Tools\xplorer\build_gui_exe.ps1

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File Tools\xplorer\build_gui_exe.ps1 -KeepConsole
    # keeps a console window so you can see Python tracebacks while debugging
#>
[CmdletBinding()]
param(
    [switch]$KeepConsole,
    [switch]$SkipCopy
)

$ErrorActionPreference = 'Stop'

# repo root = two levels up from this script
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Repo      = Split-Path -Parent (Split-Path -Parent $ScriptDir)
$Entry     = Join-Path $ScriptDir 'release_gui.py'
$Name      = 'ArcskyRelease'
$WorkDir   = Join-Path $env:TEMP 'xplorer-gui-build'
$DistDir   = Join-Path $WorkDir 'dist'

Write-Host "Xplorer release GUI packager" -ForegroundColor Cyan
Write-Host "  repo   : $Repo"
Write-Host "  entry  : $Entry"

if (-not (Test-Path $Entry)) { throw "not found: $Entry" }

# --- locate python -----------------------------------------------------------
$Py = (Get-Command python -ErrorAction SilentlyContinue)
if ($null -eq $Py) { $Py = (Get-Command py -ErrorAction SilentlyContinue) }
if ($null -eq $Py) { throw "python not found on PATH" }
$PyExe = $Py.Source
Write-Host "  python : $PyExe"

# --- verify tkinter and pyinstaller -----------------------------------------
& $PyExe -c "import tkinter" 2>$null
if ($LASTEXITCODE -ne 0) {
    throw "this Python has no tkinter; reinstall Python with the tcl/tk option"
}

& $PyExe -c "import PyInstaller" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "  installing pyinstaller..." -ForegroundColor Yellow
    & $PyExe -m pip install --disable-pip-version-check pyinstaller
    if ($LASTEXITCODE -ne 0) { throw "pip install pyinstaller failed" }
}

# --- syntax check before spending time on a build ----------------------------
& $PyExe -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read())" $Entry
if ($LASTEXITCODE -ne 0) { throw "release_gui.py has a syntax error (see above)" }
Write-Host "  syntax : OK" -ForegroundColor Green

# --- build -------------------------------------------------------------------
$WindowFlag = if ($KeepConsole) { '--console' } else { '--windowed' }

$PyiArgs = @(
    '-m', 'PyInstaller',
    '--noconfirm',
    '--clean',
    '--onefile',
    $WindowFlag,
    '--name', $Name,
    '--distpath', $DistDir,
    '--workpath', (Join-Path $WorkDir 'build'),
    '--specpath', $WorkDir,
    # keep the exe small: none of these are used by the GUI
    '--exclude-module', 'numpy',
    '--exclude-module', 'matplotlib',
    '--exclude-module', 'PIL',
    '--exclude-module', 'pandas',
    '--exclude-module', 'pytest',
    $Entry
)

Write-Host "`nRunning PyInstaller ($WindowFlag)..." -ForegroundColor Cyan
& $PyExe @PyiArgs
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }

$Built = Join-Path $DistDir "$Name.exe"
if (-not (Test-Path $Built)) { throw "expected output missing: $Built" }

$SizeMB = [math]::Round((Get-Item $Built).Length / 1MB, 1)
Write-Host "`nBuilt $Name.exe ($SizeMB MB)" -ForegroundColor Green

# --- place it next to the repo ----------------------------------------------
if (-not $SkipCopy) {
    $Target = Join-Path $Repo "$Name.exe"
    try {
        Copy-Item $Built $Target -Force
        Write-Host "Copied to $Target" -ForegroundColor Green
        Write-Host "(the GUI finds the repo by walking up from its own location,"
        Write-Host " so running it from there needs no configuration)"
    } catch {
        Write-Warning "could not copy to ${Target}: $($_.Exception.Message)"
        Write-Warning "the exe is still at $Built - close it if it is running"
    }
}

Write-Host "`nDone." -ForegroundColor Cyan
