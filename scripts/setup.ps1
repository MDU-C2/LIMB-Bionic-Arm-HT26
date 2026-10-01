param(
    [switch]$Start,
    [switch]$NoLaunch
)

# Windows 10/11: bootstrap without requiring Python, Conda, or admin access.
$ErrorActionPreference = 'Stop'
$repository = Split-Path -Parent $PSScriptRoot
$projectState = Join-Path $repository '.aurora'
# Micromamba's Windows command wrapper requires ASCII command arguments.
# Use a relative script path; Python itself handles the Unicode clone path.
$setupScript = 'scripts/setup_project.py'
$env:PYTHONUTF8 = '1'
$env:PYTHONIOENCODING = 'utf-8'

try {
    Set-Location -LiteralPath $repository
    if (-not [Environment]::Is64BitOperatingSystem -or
        $env:PROCESSOR_ARCHITECTURE -eq 'ARM64' -or
        $env:PROCESSOR_ARCHITEW6432 -eq 'ARM64') {
        throw 'This setup requires 64-bit Intel/AMD Windows 10 or 11.'
    }

    # Some native installers cannot open non-ASCII paths, even through junctions.
    # Use a separate cache for each clone under the current user's local data.
    # This requires no admin rights and leaves global Python installs untouched.
    $runtimeRoot = $env:AURORA_RUNTIME_ROOT
    if (-not $runtimeRoot) { $runtimeRoot = Join-Path $env:LOCALAPPDATA 'aurora' }
    if ($runtimeRoot -match '[^\x00-\x7F]|\s' -or -not [IO.Path]::IsPathRooted($runtimeRoot)) {
        throw 'Set AURORA_RUNTIME_ROOT to an absolute writable ASCII path without spaces and retry.'
    }
    $hasher = [Security.Cryptography.SHA256]::Create()
    try {
        $cloneHash = ([BitConverter]::ToString($hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($repository)))).Replace('-', '').Substring(0, 16)
    }
    finally { $hasher.Dispose() }
    New-Item -ItemType Directory -Path $projectState -Force | Out-Null
    New-Item -ItemType Directory -Path $runtimeRoot -Force | Out-Null
    $stateDirectory = Join-Path $runtimeRoot $cloneHash
    New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
    # Pin the manager too: newer cache layouts can exceed Windows path limits.
    $micromamba = Join-Path $stateDirectory 'micromamba-2.3.3.exe'
    $prefix = Join-Path $stateDirectory 'env'
    $environmentFile = Join-Path $stateDirectory 'environment.yml'
    $env:MAMBA_ROOT_PREFIX = Join-Path $stateDirectory 'mamba'
    $env:AURORA_SETUP_DIRECTORY = $stateDirectory
    @{ prefix = $prefix } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $projectState 'runtime.json') -Encoding UTF8
    $readyFile = Join-Path $stateDirectory 'ready.json'

    if ($Start -and -not $NoLaunch -and (Test-Path -LiteralPath $micromamba) -and
        (Test-Path -LiteralPath $readyFile) -and
        (Test-Path -LiteralPath (Join-Path $prefix 'python.exe'))) {
        & $micromamba --no-rc run -p $prefix python $setupScript --launch
        $launchExit = $LASTEXITCODE
        if ($launchExit -ne 10) { exit $launchExit }
        # Code 10 means dependencies changed or an earlier setup was interrupted.
    }

    New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
    # Invalidate readiness before doing any work, including a manual reinstall.
    if (Test-Path -LiteralPath $readyFile) { Remove-Item -LiteralPath $readyFile }

    if (-not (Test-Path -LiteralPath $micromamba)) {
        Write-Host 'Downloading the local Python environment manager...'
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        $download = "$micromamba.download"
        Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/mamba-org/micromamba-releases/releases/download/2.3.3-0/micromamba-win-64' -OutFile $download
        Move-Item -LiteralPath $download -Destination $micromamba -Force
    }

    Write-Host 'Installing Python, GUI, simulation, camera, and sensor dependencies...'
    Copy-Item -LiteralPath (Join-Path $repository 'src\simulation\environment.yml') -Destination $environmentFile -Force
    $operation = 'create'
    if (Test-Path -LiteralPath (Join-Path $prefix 'conda-meta\history')) { $operation = 'install' }
    & $micromamba --no-rc $operation -y -p $prefix -f $environmentFile
    if ($LASTEXITCODE -ne 0) { throw 'Python environment installation failed.' }

    $setupArguments = @('--install')
    if (-not $NoLaunch) { $setupArguments += '--launch' }
    & $micromamba --no-rc run -p $prefix python $setupScript @setupArguments
    exit $LASTEXITCODE
}
catch {
    Write-Host "Setup failed: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host 'Check your internet connection and free disk space, then retry.'
    exit 1
}
