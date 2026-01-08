param(
  [Parameter(Mandatory = $false)][ValidateSet("augmented_lagrange","penalty_auto","penalty_soft")]
  [string]$ContactMode = "augmented_lagrange",

  # Comma-separated list of kPa values, e.g. "0.3,0.6,1.0"
  [Parameter(Mandatory = $false)]
  [string]$PressureKPaList = "0.3,0.6,1.0",

  # Per-point timeout (seconds). Each pressure point is run in its own MATLAB process.
  [Parameter(Mandatory = $false)]
  [int]$TimeoutPerPointSec = 600,

  # Optional: linear solver mode passed through to MATLAB (e.g. direct_pardiso).
  [Parameter(Mandatory = $false)]
  [string]$LinearSolverMode = "direct_pardiso"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function IsoNow() { (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss") }

function Ensure-Dir([string]$p) {
  if (-not (Test-Path $p)) { New-Item -ItemType Directory -Path $p | Out-Null }
}

function Append-MetricsFailureRow([string]$metricsPath, [double]$PkPa, [string]$reason, [int]$timeoutSec) {
  $ts = IsoNow
  $row = "{0},{1},NaN,NaN,NaN,0,NaN,NaN,none,0,{2},NaN,{3},0,watchdog,{4},{5},0,watchdog" -f `
    $ts, $PkPa, ($reason -replace ',', ';'), $LinearSolverMode, $ContactMode, $ContactMode
  Add-Content -Path $metricsPath -Value $row -Encoding UTF8

  $err = @{
    exit_status = "KILLED_BY_WATCHDOG"
    P_load_kPa = $PkPa
    killed_at_iso = $ts
    timeout_sec = $timeoutSec
    reason = $reason
  } | ConvertTo-Json -Depth 6
  $errPath = Join-Path (Split-Path $metricsPath -Parent) ("errors_{0}.json" -f ("P{0}kPa" -f $PkPa).Replace('.','p'))
  Set-Content -Path $errPath -Value $err -Encoding UTF8
}

$repoRoot = (Get-Location).Path
$outRoot = Join-Path $repoRoot "out\\pyramid_5x5"
Ensure-Dir $outRoot

$sweepDir = Join-Path $outRoot ("pressure_sweep_{0}" -f (Get-Date).ToString("yyyyMMdd_HHmmss"))
Ensure-Dir $sweepDir
$metricsPath = Join-Path $sweepDir "metrics_pressure.csv"

Write-Host "== run_pressure_sweep_watchdog =="
Write-Host ("SweepDir            : {0}" -f $sweepDir)
Write-Host ("ContactMode         : {0}" -f $ContactMode)
Write-Host ("PressureKPaList     : {0}" -f $PressureKPaList)
Write-Host ("TimeoutPerPointSec  : {0}" -f $TimeoutPerPointSec)
Write-Host ("LinearSolverMode    : {0}" -f $LinearSolverMode)

# Initialize header by running MATLAB in header-only mode (first point will also do it, but keep consistent).
$env:PRESSURE_SWEEP_DIR = $sweepDir
$env:PHASE2_CONTACT_MODE = $ContactMode
$env:SIM_LINEAR_SOLVER_MODE = $LinearSolverMode

$plist = @()
foreach ($s in ($PressureKPaList -split ",")) {
  $v = [double]::Parse($s.Trim(), [System.Globalization.CultureInfo]::InvariantCulture)
  $plist += $v
}

foreach ($PkPa in $plist) {
  $env:SIM_P_LOAD_KPA = ([string]$PkPa)
  Write-Host ("-- Point P_load_kPa={0}" -f $PkPa)

  $preServers = @()
  try { $preServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id } catch { $preServers = @() }

  $p = Start-Process -FilePath "matlab" -ArgumentList @("-batch","run_pyramid_array_5x5_pressure_sweep") -PassThru -NoNewWindow
  $deadline = (Get-Date).AddSeconds($TimeoutPerPointSec)

  while (-not $p.HasExited) {
    if ((Get-Date) -ge $deadline) {
      Write-Host ("TIMEOUT point {0} kPa: killing MATLAB + started COMSOL server..." -f $PkPa)
      try { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } catch {}
      try { Stop-Process -Name "MATLAB" -Force -ErrorAction SilentlyContinue } catch {}

      try {
        $postServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue
        foreach ($srv in $postServers) {
          if ($preServers -notcontains $srv.Id) {
            Write-Host ("Killing comsolmphserver PID: {0}" -f $srv.Id)
            try { Stop-Process -Id $srv.Id -Force -ErrorAction SilentlyContinue } catch {}
          }
        }
      } catch {}

      # Ensure a failure row + error json exists.
      if (Test-Path $metricsPath) {
        Append-MetricsFailureRow $metricsPath $PkPa "KILLED_BY_WATCHDOG" $TimeoutPerPointSec
      }
      break
    }
    Start-Sleep -Seconds 5
  }

  if ($p.HasExited) {
    Write-Host ("DONE point {0} kPa (exit={1})" -f $PkPa, $p.ExitCode)
  }
}

Write-Host ("Completed. Metrics: {0}" -f $metricsPath)
