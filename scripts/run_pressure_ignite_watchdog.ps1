param(
  [Parameter(Mandatory = $false)]
  [string]$FromSimDir = "out\\pyramid_5x5\\sim_20260107_152913\\",

  [Parameter(Mandatory = $false)]
  [double]$PloadKPa = 0.6,

  [Parameter(Mandatory = $false)]
  [ValidateSet("augmented_lagrange","penalty_auto","penalty_soft")]
  [string]$ContactMode = "augmented_lagrange",

  [Parameter(Mandatory = $false)]
  [string]$LinearSolverMode = "direct_pardiso",

  [Parameter(Mandatory = $false)]
  [int]$TimeoutSec = 900,

  [Parameter(Mandatory = $false)]
  [int]$HeartbeatSec = 60
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function IsoNow() { (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss") }

function Normalize-Dir([string]$p) {
  if ([string]::IsNullOrWhiteSpace($p)) { return $p }
  $x = $p.Trim()
  $x = $x.Replace('/','\\')
  if ($x.EndsWith('\\') -eq $false) { $x = $x + '\\' }
  return $x
}

function Try-FindOutDir([datetime]$startTime) {
  $root = Join-Path (Get-Location).Path "out\\pyramid_5x5"
  if (-not (Test-Path $root)) { return $null }
  $dirs = Get-ChildItem $root -Directory -Filter "pressure_ignite_*" | Where-Object { $_.LastWriteTime -ge $startTime.AddSeconds(-2) } | Sort-Object LastWriteTime -Descending
  if ($dirs.Count -gt 0) { return $dirs[0].FullName }
  return $null
}

function Read-IgniteStatus([string]$outDir) {
  $status = "NA"
  $p = Join-Path $outDir "summary.txt"
  if (-not (Test-Path $p)) { return $status }
  try {
    $m = Select-String -Path $p -Pattern "^exit_status:" -ErrorAction SilentlyContinue | Select-Object -Last 1
    if ($null -ne $m) { $status = ($m.Line -replace "^exit_status:\\s*","").Trim() }
  } catch {}
  return $status
}

function Write-Heartbeat([string]$hbPath, [string]$reason, [string]$outDir) {
  $ts = IsoNow
  $st = "NA"
  if ($outDir -and (Test-Path $outDir)) { $st = Read-IgniteStatus $outDir }
  $outDirStr = "NA"
  if ($outDir -and $outDir.Trim() -ne "") { $outDirStr = $outDir }
  $line = "{0} {1} exit_status={2} out_dir={3}" -f $ts, $reason, $st, $outDirStr
  Add-Content -Path $hbPath -Value $line -Encoding UTF8
}

$FromSimDir = Normalize-Dir $FromSimDir

Write-Host "== run_pressure_ignite_watchdog =="
Write-Host ("FromSimDir        : {0}" -f $FromSimDir)
Write-Host ("PloadKPa          : {0}" -f $PloadKPa)
Write-Host ("ContactMode       : {0}" -f $ContactMode)
Write-Host ("LinearSolverMode  : {0}" -f $LinearSolverMode)
Write-Host ("TimeoutSec        : {0}" -f $TimeoutSec)
Write-Host ("HeartbeatSec      : {0}" -f $HeartbeatSec)

$preServers = @()
try { $preServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id } catch { $preServers = @() }

$start = Get-Date
$knownOutDir = $null
$hbPath = Join-Path (Get-Location).Path "logs\\pressure_ignite_heartbeat.txt"
if (-not (Test-Path (Split-Path $hbPath -Parent))) { New-Item -ItemType Directory -Path (Split-Path $hbPath -Parent) | Out-Null }

Write-Heartbeat $hbPath "STARTED" $knownOutDir

$env:PRESSURE_IGNITE_FROM_SIM_DIR = $FromSimDir
$env:SIM_P_LOAD_KPA = ([string]$PloadKPa)
$env:PHASE2_CONTACT_MODE = $ContactMode
$env:SIM_LINEAR_SOLVER_MODE = $LinearSolverMode
$env:SIM_TN_EPS_PA = "1"
$env:SIM_DISABLE_SEGREGATED = "1"

$p = Start-Process -FilePath "matlab" -ArgumentList @("-batch","run_pyramid_array_5x5_pressure_ignite") -PassThru -NoNewWindow
Write-Host ("MATLAB PID: {0}" -f $p.Id)

$deadline = $start.AddSeconds($TimeoutSec)
while (-not $p.HasExited) {
  if ((Get-Date) -ge $deadline) {
    Write-Host ("TIMEOUT: exceeded {0}s, killing MATLAB + started COMSOL server..." -f $TimeoutSec)
    if ($null -eq $knownOutDir -or -not (Test-Path $knownOutDir)) {
      $od = Try-FindOutDir $start
      if ($od) { $knownOutDir = $od }
    }
    Write-Heartbeat $hbPath "KILLED" $knownOutDir

    # Persist errors.json + append summary note into outDir if it exists.
    if ($knownOutDir -and (Test-Path $knownOutDir)) {
      $ts = IsoNow
      $err = @{
        exit_status = "KILLED_BY_WATCHDOG"
        P_load_kPa = $PloadKPa
        killed_at_iso = $ts
        timeout_sec = $TimeoutSec
      } | ConvertTo-Json -Depth 6
      Set-Content -Path (Join-Path $knownOutDir "errors.json") -Value $err -Encoding UTF8
      $lines = @(
        "",
        "WatchdogKill:",
        ("  exit_status: KILLED_BY_WATCHDOG"),
        ("  P_load_kPa: {0}" -f $PloadKPa),
        ("  killed_at_iso: {0}" -f $ts),
        ("  timeout_sec: {0}" -f $TimeoutSec)
      )
      $sum = Join-Path $knownOutDir "summary.txt"
      if (Test-Path $sum) { Add-Content -Path $sum -Value $lines -Encoding UTF8 } else { Set-Content -Path (Join-Path $knownOutDir "summary_watchdog.txt") -Value $lines -Encoding UTF8 }
    }

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

    exit 124
  }

  if ($null -eq $knownOutDir -or -not (Test-Path $knownOutDir)) {
    $od = Try-FindOutDir $start
    if ($od) { $knownOutDir = $od }
  }
  Write-Heartbeat $hbPath "HEARTBEAT" $knownOutDir
  Start-Sleep -Seconds $HeartbeatSec
}

Write-Heartbeat $hbPath "ENDED" $knownOutDir
Write-Host "MATLAB finished within timeout."
if ($knownOutDir) { Write-Host ("OutDir: {0}" -f $knownOutDir) }
exit 0
