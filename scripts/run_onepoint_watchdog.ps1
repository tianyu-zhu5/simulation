param(
  [Parameter(Mandatory=$false)][string]$ContactMode = "augmented_lagrange",
  [Parameter(Mandatory=$false)][string]$ResumeFromDir = "out/pyramid_5x5/sim_20260107_171121/",
  [Parameter(Mandatory=$false)][double]$SingleTargetUm = 1.0235,
  [Parameter(Mandatory=$false)][int]$TimeoutSec = 900,
  [Parameter(Mandatory=$false)][int]$HeartbeatSec = 60,
  [Parameter(Mandatory=$false)][switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Normalize-Dir([string]$p) {
  if ([string]::IsNullOrWhiteSpace($p)) { return $p }
  $pp = $p.Trim()
  $pp = $pp -replace "/", "\\"
  $pp = $pp.TrimEnd("\")
  $pp = $pp + "\"
  return $pp
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$logDir = Join-Path $repoRoot "logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }

$startIso = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")
$hbName = ("watchdog_heartbeat_{0}.txt" -f (Get-Date).ToString("yyyyMMdd_HHmmss"))
$hbPath = Join-Path $logDir $hbName
$knownOutDir = $null
$lastSeenStatus = "NA"
$lastSeenTarget = "NA"
$lastSeenMode = "NA"

function Try-FindOutDir([datetime]$startTime) {
  try {
    $outRoot = Join-Path $repoRoot "out\\pyramid_5x5"
    if (-not (Test-Path $outRoot)) { return $null }
    $cand = Get-ChildItem $outRoot -Directory -Filter "sim_*" -ErrorAction SilentlyContinue |
      Where-Object { $_.LastWriteTime -ge $startTime.AddSeconds(-5) } |
      Sort-Object Name |
      Select-Object -Last 1
    if ($null -ne $cand) { return $cand.FullName }
  } catch {}
  return $null
}

function Read-FallbackStatus([string]$outDir) {
  $status = $null
  $target = $null
  $mode = $null
  try {
    $fr = Join-Path $outDir "fallback_report.json"
    if (-not (Test-Path $fr)) { return @($status,$target,$mode) }
    $raw = Get-Content $fr -Raw -ErrorAction Stop
    $j = $raw | ConvertFrom-Json -ErrorAction Stop
    if ($null -ne $j.status) { $status = [string]$j.status }
    if ($null -ne $j.target_delta_total_um) { $target = [string]$j.target_delta_total_um }
    if ($null -ne $j.attempt_mode) { $mode = [string]$j.attempt_mode }
    if (($status -eq $null -or $status -eq "") -and $null -ne $j.live_attempt) {
      if ($null -ne $j.live_attempt.status) { $status = [string]$j.live_attempt.status }
      if ($null -ne $j.live_attempt.target_delta_total_um) { $target = [string]$j.live_attempt.target_delta_total_um }
      if ($null -ne $j.live_attempt.attempt_mode) { $mode = [string]$j.live_attempt.attempt_mode }
    }
  } catch {}
  return @($status,$target,$mode)
}

function Write-Heartbeat([string]$reason = "heartbeat") {
  $ts = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")

  if ($null -eq $knownOutDir -or -not (Test-Path $knownOutDir)) {
    $od = Try-FindOutDir $script:StartTime
    if ($null -ne $od) {
      $knownOutDir = $od
      $hbPath = Join-Path $knownOutDir "watchdog_heartbeat.txt"
    }
  }

  if ($null -ne $knownOutDir -and (Test-Path $knownOutDir)) {
    $vals = Read-FallbackStatus $knownOutDir
    if ($vals[0]) { $lastSeenStatus = $vals[0] }
    if ($vals[1]) { $lastSeenTarget = $vals[1] }
    if ($vals[2]) { $lastSeenMode = $vals[2] }
  }

  $outDirStr = "NA"
  if ($null -ne $knownOutDir -and $knownOutDir -ne "") { $outDirStr = $knownOutDir }
  $line = "{0} {1} status={2} target_delta_total_um={3} attempt_mode={4} out_dir={5}" -f $ts, $reason, $lastSeenStatus, $lastSeenTarget, $lastSeenMode, $outDirStr
  Add-Content -Path $hbPath -Value $line -Encoding UTF8
}

$ResumeFromDir = Normalize-Dir $ResumeFromDir

Write-Host "== run_onepoint_watchdog =="
Write-Host ("ContactMode     : {0}" -f $ContactMode)
Write-Host ("ResumeFromDir   : {0}" -f $ResumeFromDir)
Write-Host ("SingleTargetUm  : {0}" -f $SingleTargetUm)
Write-Host ("TimeoutSec      : {0}" -f $TimeoutSec)
Write-Host ("HeartbeatSec    : {0}" -f $HeartbeatSec)
Write-Host ("DryRun          : {0}" -f [bool]$DryRun)

# Record currently-running COMSOL servers so we only kill the ones we start.
$preServers = @()
try {
  $preServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id
} catch { $preServers = @() }

$script:StartTime = Get-Date
Write-Heartbeat "STARTED"

$env:PHASE2_CONTACT_MODE = $ContactMode
$env:RESUME_POST_ONSET_ONLY = "1"
$env:RESUME_FROM_DIR = $ResumeFromDir
$env:SIM_POST_SKIP_STATIONARY = "1"  # PTC-first
$env:SIM_GLOBAL_TIMEOUT_S = [string]$TimeoutSec
$env:SIM_BUDGET_POST_ONSET_S = [string]$TimeoutSec
$env:SIM_PER_SOLVE_TIMEOUT_S = "120"

# Single-point target: prefer SIM_SINGLE_TARGET_UM, with SIM_MICRO_TARGETS_UM as a fallback.
$env:SIM_SINGLE_TARGET_UM = [string]$SingleTargetUm
$env:SIM_MICRO_TARGETS_UM = [string]$SingleTargetUm

# Keep TD_RELAX as last fallback, but limit solver work to avoid "grinding" forever.
$env:SIM_TD_MAXITER = "10"
$env:SIM_TD_MAXSTEPS = "200"

$args = @("-batch", "run_pyramid_array_5x5_sim_runbook")
Write-Host ("Launching: matlab {0}" -f ($args -join " "))

$p = $null
if (-not $DryRun) {
  $p = Start-Process -FilePath "matlab" -ArgumentList $args -PassThru -NoNewWindow
  Write-Host ("MATLAB PID: {0}" -f $p.Id)
}

try {
  $deadline = $script:StartTime.AddSeconds($TimeoutSec)
  while ($true) {
    if ($DryRun) {
      if ((Get-Date) -ge $deadline) { throw "dryrun_timeout" }
    } else {
      if ($p.HasExited) {
        Write-Heartbeat "ENDED"
        Write-Host "MATLAB finished within timeout."
        exit 0
      }
      if ((Get-Date) -ge $deadline) { throw "timeout" }
    }

    Write-Heartbeat "HEARTBEAT"
    $sleepSec = [Math]::Max(1, [Math]::Min($HeartbeatSec, [int]([Math]::Ceiling(($deadline - (Get-Date)).TotalSeconds))))
    Start-Sleep -Seconds $sleepSec
  }
} catch {
  $why = $_.Exception.Message
  if ($DryRun -and $why -eq "dryrun_timeout") {
    Write-Heartbeat "DRYRUN_DONE"
    Write-Host ("DRYRUN finished (TimeoutSec={0}). Heartbeat file: {1}" -f $TimeoutSec, $hbPath)
    exit 0
  }

  Write-Host ("TIMEOUT: exceeded {0}s, killing MATLAB + started COMSOL server..." -f $TimeoutSec)
  Write-Heartbeat ("KILLED")

  if ($null -ne $p) {
    try { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } catch {}
    try { Stop-Process -Name "MATLAB" -Force -ErrorAction SilentlyContinue } catch {}
  }

  # Kill only the COMSOL mphservers that appeared after we started.
  try {
    $postServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue
    foreach ($srv in $postServers) {
      if ($preServers -notcontains $srv.Id) {
        Write-Host ("Killing comsolmphserver PID: {0}" -f $srv.Id)
        try { Stop-Process -Id $srv.Id -Force -ErrorAction SilentlyContinue } catch {}
      }
    }
  } catch {}

  $tsKill = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ss")
  $killLine = "KILLED at $tsKill, last_seen_status=$lastSeenStatus, last_seen_target=$lastSeenTarget, last_seen_mode=$lastSeenMode"
  Add-Content -Path $hbPath -Value $killLine -Encoding UTF8
  exit 124
}
