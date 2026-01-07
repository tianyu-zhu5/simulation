param(
  [Parameter(Mandatory=$false)][string]$ContactMode = "augmented_lagrange",
  [Parameter(Mandatory=$false)][string]$ResumeFromDir = "out/pyramid_5x5/sim_20260107_171121/",
  [Parameter(Mandatory=$false)][double]$SingleTargetUm = 1.0235,
  [Parameter(Mandatory=$false)][int]$TimeoutSec = 900
)

$ErrorActionPreference = "Stop"

function Normalize-Dir([string]$p) {
  if ([string]::IsNullOrWhiteSpace($p)) { return $p }
  $pp = $p.Trim()
  $pp = $pp -replace "/", "\\"
  if (-not $pp.EndsWith("\\")) { $pp = $pp + "\\" }
  return $pp
}

$ResumeFromDir = Normalize-Dir $ResumeFromDir

Write-Host "== run_onepoint_watchdog =="
Write-Host ("ContactMode     : {0}" -f $ContactMode)
Write-Host ("ResumeFromDir   : {0}" -f $ResumeFromDir)
Write-Host ("SingleTargetUm  : {0}" -f $SingleTargetUm)
Write-Host ("TimeoutSec      : {0}" -f $TimeoutSec)

# Record currently-running COMSOL servers so we only kill the ones we start.
$preServers = @()
try {
  $preServers = Get-Process -Name "comsolmphserver" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id
} catch { $preServers = @() }

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

$p = Start-Process -FilePath "matlab" -ArgumentList $args -PassThru -NoNewWindow
Write-Host ("MATLAB PID: {0}" -f $p.Id)

try {
  Wait-Process -Id $p.Id -Timeout $TimeoutSec -ErrorAction Stop
  Write-Host "MATLAB finished within timeout."
  exit 0
} catch {
  Write-Host ("TIMEOUT: exceeded {0}s, killing MATLAB + started COMSOL server..." -f $TimeoutSec)
  try { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } catch {}
  try { Stop-Process -Name "MATLAB" -Force -ErrorAction SilentlyContinue } catch {}

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

  exit 124
}

