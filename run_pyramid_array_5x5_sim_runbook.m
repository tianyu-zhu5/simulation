function run_pyramid_array_5x5_sim_runbook()
%RUN_PYRAMID_ARRAY_5X5_SIM_RUNBOOK Solve the 5x5 pyramid array model using the RUNBOOK strategy.
%
% Strategy (see RUNBOOK_PYRAMID_SOLVE.md + suggestion.md):
% - Long-term path: use SolidContact (dcnt1) only, explicitly bound to contact pair pc
% - Disable/remove obsolete Contact (cnt1)
% - Use displacement-control (Displacement2) on the upper plate, with continuation in delta
% - Enable geometric nonlinearity explicitly
% - Export minimal summary metrics and save solved MPH

import com.comsol.model.util.*

tpl = fullfile(pwd, 'out', 'pyramid_5x5', 'Pyramid_5x5_Mech.mph');
if ~exist(tpl, 'file')
    error('Missing 5x5 template: %s', tpl);
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>

simDir = fullfile(pwd, 'out', 'pyramid_5x5', ['sim_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(simDir, 'dir')
    mkdir(simDir);
end
resultsDir = fullfile(pwd, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
phase2MatrixPath = fullfile(resultsDir, 'phase2_matrix.csv');

summaryPath = fullfile(simDir, 'Pyramid_5x5_summary.txt');
metricsPath = fullfile(simDir, 'Pyramid_5x5_metrics.csv');
checkpointMph = fullfile(simDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
errorsPath = fullfile(simDir, 'errors.json');
fallbackReportPath = fullfile(simDir, 'fallback_report.json');
tdBridgePath = fullfile(simDir, 'td_bridge_results.csv');
ptcBridgePath = fullfile(simDir, 'ptc_bridge_results.csv');
baselineCsvPath = fullfile(pwd, 'baseline_results.csv');
outMph = fullfile(simDir, 'Pyramid_5x5_solved.mph');

% Ensure we always flush minimal artifacts even if we fail early (e.g. mesh build).
cleanupObj = onCleanup(@() finalize_run_files(summaryPath, metricsPath, errorsPath, fallbackReportPath)); %#ok<NASGU>

resumePostOnsetOnly = get_env_bool('RESUME_POST_ONSET_ONLY', false);
resumeFromDir = get_env_or_default('RESUME_FROM_DIR', '');
checkpointSourceDir = 'none';
lastSuccessFromMetricsUm = NaN;
lastFzFromMetricsN = NaN;
resumeCheckpointPath = '';
resumeMetricsPath = '';
if resumePostOnsetOnly
    resumeFromDir = resolve_path(pwd, resumeFromDir);
    checkpointSourceDir = resumeFromDir;
    resumeCheckpointPath = fullfile(resumeFromDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
    resumeMetricsPath = fullfile(resumeFromDir, 'Pyramid_5x5_metrics.csv');
    if ~exist(resumeCheckpointPath, 'file')
        error('RESUME_POST_ONSET_ONLY enabled but missing checkpoint: %s', resumeCheckpointPath);
    end
    if ~exist(resumeMetricsPath, 'file')
        error('RESUME_POST_ONSET_ONLY enabled but missing metrics: %s', resumeMetricsPath);
    end
    [lastSuccessFromMetricsUm, lastFzFromMetricsN] = read_last_metrics_delta(resumeMetricsPath);
    if ~isfinite(lastSuccessFromMetricsUm)
        error('RESUME_POST_ONSET_ONLY enabled but could not parse last_success from metrics: %s', resumeMetricsPath);
    end
    try, copyfile(resumeCheckpointPath, checkpointMph, 'f'); catch, end
    try, copyfile(resumeMetricsPath, metricsPath, 'f'); catch, end
end
modelSource = tpl;
if resumePostOnsetOnly
    modelSource = resumeCheckpointPath;
end

% Optional: switch linear solver mode (e.g. force Direct(PARDISO)) for this run.
linearSolverMode = get_env_or_default('SIM_LINEAR_SOLVER_MODE', 'default');
linearSolverSwitched = false;
linearSolverNote = 'none';

model = mphload(modelSource);
model.hist.disable();

try
    [linearSolverSwitched, linearSolverNote] = configure_linear_solver_mode(model, linearSolverMode);
catch ME
    linearSolverSwitched = false;
    linearSolverNote = string(ME.message);
end

comp = model.component('comp1');
solid = comp.physics('solid');

% Representative geometry (keep Lpyr fixed for now; sweep later)
try, model.param.set('Lpyr', '10[um]'); catch, end

% Phase3.5-A: increase tip truncation (geometry-only).
% Apply even in RESUME mode; if rat_tip changes, rebuild geometry to keep model consistent.
ratTipWanted = numeric_env('SIM_RAT_TIP', NaN);
if ~isfinite(ratTipWanted)
    ratTipWanted = 0.25;
end
ratTipPrev = NaN;
try, ratTipPrev = model.param.evaluate('rat_tip'); catch, end
ratTipChanged = false;
ratTipGeomRebuilt = false;
if isfinite(ratTipPrev) && abs(ratTipPrev - ratTipWanted) > 1e-12
    ratTipChanged = true;
end
try, model.param.set('rat_tip', sprintf('%.6g', ratTipWanted)); catch, end
if ratTipChanged
    try
        comp.geom('geom1').run;
        ratTipGeomRebuilt = true;
    catch
        ratTipGeomRebuilt = false;
    end
end

% Use displacement-control continuation in delta (um), with adaptive step subdivision.
% NOTE: With gap0=1[um], contact starts near delta≈1[um]. We therefore solve a baseline
% delta that engages contact, then apply indentation steps on top of that baseline.
try
    gap0_um = model.param.evaluate('gap0') * 1e6;
catch
    gap0_um = 1.0;
end
deltaBaseUm = gap0_um + 0.05;
deltaIndentUm = [0.05 0.1 0.15 0.2 0.22 0.25 0.3];
deltaTargetsUm = deltaBaseUm + deltaIndentUm;
minStepUm = 1e-4; % smallest step allowed when bisecting (contact onset can require tiny steps)
maxBisectLevels = 12;
minIndentStepUm = 0.001;
perSolveTimeoutPreS = 120;
segmentedMaxStepUmPre = 0.002;
segmentedMaxStagesPre = 3;
perSolveTimeoutPostS = 120;
segmentedMaxStepUmPost = 0.001;
segmentedMaxStagesPost = 5;
globalTimeoutS = 900;
onsetBridgeMaxGapUm = 0.01;
preBisectLevels = 2;
ptcTimeStep = 0.05;
ptcMaxSteps = 50;
ptcDamping = 0.5;
bridgeNT = 25;
bridgeDt = 0.05;
tdMaxIterEffective = numeric_env('SIM_TD_MAXITER', 10);
tdMaxStepsEffective = numeric_env('SIM_TD_MAXSTEPS', 200);
bridgeModeDefault = 'PTC';
allowRampFallback = false;
budgetPreOnsetS = 300;
budgetPostOnsetS = 300;
FzEps = 1e-9;
phase1TargetUm = 1.0227;
microTargetsUm = [1.0205 1.0210 1.0215 1.0220 1.0223 1.0225 1.0227 1.0230 1.0235 1.0240 1.0245 1.0250 1.026 1.027 1.028 1.029 1.030];
microTargetsOverride = getenv('SIM_MICRO_TARGETS_UM');
if ~isempty(strtrim(microTargetsOverride))
    try
        ov = parse_num_list(microTargetsOverride);
        if ~isempty(ov)
            microTargetsUm = ov(:)';
        end
    catch
    end
end
singleTargetOverride = getenv('SIM_SINGLE_TARGET_UM');
if ~isempty(strtrim(singleTargetOverride))
    try
        ov1 = parse_num_list(singleTargetOverride);
        if ~isempty(ov1)
            microTargetsUm = ov1(1);
        end
    catch
    end
end
singleTargetMode = ~isempty(strtrim(singleTargetOverride));
microTargetsUmFull = microTargetsUm;
% In RESUME_POST_ONSET_ONLY single-target mode we may intentionally sample a point
% below the last_success (to add RP points in the solvable interval). Do not
% filter such a target out.
if resumePostOnsetOnly && isfinite(lastSuccessFromMetricsUm) && ~singleTargetMode
    microTargetsUm = microTargetsUm(microTargetsUm > lastSuccessFromMetricsUm + 1e-12);
end
earlyExitTargetUm = phase1TargetUm;
if resumePostOnsetOnly && isfinite(lastSuccessFromMetricsUm) && lastSuccessFromMetricsUm >= phase1TargetUm - 1e-12
    earlyExitTargetUm = max(microTargetsUmFull);
end
postSkipStationary = get_env_bool('SIM_POST_SKIP_STATIONARY', false);
disableTdRelax = get_env_bool('SIM_DISABLE_TD_RELAX', false);
disableSegmented = get_env_bool('SIM_DISABLE_SEGMENTED', false);
localMeshFactor = numeric_env('SIM_LOCAL_MESH_FACTOR', 1);
localMeshEnabled = isfinite(localMeshFactor) && localMeshFactor > 1;
selectionBoundaryCount = NaN;
localMeshBoundaryCountAfterFilter = NaN;
localMeshZWindowUm = numeric_env('SIM_Z_WINDOW_UM', 2.5);
localMeshHGrad = numeric_env('SIM_LOCAL_HGRAD', 1.2);
localMeshMinToMax = numeric_env('SIM_LOCAL_MIN_TO_MAX', 0.2); % min = max * 0.2 => max/5
localMeshMaxSizeM = NaN;
localMeshMinSizeM = NaN;
localMeshZTopUm = NaN;
localMeshZThresholdUm = NaN;
meshMinQuality = NaN;
meshInvertedElements = NaN;
meshCountQualLt0p1 = NaN;
meshCountQualLt0p01 = NaN;
contactMode = get_env_or_default('PHASE2_CONTACT_MODE', 'penalty_soft');
contactModeRequested = contactMode;
nuMode = get_env_or_default('PHASE2_NU_MODE', 'prod');
penaltyFactorMult = str2double(get_env_or_default('PHASE2_PENALTY_FACTOR_MULT', '1.0'));
if ~isfinite(penaltyFactorMult)
    penaltyFactorMult = 1.0;
end
contactTolScale = str2double(get_env_or_default('PHASE2_CONTACT_TOL_SCALE', '1.0'));
if ~isfinite(contactTolScale)
    contactTolScale = 1.0;
end
globalTimeoutS = numeric_env('SIM_GLOBAL_TIMEOUT_S', globalTimeoutS);
perSolveTimeoutPreS = numeric_env('SIM_PER_SOLVE_TIMEOUT_S', perSolveTimeoutPreS);
perSolveTimeoutPostS = numeric_env('SIM_PER_SOLVE_TIMEOUT_S', perSolveTimeoutPostS);
budgetPreOnsetS = numeric_env('SIM_BUDGET_PRE_ONSET_S', budgetPreOnsetS);
budgetPostOnsetS = numeric_env('SIM_BUDGET_POST_ONSET_S', budgetPostOnsetS);
segmentedMaxStepUmPost = numeric_env('SIM_SEG_MAX_STEP_UM', segmentedMaxStepUmPost);
segStagesOverride = numeric_env('SIM_SEG_MAX_STAGES', NaN);
if isfinite(segStagesOverride)
    segmentedMaxStagesPost = max(1, round(segStagesOverride));
end
tdDtOverride = numeric_env('SIM_TD_DT', NaN);
if isfinite(tdDtOverride) && tdDtOverride > 0
    bridgeDt = tdDtOverride;
end
FzEps = numeric_env('SIM_FZ_EPS_N', FzEps);
try, model.param.set('delta', '0[um]'); catch, end
try, model.param.set('P_load', '0[Pa]'); catch, end

% Long-term: enforce dcnt1-only and bind explicitly to pair pc.
dcntOnlyOk = false;
dcntOnlyNote = 'none';
try
    [dcntOnlyOk, dcntOnlyNote] = enforce_dcnt1_only(solid, 'dcnt1', 'cnt1', 'pc');
catch ME
    dcntOnlyOk = false;
    dcntOnlyNote = string(ME.message);
end

contactModeSupported = true;
contactModeEffective = contactMode;
contactModeNote = 'none';
try
    dcnt = solid.feature('dcnt1');
    [contactModeSupported, contactModeEffective, contactModeNote] = apply_contact_mode(dcnt, 'dcnt1', contactMode, penaltyFactorMult, contactTolScale);
catch ME
    contactModeSupported = false;
    contactModeNote = string(ME.message);
end

nuSupported = false;
nuParamName = '';
nuValueEffective = NaN;
nuNote = 'none';
try
    [nuSupported, nuParamName, nuValueEffective, nuNote] = apply_nu_mode(model, nuMode);
catch ME
    nuSupported = false;
    nuNote = string(ME.message);
end

materialTags = {};
pdmsMaterialTag = '';
pdmsPropKeys = {};
materialDiagNote = 'none';
try
    [materialTags, pdmsMaterialTag, pdmsPropKeys, materialDiagNote] = collect_material_diagnostics(model);
catch ME
    materialDiagNote = string(ME.message);
end

% Practical continuation trick:
% keep contact disabled during the initial approach, then enable it slightly after first touch.
% (dcnt1 can be hard to initialize when enabled too early with a positive gap.)
cntEnableUm = gap0_um + 0.02;

% Pressure load off (keep node but zero it)
try
    bndl = solid.feature('bndl1');
    bndl.set('forceType', 'FollowerPressure');
    bndl.set('pressure', '0[Pa]');
catch
end

% Upper plate drive (per suggestion.md):
% Do NOT prescribe displacement on the contact face (plate bottom). Prescribe only on the
% plate top face. Plate thickness is increased in the builder to reduce compression strain.
pc = model.component('comp1').pair('pc');
bnd_rigid_top = solid.feature('bndl1').selection.entities;
bnd_rigid_bot = pc.source.entities;
bnd_eval = pc.destination.entities;
try, selectionBoundaryCount = numel(bnd_eval); catch, selectionBoundaryCount = NaN; end
try, solid.feature('disp1').active(false); catch, end
try, solid.feature('disp_rigid').active(false); catch, end

try
    solid.feature('disp_top');
    hasDispTop = true;
catch
    hasDispTop = false;
end
if ~hasDispTop
    solid.create('disp_top', 'Displacement2', 2);
end
solid.feature('disp_top').selection.set(bnd_rigid_top);
solid.feature('disp_top').set('Direction', {'prescribed','prescribed','prescribed'});
solid.feature('disp_top').set('U0', {'0','0','-delta'});

% Study: enable geometric nonlinearity and continuation in delta
st = model.study('std1').feature('stat');
st.set('geometricNonlinearity', 'on');
try, st.set('geometricNonlinearityActive', 'on'); catch, end
st.set('useparam', 'off');
try, st.set('initmethod', 'sol'); catch, end
try, st.set('initsol', 'current'); catch, end
try, st.set('useinitsol', 'off'); catch, end

% Pre-mesh summary (so mesh failures still leave a readable summary with local-mesh config).
try
    fid0 = fopen(summaryPath, 'w', 'n', 'UTF-8');
    fprintf(fid0, "summary_stage: PRE_MESH\n");
    fprintf(fid0, "Model_source: %s\n", string(modelSource));
    fprintf(fid0, "resume_post_onset_only: %d\n", resumePostOnsetOnly);
    fprintf(fid0, "resume_from_dir: %s\n", string_or_none(resumeFromDir));
    fprintf(fid0, "checkpoint_source_dir: %s\n", string_or_none(checkpointSourceDir));
    fprintf(fid0, "rat_tip_prev: %s\n", num2str(ratTipPrev));
    fprintf(fid0, "rat_tip_wanted: %s\n", num2str(ratTipWanted));
    fprintf(fid0, "rat_tip_changed: %d\n", ratTipChanged);
    fprintf(fid0, "rat_tip_geom_rebuilt: %d\n", ratTipGeomRebuilt);
    fprintf(fid0, "local_mesh_enabled: %d\n", localMeshEnabled);
    fprintf(fid0, "local_mesh_factor: %g\n", localMeshFactor);
    fprintf(fid0, "local_mesh_z_window_um: %g\n", localMeshZWindowUm);
    fprintf(fid0, "selection_boundary_count: %s\n", num2str(selectionBoundaryCount));
    fclose(fid0);
catch
end

% Mesh controls:
% - Global mesh1.autoMeshSize(3) remains as in the template (do not override here).
% - Optionally refine locally on the contact destination boundaries via a filtered subset
%   of pc.destination.entities (centroid_z filter in a z-window near the pyramid tips).

% Mesh: keep the template default (the 5x5 model can be very large).
% IMPORTANT: explicitly build the mesh and fail fast if meshing is incomplete.
meshHasProblems = false;
meshProblemsNote = 'none';
try
    if ~resumePostOnsetOnly || ratTipChanged || localMeshEnabled
        if localMeshEnabled
            % Base mesh run first (required to evaluate centroid_z integrals robustly).
            comp.mesh('mesh1').run;

            % Filter the refined boundary set: only boundaries within z_window from the topmost centroid.
            try
                [bndFiltered, localMeshZTopUm, localMeshZThresholdUm] = filter_boundaries_by_centroid_z(model, bnd_eval, localMeshZWindowUm);
                localMeshBoundaryCountAfterFilter = numel(bndFiltered);
            catch
                bndFiltered = [];
                localMeshBoundaryCountAfterFilter = 0;
            end
            if isempty(bndFiltered)
                error('Local mesh enabled but boundary filter produced empty selection (z_window_um=%g).', localMeshZWindowUm);
            end

	            % Configure local mesh sizing on the filtered selection (quality-first settings).
	            try
	                msBase = mphmeshstats(model);
	                [globalHmaxM, globalHminM] = extract_mesh_hmax_hmin(msBase);
	                % Quality-first sizing: prefer explicit hmax/hmin to avoid pathological size behavior.
	                if isfinite(globalHmaxM)
	                    localMeshMaxSizeM = globalHmaxM / localMeshFactor;
	                else
	                    % Fallback: derive a conservative absolute size from the z-window.
	                    localMeshMaxSizeM = max(0.3e-6, (localMeshZWindowUm * 1e-6) / localMeshFactor);
	                end
	                localMeshMinSizeM = localMeshMaxSizeM * localMeshMinToMax;
	                if ~isfinite(localMeshHGrad)
	                    localMeshHGrad = 1.2;
	                end
	                localMeshHGrad = min(localMeshHGrad, 1.2);

	                mesh1 = comp.mesh('mesh1');
	                rebuild_mesh_with_local_size(mesh1, bndFiltered, localMeshHGrad, localMeshMaxSizeM, localMeshMinSizeM);
	                comp.mesh('mesh1').run;
	            catch ME2
	                error('Local mesh configuration failed: %s', string(ME2.message));
	            end
	        else
	            comp.mesh('mesh1').run;
	        end
	    end
    ms = mphmeshstats(model);
    if isfield(ms, 'isempty') && ms.isempty
        error('Mesh is empty after mesh1.run().');
    end
	    if isfield(ms, 'hasproblems') && ms.hasproblems
	        meshHasProblems = true;
	        try
	            if isfield(ms, 'problemtext')
	                meshProblemsNote = string(ms.problemtext);
            elseif isfield(ms, 'problems')
                meshProblemsNote = string(ms.problems);
            else
                meshProblemsNote = 'mesh hasproblems=true (details unavailable)';
            end
	        catch
	            meshProblemsNote = 'mesh hasproblems=true (details parse failed)';
	        end
	        [meshMinQuality, meshInvertedElements] = extract_mesh_quality(ms);
	        [meshCountQualLt0p1, meshCountQualLt0p01] = extract_mesh_quality_counts(ms);
	    end
	catch ME
	    error('Mesh build failed: %s', string(ME.message));
	end

% Solve via continuation
% Evaluate contact quantities on the contact destination boundaries.
% (bnd_eval already computed earlier from pc.destination.entities)

solveTime = 0;
hasSol = false;
prevDeltaUm = 0.0;
deltaHistoryUm = [];
lastSuccessDeltaUm = NaN;
failDeltaUm = NaN;
failDeltaIndentUm = NaN;
failReason = '';
err = [];
globalT0 = tic;
bisectLevelsUsed = 0;
narrowFailIntervalUm = [NaN NaN];
fallbackAttempts = struct('method', {}, 'delta_base_um', {}, 'delta_indent_um', {}, 'delta_total_um', {}, 'elapsed_s', {}, 'outcome', {}, 'error_summary', {});
bridgeAttemptId = 0;
preBisectAttempts = struct('delta_total_um', {}, 'success', {}, 'elapsed_s', {}, 'error_summary', {});
preBisectLastOk = NaN;
preBisectFail = NaN;
segmentedAttempts = struct('stage_id', {}, 'target_um', {}, 'method', {}, 'success', {}, 'elapsed_s', {}, 'error_summary', {});
microTargetAttempts = struct('target_delta_total_um', {}, 'attempt_order', {}, 'success', {}, 'elapsed_s', {}, 'error_summary', {});
liveAttempt = make_live_attempt('IDLE', NaN, 'none', 'none', 'none', 'none', NaN, 'none');
bridgeModeUsed = 'none';
bridgeSuccess = false;
bridgeFailReason = '';
ptcAttempted = false;
ptcFailReason = '';
segmentedAttempted = false;
segmentedFailReason = '';
tdAttempted = false;
tdFailReason = '';
bridgePolicy = 'PTC>SEGMENTED>TD_RELAX';
gitCommitHash = get_git_hash();
deltaPlanMode = tern(resumePostOnsetOnly, 'post_onset_micro', 'pre_onset');
contactOnsetDetected = false;
contactNote = '';
budgetUsedPre = 0;
budgetUsedPost = 0;
earlyExit = false;
earlyExitReason = '';
reserveGateTriggered = false;
reserveGateReason = '';
exitStatus = 'FAIL';
stopReason = 'none';
stopRequested = false;

wTop = nan(size(deltaTargetsUm));
wBot = nan(size(deltaTargetsUm));
tnMax = nan(size(deltaTargetsUm));
Ac = nan(size(deltaTargetsUm));
pnAvg = nan(size(deltaTargetsUm));
FzPlate = nan(size(deltaTargetsUm));
targetSolved = false(size(deltaTargetsUm));

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "Model_source: %s\n", modelSource);
fprintf(fid, "Template: %s\n", tpl);
fprintf(fid, "git_commit_hash: %s\n", gitCommitHash);
fprintf(fid, "bridge_policy: %s\n", bridgePolicy);
fprintf(fid, "delta_plan_mode: %s\n", deltaPlanMode);
fprintf(fid, "resume_post_onset_only: %d\n", resumePostOnsetOnly);
fprintf(fid, "resume_from_dir: %s\n", string_or_none(resumeFromDir));
fprintf(fid, "checkpoint_source_dir: %s\n", string_or_none(checkpointSourceDir));
fprintf(fid, "last_success_from_metrics_um: %s\n", num2str(lastSuccessFromMetricsUm));
fprintf(fid, "micro_targets: %s\n", mat2str(microTargetsUm));
fprintf(fid, "micro_targets_full: %s\n", mat2str(microTargetsUmFull));
fprintf(fid, "Fz_eps: %g\n", FzEps);
fprintf(fid, "contact_mode_requested: %s\n", contactModeRequested);
fprintf(fid, "contact_mode: %s\n", contactMode);
fprintf(fid, "contact_mode_effective: %s\n", contactModeEffective);
fprintf(fid, "contact_mode_supported: %d\n", contactModeSupported);
fprintf(fid, "contact_mode_note: %s\n", string_or_none(contactModeNote));
fprintf(fid, "penalty_factor_mult: %g\n", penaltyFactorMult);
fprintf(fid, "contact_tolerance_scale: %g\n", contactTolScale);
fprintf(fid, "nu_mode: %s\n", nuMode);
fprintf(fid, "nu_value_effective: %g\n", nuValueEffective);
fprintf(fid, "nu_supported: %d\n", nuSupported);
fprintf(fid, "nu_note: %s\n", string_or_none(nuNote));
fprintf(fid, "material_tags: [%s]\n", join_list(materialTags));
fprintf(fid, "pdms_material_tag: %s\n", string_or_none(pdmsMaterialTag));
fprintf(fid, "pdms_prop_keys: [%s]\n", join_list(pdmsPropKeys));
fprintf(fid, "material_diag_note: %s\n", string_or_none(materialDiagNote));
fprintf(fid, "diag_error: %s\n", tern(strlength(string(materialDiagNote))>0 && string(materialDiagNote)~="none", string(materialDiagNote), "none"));
if ~nuSupported
    fprintf(fid, "nu_supported_reason: %s\n", nu_supported_reason(nuNote));
end
if ~nuSupported
    fprintf(fid, "nu_mode_note: model not E-nu based; nu tuning disabled\n");
end
fprintf(fid, "rat_tip_prev: %s\n", num2str(ratTipPrev));
fprintf(fid, "rat_tip_wanted: %s\n", num2str(ratTipWanted));
fprintf(fid, "rat_tip_changed: %d\n", ratTipChanged);
fprintf(fid, "rat_tip_geom_rebuilt: %d\n", ratTipGeomRebuilt);
fprintf(fid, "local_mesh_enabled: %d\n", localMeshEnabled);
fprintf(fid, "local_mesh_factor: %g\n", localMeshFactor);
fprintf(fid, "selection_boundary_count: %s\n", num2str(selectionBoundaryCount));
fprintf(fid, "local_mesh_boundary_count_after_filter: %s\n", num2str(localMeshBoundaryCountAfterFilter));
fprintf(fid, "local_mesh_z_window_um: %g\n", localMeshZWindowUm);
fprintf(fid, "local_mesh_z_top_um: %s\n", num2str(localMeshZTopUm));
fprintf(fid, "local_mesh_z_threshold_um: %s\n", num2str(localMeshZThresholdUm));
fprintf(fid, "local_mesh_growth_rate: %g\n", localMeshHGrad);
fprintf(fid, "local_mesh_hmax_m: %s\n", num2str(localMeshMaxSizeM));
fprintf(fid, "local_mesh_hmin_m: %s\n", num2str(localMeshMinSizeM));
fprintf(fid, "mesh_hasproblems: %d\n", meshHasProblems);
fprintf(fid, "mesh_problems_note: %s\n", string_or_none(meshProblemsNote));
fprintf(fid, "mesh_min_quality: %s\n", num2str(meshMinQuality));
fprintf(fid, "mesh_inverted_elements: %s\n", num2str(meshInvertedElements));
fprintf(fid, "mesh_count_quality_lt_0_1: %s\n", num2str(meshCountQualLt0p1));
fprintf(fid, "mesh_count_quality_lt_0_01: %s\n", num2str(meshCountQualLt0p01));
fprintf(fid, "linear_solver_mode: %s\n", string_or_none(linearSolverMode));
fprintf(fid, "linear_solver_switched: %d\n", tern(linearSolverSwitched, 1, 0));
fprintf(fid, "linear_solver_note: %s\n", string_or_none(linearSolverNote));
try, fprintf(fid, "Lpyr: %s\n", char(model.param.get('Lpyr'))); catch, end
fprintf(fid, "delta_base_um: %g\n", deltaBaseUm);
fprintf(fid, "delta_indent_list_um: %s\n", mat2str(deltaIndentUm));
fprintf(fid, "delta_total_list_um: %s\n", mat2str(deltaTargetsUm));
fprintf(fid, "gap0_um: %g\n", gap0_um);
fprintf(fid, "cntEnableUm: %g\n", cntEnableUm);
fprintf(fid, "MinStep_um: %g\n", minStepUm);
fprintf(fid, "max_bisect_levels: %g\n", maxBisectLevels);
fprintf(fid, "min_indent_step_um: %g\n", minIndentStepUm);
fprintf(fid, "per_solve_timeout_pre_s: %g\n", perSolveTimeoutPreS);
fprintf(fid, "per_solve_timeout_post_s: %g\n", perSolveTimeoutPostS);
fprintf(fid, "global_timeout_s: %g\n", globalTimeoutS);
fprintf(fid, "budget_pre_onset_s: %g\n", budgetPreOnsetS);
fprintf(fid, "budget_post_onset_micro_s: %g\n", budgetPostOnsetS);
fprintf(fid, "early_exit_target_um: %g\n", earlyExitTargetUm);
fprintf(fid, "post_skip_stationary: %d\n", postSkipStationary);
fprintf(fid, "disable_td_relax: %d\n", disableTdRelax);
fprintf(fid, "disable_segmented: %d\n", disableSegmented);
ptcOnlyMode = resumePostOnsetOnly && singleTargetMode && disableSegmented && disableTdRelax;
fprintf(fid, "ptc_only_mode: %d\n", ptcOnlyMode);
fprintf(fid, "onset_bridge_max_gap_um: %g\n", onsetBridgeMaxGapUm);
fprintf(fid, "pre_bisect_levels: %g\n", preBisectLevels);
fprintf(fid, "segmented_max_step_um_pre: %g\n", segmentedMaxStepUmPre);
fprintf(fid, "segmented_max_stages_pre: %g\n", segmentedMaxStagesPre);
fprintf(fid, "segmented_max_step_um_post: %g\n", segmentedMaxStepUmPost);
fprintf(fid, "segmented_max_stages_post: %g\n", segmentedMaxStagesPost);
fprintf(fid, "ptc_timestep: %g\n", ptcTimeStep);
fprintf(fid, "ptc_max_steps: %g\n", ptcMaxSteps);
fprintf(fid, "ptc_damping: %g\n", ptcDamping);
fprintf(fid, "bridge_mode_default: %s\n", bridgeModeDefault);
fprintf(fid, "bridgeNT: %g\n", bridgeNT);
fprintf(fid, "bridgeDt: %g\n", bridgeDt);
fprintf(fid, "td_maxiter: %g\n", tdMaxIterEffective);
fprintf(fid, "td_maxsteps: %g\n", tdMaxStepsEffective);
fprintf(fid, "allow_ramp_fallback: %d\n", allowRampFallback);
fprintf(fid, "MechanicsNote: plate driven by top-face displacement (bottom contact face not prescribed).\n");
fprintf(fid, "\nProgressLog:\n");
fclose(fid);

initialize_metrics_file(metricsPath, resumeMetricsPath, resumePostOnsetOnly);
meshWarningContinue = 0;
if meshHasProblems
    if isfinite(meshInvertedElements) && meshInvertedElements > 0
        error('Mesh has inverted elements. Aborting before solve. mesh_min_quality=%s; mesh_inverted_elements=%s', num2str(meshMinQuality), num2str(meshInvertedElements));
    end
    if isfinite(meshMinQuality) && meshMinQuality < 0.10
        error('Mesh min quality < 0.10. Aborting before solve. mesh_min_quality=%s; mesh_inverted_elements=%s', num2str(meshMinQuality), num2str(meshInvertedElements));
    end
    meshWarningContinue = 1;
    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
    fprintf(fid, "mesh_warning_continue: %d\n", meshWarningContinue);
    fprintf(fid, "mesh_warning_note: continuing despite hasproblems=1 (no inverted elems, min_quality>=0.10)\n");
    fclose(fid);
end

% Start from a guaranteed-easy state: delta=0 (no contact), then continue upwards.
candUm = NaN;
cntWasActive = false;
switchToMicro = false;
lastFzPlate = NaN;
try
    if resumePostOnsetOnly
        deltaPlanMode = 'post_onset_micro';
        prevDeltaUm = lastSuccessFromMetricsUm;
        lastSuccessDeltaUm = lastSuccessFromMetricsUm;
        lastFzPlate = lastFzFromMetricsN;
        hasSol = true;
        cntWasActive = (prevDeltaUm >= cntEnableUm);
        deltaHistoryUm(end+1) = prevDeltaUm; %#ok<AGROW>
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "  - RESUME_POST_ONSET_ONLY enabled; skipping pre_onset and continuing from checkpoint\n");
        fprintf(fid, "    last_success_from_metrics_um: %.6g\n", lastSuccessFromMetricsUm);
        fprintf(fid, "    last_success_internal_um: %.6g\n", lastSuccessDeltaUm);
        fprintf(fid, "    checkpoint_source_dir: %s\n", string_or_none(checkpointSourceDir));
        fprintf(fid, "    micro_targets_active: %s\n", mat2str(microTargetsUm));
        fclose(fid);
        flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
    else
        try, st.set('useinitsol', 'off'); catch, end
        model.param.set('delta', '0[um]');
        try, solid.feature('dcnt1').active(false); catch, end
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "  - initial solve at delta=0 um (useinitsol=off)\n");
        fclose(fid);
        t0 = tic;
        model.study('std1').run();
        initialElapsed = toc(t0);
        solveTime = solveTime + initialElapsed;
        [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, initialElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
        hasSol = true;
        prevDeltaUm = 0.0;
        cntWasActive = false;
        deltaHistoryUm(end+1) = 0.0; %#ok<AGROW>
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "    initial OK at delta=0 um\n");
        fclose(fid);

        fallbackAttempts(end+1) = make_attempt('stat', deltaBaseUm, -deltaBaseUm, 0.0, initialElapsed, true, ''); %#ok<AGROW>
        flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));

        metrics0 = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
        append_metrics_row(metricsPath, 0.0, deltaBaseUm, metrics0);
        lastSuccessDeltaUm = 0.0;
        try, mphsave(model, checkpointMph); catch, end

        % Daytime pre-onset targets (skip low deltas).
        preOnsetTargetsUm = [1.01, 1.015, 1.018, 1.019, 1.0195, 1.01975, 1.02];
        deltaRampUm = unique([0, preOnsetTargetsUm], 'stable');

        for iT = 1:numel(deltaRampUm)
            if stopRequested
                break;
            end
            targetUm = deltaRampUm(iT);
            pending = targetUm; % queue of deltas to solve (midpoints inserted on failure)
            while ~isempty(pending)
                if stopRequested
                    pending = [];
                    break;
                end
                candUm = pending(1);
                stepUm = candUm - prevDeltaUm;
                if stepUm < 0
                    error('Non-monotone continuation: prev=%g um, cand=%g um', prevDeltaUm, candUm);
                end

        [cntActive, useInit] = prepare_stationary_step(model, solid, st, candUm, cntEnableUm, cntWasActive, hasSol);

        if toc(globalT0) > globalTimeoutS
            if isfinite(lastSuccessDeltaUm)
                stopRequested = true;
                exitStatus = 'SUCCESS_BUT_TIMEOUT';
                stopReason = sprintf('global_timeout_s exceeded before solve delta=%g um: %.1fs > %.1fs', candUm, toc(globalT0), globalTimeoutS);
                pending = [];
                break;
            end
            error('Global timeout exceeded before solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
        end
        [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
        if ~budgetOk
            if isfinite(lastSuccessDeltaUm)
                stopRequested = true;
                exitStatus = 'SUCCESS_BUT_BUDGET';
                stopReason = budgetWhy;
                pending = [];
                break;
            end
            failReason = budgetWhy;
            error('%s', budgetWhy);
        end

        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "  - try delta=%g um (step=%g um, useinitsol=%s)\n", candUm, stepUm, useInit);
        fclose(fid);

        t0 = tic;
        ok = true;
        errMsg = '';
        try
            model.study('std1').run();
        catch ME
            ok = false;
            errMsg = string(ME.message);
        end
        solveElapsed = toc(t0);
        solveTime = solveTime + solveElapsed;

        perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
        if solveElapsed > perSolveTimeoutS
            ok = false;
            errMsg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', solveElapsed, perSolveTimeoutS);
        end
        [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, solveElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);

        candIndent = candUm - deltaBaseUm;
        fallbackAttempts(end+1) = make_attempt('stat', deltaBaseUm, candIndent, candUm, solveElapsed, ok, errMsg); %#ok<AGROW>
        flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));

        if toc(globalT0) > globalTimeoutS
            if ok
                stopRequested = true;
                exitStatus = 'SUCCESS_BUT_TIMEOUT';
                stopReason = sprintf('global_timeout_s exceeded after successful delta=%g um: %.1fs > %.1fs', candUm, toc(globalT0), globalTimeoutS);
            else
                error('Global timeout exceeded after solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
            end
        end

        if ok
            fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
            fprintf(fid, "    solve OK at delta=%g um\n", candUm);
            fclose(fid);

                hasSol = true;
                prevDeltaUm = candUm;
                cntWasActive = cntActive;
                deltaHistoryUm(end+1) = candUm; %#ok<AGROW>
                pending(1) = [];
            lastSuccessDeltaUm = candUm;
            bisectLevelsUsed = 0;
            narrowFailIntervalUm = [NaN NaN];

                metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                append_metrics_row(metricsPath, candUm, deltaBaseUm, metrics);
                try, mphsave(model, checkpointMph); catch, end

                lastFzPlate = metrics.FzPlate;
                if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                    contactOnsetDetected = true;
                    if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                        contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                    end
                end
                if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                    deltaPlanMode = 'post_onset_micro';
                    switchToMicro = true;
                end
                if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= earlyExitTargetUm
                    earlyExit = true;
                    earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                end
                if switchToMicro || earlyExit
                    pending = [];
                    break;
                end

                if stopRequested && startsWith(exitStatus, 'SUCCESS_BUT_')
                    pending = [];
                    break;
                end

                % Record metrics for the requested deltaTargets points.
                tgtIdx = find(abs(deltaTargetsUm - candUm) < 1e-12, 1);
                if ~isempty(tgtIdx)
                    iOut = tgtIdx;
                    targetSolved(iOut) = true;
                    wTop(iOut) = metrics.wTop;
                    wBot(iOut) = metrics.wBot;
                    tnMax(iOut) = metrics.tnMax;
                    Ac(iOut) = metrics.Ac;
                    pnAvg(iOut) = metrics.pnAvg;
                    FzPlate(iOut) = metrics.FzPlate;
                end
        else
            fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
            fprintf(fid, "    solve FAILED at delta=%g um: %s\n", candUm, errMsg);
            fclose(fid);

            failDeltaUm = candUm;
            failDeltaIndentUm = candIndent;
            failReason = errMsg;
            narrowFailIntervalUm = [prevDeltaUm, candUm];

            % Pre-bridge micro-bisect in delta_indent space (shrink gap before TD).
            preLastOk = prevDeltaUm;
            preFail = candUm;
            preIndentOk = preLastOk - deltaBaseUm;
            preIndentFail = preFail - deltaBaseUm;
            for lvl = 1:preBisectLevels
                midIndent = 0.5 * (preIndentOk + preIndentFail);
                midUm = deltaBaseUm + midIndent;
                if toc(globalT0) > globalTimeoutS
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_TIMEOUT';
                        stopReason = sprintf('global_timeout_s exceeded during pre-bisect (preserving last_success=%g um): %.1fs > %.1fs', lastSuccessDeltaUm, toc(globalT0), globalTimeoutS);
                        pending = [];
                        break;
                    end
                    error('Global timeout exceeded during pre-bisect: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                end

                [cntActive, useInit] = prepare_stationary_step(model, solid, st, midUm, cntEnableUm, cntWasActive, hasSol);
                [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                if ~budgetOk
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = budgetWhy;
                        pending = [];
                        break;
                    end
                    failReason = budgetWhy;
                    error('%s', budgetWhy);
                end
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "    pre-bisect try delta=%g um (useinitsol=%s)\n", midUm, useInit);
                fclose(fid);

                t0 = tic;
                okPre = true;
                errPre = '';
                try
                    model.study('std1').run();
                catch ME
                    okPre = false;
                    errPre = string(ME.message);
                end
                elapsedPre = toc(t0);
                perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
                if elapsedPre > perSolveTimeoutS
                    okPre = false;
                    errPre = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', elapsedPre, perSolveTimeoutS);
                end
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsedPre, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                preBisectAttempts(end+1) = make_pre_bisect_attempt(midUm, okPre, elapsedPre, errPre); %#ok<AGROW>
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));

                if okPre
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "      pre-bisect OK at delta=%g um\n", midUm);
                    fclose(fid);
                    hasSol = true;
                    prevDeltaUm = midUm;
                    cntWasActive = cntActive;
                    deltaHistoryUm(end+1) = midUm; %#ok<AGROW>
                    lastSuccessDeltaUm = midUm;
                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, midUm, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end
                    preLastOk = midUm;
                    preIndentOk = preLastOk - deltaBaseUm;
                else
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "      pre-bisect FAILED at delta=%g um: %s\n", midUm, errPre);
                    fclose(fid);
                    preFail = midUm;
                    preIndentFail = preFail - deltaBaseUm;
                    failReason = errPre;
                end
            end
            preBisectLastOk = preLastOk;
            preBisectFail = preFail;
            narrowFailIntervalUm = [preBisectLastOk, preBisectFail];
            failDeltaUm = preBisectFail;
            failDeltaIndentUm = preBisectFail - deltaBaseUm;

            % Optional pre-activate contact at last_ok to improve TD initialization.
            if preBisectLastOk < cntEnableUm && preBisectFail >= cntEnableUm
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "    pre-activate contact at delta=%g um\n", preBisectLastOk);
                fclose(fid);
                try
                    model.param.set('delta', sprintf('%g[um]', preBisectLastOk));
                    try, solid.feature('dcnt1').active(true); catch, end
                    try, st.set('useinitsol', 'off'); catch, end
                    [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    if ~budgetOk
                        if isfinite(lastSuccessDeltaUm)
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_BUDGET';
                            stopReason = budgetWhy;
                        else
                            failReason = budgetWhy;
                        end
                        pending = [];
                        break;
                    end
                    t0 = tic;
                    model.study('std1').run();
                    elapsedPreAct = toc(t0);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsedPreAct, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    fallbackAttempts(end+1) = make_attempt('pre_activate', deltaBaseUm, preBisectLastOk - deltaBaseUm, preBisectLastOk, elapsedPreAct, true, ''); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    cntWasActive = true;
                catch ME
                    elapsedPreAct = 0;
                    fallbackAttempts(end+1) = make_attempt('pre_activate', deltaBaseUm, preBisectLastOk - deltaBaseUm, preBisectLastOk, elapsedPreAct, false, string(ME.message)); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                end
            end

            % Onset-cross fallback: PTC -> segmented stationary -> TD_RELAX (last fallback).
            if abs(preBisectFail - preBisectLastOk) <= onsetBridgeMaxGapUm
                % Bridge Mode 1: Stationary PTC
                bridgeModeUsed = 'PTC';
                ptcAttempted = true;
                bridgeAttemptId = bridgeAttemptId + 1;
                [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                if ~budgetOk
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = budgetWhy;
                        pending = [];
                        break;
                    end
                    failReason = budgetWhy;
                    error('%s', budgetWhy);
                end
                [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                if reserveGateTriggered
                    failDeltaUm = preBisectFail;
                    failDeltaIndentUm = preBisectFail - deltaBaseUm;
                    failReason = reserveGateReason;
                    error('%s', reserveGateReason);
                end
                [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, preBisectLastOk, preBisectFail, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                fallbackAttempts(end+1) = make_attempt('ptc', deltaBaseUm, preBisectFail - deltaBaseUm, preBisectFail, ptcElapsed, ptcOk, ptcErrMsg); %#ok<AGROW>
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                if ptcOk
                    bridgeSuccess = true;
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    ptc OK from %g to %g um\n", preBisectLastOk, preBisectFail);
                    fclose(fid);

                    hasSol = true;
                    prevDeltaUm = preBisectFail;
                    cntWasActive = (prevDeltaUm >= cntEnableUm);
                    deltaHistoryUm(end+1) = prevDeltaUm; %#ok<AGROW>
                    lastSuccessDeltaUm = prevDeltaUm;
                    bisectLevelsUsed = 0;

                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, prevDeltaUm, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end

                    lastFzPlate = metrics.FzPlate;
                    if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                        contactOnsetDetected = true;
                        if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                            contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                        end
                    end
                    if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                        deltaPlanMode = 'post_onset_micro';
                        switchToMicro = true;
                    end
                    if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= earlyExitTargetUm
                        earlyExit = true;
                        earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                    end

                    tgtIdx = find(abs(deltaTargetsUm - prevDeltaUm) < 1e-12, 1);
                    if ~isempty(tgtIdx)
                        iOut = tgtIdx;
                        targetSolved(iOut) = true;
                        wTop(iOut) = metrics.wTop;
                        wBot(iOut) = metrics.wBot;
                        tnMax(iOut) = metrics.tnMax;
                        Ac(iOut) = metrics.Ac;
                        pnAvg(iOut) = metrics.pnAvg;
                        FzPlate(iOut) = metrics.FzPlate;
                    end
                    pending(1) = [];
                    if switchToMicro || earlyExit
                        break;
                    end
                    continue;
                else
                    ptcFailReason = ptcErrMsg;
                end

                % Bridge Mode 2: Segmented stationary (with per-stage PTC fallback)
                segmentedAttempted = true;
                segErrMsg = '';
                segLastOk = preBisectLastOk;
                gapUm = preBisectFail - preBisectLastOk;
                nSeg = max(2, ceil(gapUm / segmentedMaxStepUmPre));
                nSeg = min(nSeg, segmentedMaxStagesPre);
                segTargets = linspace(preBisectLastOk, preBisectFail, nSeg + 1);
                segOk = true;
                for s = 2:numel(segTargets)
                    segTarget = segTargets(s);
                    stageId = s - 1;

                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [cntActive, useInit] = prepare_stationary_step(model, solid, st, segTarget, cntEnableUm, cntWasActive, hasSol);
                    [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    if ~budgetOk
                        if isfinite(lastSuccessDeltaUm)
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_BUDGET';
                            stopReason = budgetWhy;
                            pending = [];
                            break;
                        end
                        failReason = budgetWhy;
                        error('%s', budgetWhy);
                    end
                    t0 = tic;
                    okSeg = true;
                    errSeg = '';
                    try
                        model.study('std1').run();
                    catch ME
                        okSeg = false;
                        errSeg = string(ME.message);
                    end
                    elapsedSeg = toc(t0);
                    perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
                    if elapsedSeg > perSolveTimeoutS
                        okSeg = false;
                        errSeg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', elapsedSeg, perSolveTimeoutS);
                    end
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsedSeg, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    segmentedAttempts(end+1) = make_segment_attempt(stageId, segTarget, 'stat', okSeg, elapsedSeg, errSeg); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    if okSeg
                        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                        fprintf(fid, "    segmented OK at delta=%g um\n", segTarget);
                        fclose(fid);

                        hasSol = true;
                        prevDeltaUm = segTarget;
                        cntWasActive = cntActive;
                        deltaHistoryUm(end+1) = segTarget; %#ok<AGROW>
                        lastSuccessDeltaUm = segTarget;
                        metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                        append_metrics_row(metricsPath, segTarget, deltaBaseUm, metrics);
                        try, mphsave(model, checkpointMph); catch, end
                        segLastOk = segTarget;
                        continue;
                    end

                    % PTC fallback for this segment
                    ptcAttempted = true;
                    bridgeAttemptId = bridgeAttemptId + 1;
                    [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    if ~budgetOk
                        if isfinite(lastSuccessDeltaUm)
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_BUDGET';
                            stopReason = budgetWhy;
                            pending = [];
                            break;
                        end
                        failReason = budgetWhy;
                        error('%s', budgetWhy);
                    end
                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, segLastOk, segTarget, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    segmentedAttempts(end+1) = make_segment_attempt(stageId, segTarget, 'ptc', ptcOk, ptcElapsed, ptcErrMsg); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    if ptcOk
                        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                        fprintf(fid, "    segmented PTC OK at delta=%g um\n", segTarget);
                        fclose(fid);

                        hasSol = true;
                        prevDeltaUm = segTarget;
                        cntWasActive = (prevDeltaUm >= cntEnableUm);
                        deltaHistoryUm(end+1) = segTarget; %#ok<AGROW>
                        lastSuccessDeltaUm = segTarget;
                        metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                        append_metrics_row(metricsPath, segTarget, deltaBaseUm, metrics);
                        try, mphsave(model, checkpointMph); catch, end
                        segLastOk = segTarget;
                        continue;
                    end

                    segOk = false;
                    segErrMsg = ptcErrMsg;
                    break;
                end

                if segOk && abs(segLastOk - preBisectFail) < 1e-12
                    try
                        lastFzPlate = metrics.FzPlate;
                        if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                            contactOnsetDetected = true;
                            if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                                contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                            end
                        end
                    catch
                    end
                    if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                        deltaPlanMode = 'post_onset_micro';
                        switchToMicro = true;
                    end
                    if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= earlyExitTargetUm
                        earlyExit = true;
                        earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                    end
                    bridgeModeUsed = 'SEGMENTED';
                    bridgeSuccess = true;
                    pending(1) = [];
                    if switchToMicro || earlyExit
                        break;
                    end
                    continue;
                else
                    segmentedFailReason = segErrMsg;
                end

                % Bridge Mode 3: TD_RELAX (last fallback, only if not consistent-init)
                if ~is_consistent_init_error(segmentedFailReason) && ~is_consistent_init_error(ptcFailReason)
                    tdAttempted = true;
                    bridgeModeUsed = 'TD_RELAX';
                    bridgeAttemptId = bridgeAttemptId + 1;
                    [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    if ~budgetOk
                        if isfinite(lastSuccessDeltaUm)
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_BUDGET';
                            stopReason = budgetWhy;
                            pending = [];
                            break;
                        end
                        failReason = budgetWhy;
                        error('%s', budgetWhy);
                    end
                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [tdOk, tdElapsed, tdErrMsg] = attempt_td_bridge('TD_RELAX', model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, segLastOk, preBisectFail, bridgeNT, bridgeDt, tdBridgePath, bridgeAttemptId);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, tdElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    fallbackAttempts(end+1) = make_attempt('td_relax', deltaBaseUm, preBisectFail - deltaBaseUm, preBisectFail, tdElapsed, tdOk, tdErrMsg); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    if tdOk
                        bridgeSuccess = true;
                        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                        fprintf(fid, "    td-relax OK from %g to %g um\n", segLastOk, preBisectFail);
                        fclose(fid);

                        hasSol = true;
                        prevDeltaUm = preBisectFail;
                        cntWasActive = (prevDeltaUm >= cntEnableUm);
                        deltaHistoryUm(end+1) = prevDeltaUm; %#ok<AGROW>
                        lastSuccessDeltaUm = prevDeltaUm;
                        bisectLevelsUsed = 0;

                        metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                        append_metrics_row(metricsPath, prevDeltaUm, deltaBaseUm, metrics);
                        try, mphsave(model, checkpointMph); catch, end

                        lastFzPlate = metrics.FzPlate;
                        if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                            contactOnsetDetected = true;
                            if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                                contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                            end
                        end
                        if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                            deltaPlanMode = 'post_onset_micro';
                            switchToMicro = true;
                        end
                        if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= earlyExitTargetUm
                            earlyExit = true;
                            earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                        end

                        tgtIdx = find(abs(deltaTargetsUm - prevDeltaUm) < 1e-12, 1);
                        if ~isempty(tgtIdx)
                            iOut = tgtIdx;
                            targetSolved(iOut) = true;
                            wTop(iOut) = metrics.wTop;
                            wBot(iOut) = metrics.wBot;
                            tnMax(iOut) = metrics.tnMax;
                            Ac(iOut) = metrics.Ac;
                            pnAvg(iOut) = metrics.pnAvg;
                            FzPlate(iOut) = metrics.FzPlate;
                        end
                        pending(1) = [];
                        if switchToMicro || earlyExit
                            break;
                        end
                        continue;
                    else
                        tdFailReason = tdErrMsg;
                        if is_consistent_init_error(tdErrMsg)
                            error('td_relax_consistent_init: %s', tdErrMsg);
                        end
                    end
                else
                    tdFailReason = 'skipped_due_to_consistent_init';
                end

                bridgeFailReason = string_or_none(segmentedFailReason);
                error('bridge_failed: %s', bridgeFailReason);
            end

            if bisectLevelsUsed >= maxBisectLevels
                error('max_bisect_levels exceeded: %d', maxBisectLevels);
            end

            if abs(candIndent - (prevDeltaUm - deltaBaseUm)) < minIndentStepUm
                error('min_indent_step_um reached: %.6g', minIndentStepUm);
            end

            bisectLevelsUsed = bisectLevelsUsed + 1;
            midIndent = (candIndent + (prevDeltaUm - deltaBaseUm)) / 2;
            midUm = deltaBaseUm + midIndent;
            pending = [midUm, pending]; %#ok<AGROW>
            end
            if switchToMicro || earlyExit
                break;
            end
        end
        if switchToMicro || earlyExit
            break;
        end
        end
    end
    if ~earlyExit && strcmp(deltaPlanMode, 'post_onset_micro')
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "  - switch to post_onset_micro targets\n");
        fclose(fid);
        for iM = 1:numel(microTargetsUm)
            targetUm = microTargetsUm(iM);
            if earlyExit
                break;
            end
            % In resume single-target mode, allow a "down-step" sampling point
            % (target <= last_success_from_metrics) instead of skipping it.
            if ~singleTargetMode && targetUm <= lastSuccessDeltaUm + 1e-12
                continue;
            end
            prevDeltaBefore = prevDeltaUm;
            candUm = targetUm;
            if toc(globalT0) > globalTimeoutS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded before post_onset solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                if isfinite(lastSuccessDeltaUm)
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_TIMEOUT';
                    stopReason = failReason;
                    break;
                end
                error('%s', failReason);
            end
            [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            if ~budgetOk
                if isfinite(lastSuccessDeltaUm)
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_BUDGET';
                    stopReason = budgetWhy;
                    break;
                end
                failReason = budgetWhy;
                error('%s', budgetWhy);
            end

            if ~postSkipStationary
                [cntActive, useInit] = prepare_stationary_step(model, solid, st, targetUm, cntEnableUm, cntWasActive, hasSol);
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "  - post_onset try delta=%g um (useinitsol=%s)\n", targetUm, useInit);
                fclose(fid);

                startIso = now_iso();
                liveAttempt = make_live_attempt('STARTED', targetUm, 'stationary', startIso, startIso, 'none', NaN, 'none');
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                t0 = tic;
                ok = true;
                errMsg = '';
                try
                    model.study('std1').run();
                catch ME
                    ok = false;
                    errMsg = string(ME.message);
                end
                elapsed = toc(t0);
                solveTime = solveTime + elapsed;
                perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
                if elapsed > perSolveTimeoutS
                    ok = false;
                    errMsg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', elapsed, perSolveTimeoutS);
                end
                endIso = now_iso();
                liveAttempt = make_live_attempt(tern(ok,'ENDED','FAILED'), targetUm, 'stationary', startIso, endIso, endIso, elapsed, tern(ok,'none',errMsg));
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'stationary', ok, elapsed, errMsg); %#ok<AGROW>
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                if toc(globalT0) > globalTimeoutS && ~ok
                    failDeltaUm = targetUm;
                    failDeltaIndentUm = targetUm - deltaBaseUm;
                    failReason = sprintf('Global timeout exceeded after post_onset solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                    narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                    error('%s', failReason);
                end
                if ~ok && strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS
                    failDeltaUm = targetUm;
                    failDeltaIndentUm = targetUm - deltaBaseUm;
                    failReason = sprintf('post_onset budget exhausted before reaching %.6g', earlyExitTargetUm);
                    narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                    error('%s', failReason);
                end

                if ok
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    post_onset OK at delta=%g um\n", targetUm);
                    fclose(fid);

                    hasSol = true;
                    prevDeltaUm = targetUm;
                    cntWasActive = cntActive;
                    deltaHistoryUm(end+1) = targetUm; %#ok<AGROW>
                    lastSuccessDeltaUm = targetUm;
                    bisectLevelsUsed = 0;

                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, targetUm, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end

                    lastFzPlate = metrics.FzPlate;
                    if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                        contactOnsetDetected = true;
                        if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                            contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                        end
                    end
                    if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < earlyExitTargetUm
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = sprintf('post_onset budget exhausted after successful target delta=%g um (stopping further targets)', targetUm);
                        break;
                    end
                    if toc(globalT0) > globalTimeoutS
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_TIMEOUT';
                        stopReason = sprintf('global_timeout_s exceeded after successful target delta=%g um: %.1fs > %.1fs', targetUm, toc(globalT0), globalTimeoutS);
                        break;
                    end
                    if lastSuccessDeltaUm >= earlyExitTargetUm
                        earlyExit = true;
                        earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                        break;
                    end
                    continue;
                end
            else
                [~, useInit] = prepare_stationary_step(model, solid, st, targetUm, cntEnableUm, cntWasActive, hasSol);
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "  - post_onset try delta=%g um (PTC-first; skip_stationary=on, useinitsol=%s)\n", targetUm, useInit);
                fclose(fid);
            end

            % post_onset: PTC first (or fallback after stationary fail)
            ptcAttempted = true;
            bridgeAttemptId = bridgeAttemptId + 1;
            [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            if ~budgetOk
                if isfinite(lastSuccessDeltaUm)
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_BUDGET';
                    stopReason = budgetWhy;
                    break;
                end
                failReason = budgetWhy;
                error('%s', budgetWhy);
            end
            startIso = now_iso();
            liveAttempt = make_live_attempt('STARTED', targetUm, 'PTC', startIso, startIso, 'none', NaN, 'none');
            flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
            [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, prevDeltaUm, targetUm, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
            endIso = now_iso();
            liveAttempt = make_live_attempt(tern(ptcOk,'ENDED','FAILED'), targetUm, 'PTC', startIso, endIso, endIso, ptcElapsed, tern(ptcOk,'none',ptcErrMsg));
            [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'PTC', ptcOk, ptcElapsed, ptcErrMsg); %#ok<AGROW>
            flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
            if toc(globalT0) > globalTimeoutS && ~ptcOk
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded after post_onset PTC: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if ~ptcOk && strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('post_onset budget exhausted before reaching %.6g', earlyExitTargetUm);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end

            if ptcOk
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "    post_onset PTC OK at delta=%g um\n", targetUm);
                fclose(fid);

                hasSol = true;
                prevDeltaUm = targetUm;
                cntWasActive = (prevDeltaUm >= cntEnableUm);
                deltaHistoryUm(end+1) = targetUm; %#ok<AGROW>
                improved = targetUm >= lastSuccessDeltaUm - 1e-12;
                if improved
                    lastSuccessDeltaUm = targetUm;
                end
                bisectLevelsUsed = 0;

                metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                append_metrics_row(metricsPath, targetUm, deltaBaseUm, metrics);
                % Avoid regressing checkpoint_last_ok.mph when sampling below last_success.
                if improved
                    try, mphsave(model, checkpointMph); catch, end
                else
                    try
                        sampleMph = fullfile(outDir, sprintf('Pyramid_5x5_checkpoint_sample_%0.9gum.mph', targetUm));
                        mphsave(model, sampleMph);
                    catch
                    end
                end

                lastFzPlate = metrics.FzPlate;
                if isfinite(lastFzPlate) && abs(lastFzPlate) > FzEps
                    contactOnsetDetected = true;
                    if ~isfinite(metrics.tnMax) || metrics.Ac <= 0
                        contactNote = 'Force indicates onset but dcnt1 metrics invalid (Ac<=0 or NaN)';
                    end
                end
                if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < earlyExitTargetUm
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_BUDGET';
                    stopReason = sprintf('post_onset budget exhausted after successful target delta=%g um (stopping further targets)', targetUm);
                    break;
                end
                if toc(globalT0) > globalTimeoutS
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_TIMEOUT';
                    stopReason = sprintf('global_timeout_s exceeded after successful target delta=%g um: %.1fs > %.1fs', targetUm, toc(globalT0), globalTimeoutS);
                    break;
                end
                if lastSuccessDeltaUm >= earlyExitTargetUm
                    earlyExit = true;
                    earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                    break;
                end
                continue;
            end

            if ptcOnlyMode
                segmentedAttempted = false;
                tdAttempted = false;
                tdFailReason = tern(disableTdRelax, 'disabled by SIM_DISABLE_TD_RELAX', 'none');
                segmentedFailReason = tern(disableSegmented, 'disabled by SIM_DISABLE_SEGMENTED', 'none');
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = ptcErrMsg;
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('ptc_only_failed at %.6g um: %s', targetUm, ptcErrMsg);
            end

            % Segmented fallback (stationary + per-stage PTC)
            if disableSegmented
                segmentedAttempted = false;
                segmentedFailReason = 'disabled by SIM_DISABLE_SEGMENTED';

                % TD_RELAX last fallback (when segmented is disabled)
                if disableTdRelax
                    tdAttempted = false;
                    tdFailReason = 'disabled by SIM_DISABLE_TD_RELAX';
                elseif ~is_consistent_init_error(segmentedFailReason) && ~is_consistent_init_error(ptcErrMsg)
                    tdAttempted = true;
                    bridgeAttemptId = bridgeAttemptId + 1;
                    [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    if ~budgetOk
                        if isfinite(lastSuccessDeltaUm)
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_BUDGET';
                            stopReason = budgetWhy;
                            break;
                        end
                        failReason = budgetWhy;
                        error('%s', budgetWhy);
                    end
                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = targetUm;
                        failDeltaIndentUm = targetUm - deltaBaseUm;
                        failReason = reserveGateReason;
                        narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                        error('%s', reserveGateReason);
                    end
                    startIso = now_iso();
                    liveAttempt = make_live_attempt('STARTED', targetUm, 'TD_RELAX', startIso, startIso, 'none', NaN, 'none');
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    [tdOk, tdElapsed, tdErrMsg] = attempt_td_bridge('TD_RELAX', model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, prevDeltaUm, targetUm, bridgeNT, bridgeDt, tdBridgePath, bridgeAttemptId);
                    endIso = now_iso();
                    liveAttempt = make_live_attempt(tern(tdOk,'ENDED','FAILED'), targetUm, 'TD_RELAX', startIso, endIso, endIso, tdElapsed, tern(tdOk,'none',tdErrMsg));
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, tdElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'TD_RELAX', tdOk, tdElapsed, tdErrMsg); %#ok<AGROW>
                    flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                    if tdOk
                        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                        fprintf(fid, "    post_onset TD_RELAX OK at delta=%g um\n", targetUm);
                        fclose(fid);
                        hasSol = true;
                        prevDeltaUm = targetUm;
                        cntWasActive = (prevDeltaUm >= cntEnableUm);
                        deltaHistoryUm(end+1) = targetUm; %#ok<AGROW>
                        lastSuccessDeltaUm = targetUm;
                        metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                        append_metrics_row(metricsPath, targetUm, deltaBaseUm, metrics);
                        try, mphsave(model, checkpointMph); catch, end
                        if toc(globalT0) > globalTimeoutS
                            stopRequested = true;
                            exitStatus = 'SUCCESS_BUT_TIMEOUT';
                            stopReason = sprintf('global_timeout_s exceeded after successful target delta=%g um: %.1fs > %.1fs', targetUm, toc(globalT0), globalTimeoutS);
                            break;
                        end
                        if lastSuccessDeltaUm >= earlyExitTargetUm
                            earlyExit = true;
                            earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                            break;
                        end
                        continue;
                    end
                    if is_consistent_init_error(tdErrMsg)
                        error('td_relax_consistent_init at %.6g um: %s', targetUm, tdErrMsg);
                    end
                end

                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = ptcErrMsg;
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('post_onset_micro_failed at %.6g um: %s', targetUm, ptcErrMsg);
            end
            segmentedAttempted = true;
            startIso = now_iso();
            liveAttempt = make_live_attempt('STARTED', targetUm, 'SEGMENTED', startIso, startIso, 'none', NaN, 'none');
            flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
            segElapsedTotal = 0;
            segErrMsg = '';
            segLastOk = prevDeltaUm;
            gapUm = targetUm - prevDeltaUm;
            nSeg = max(2, ceil(gapUm / segmentedMaxStepUmPost));
            nSeg = min(nSeg, segmentedMaxStagesPost);
            segTargets = linspace(prevDeltaUm, targetUm, nSeg + 1);
            segOk = true;
            for s = 2:numel(segTargets)
                segTarget = segTargets(s);
                stageId = s - 1;

                [cntActive, useInit] = prepare_stationary_step(model, solid, st, segTarget, cntEnableUm, cntWasActive, hasSol);
                [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                if ~budgetOk
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = budgetWhy;
                        segOk = false;
                        segErrMsg = budgetWhy;
                        break;
                    end
                    failReason = budgetWhy;
                    error('%s', budgetWhy);
                end
                t0 = tic;
                okSeg = true;
                errSeg = '';
                try
                    model.study('std1').run();
                catch ME
                    okSeg = false;
                    errSeg = string(ME.message);
                end
                elapsedSeg = toc(t0);
                segElapsedTotal = segElapsedTotal + elapsedSeg;
                perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
                if elapsedSeg > perSolveTimeoutS
                    okSeg = false;
                    errSeg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', elapsedSeg, perSolveTimeoutS);
                end
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsedSeg, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                segmentedAttempts(end+1) = make_segment_attempt(stageId, segTarget, 'stat', okSeg, elapsedSeg, errSeg); %#ok<AGROW>
                if okSeg
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    post_onset segmented OK at delta=%g um\n", segTarget);
                    fclose(fid);

                    hasSol = true;
                    prevDeltaUm = segTarget;
                    cntWasActive = cntActive;
                    deltaHistoryUm(end+1) = segTarget; %#ok<AGROW>
                    lastSuccessDeltaUm = segTarget;
                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, segTarget, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end
                    segLastOk = segTarget;
                    continue;
                end

                % PTC fallback for this segment
                ptcAttempted = true;
                bridgeAttemptId = bridgeAttemptId + 1;
                [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                if ~budgetOk
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = budgetWhy;
                        segOk = false;
                        segErrMsg = budgetWhy;
                        break;
                    end
                    failReason = budgetWhy;
                    error('%s', budgetWhy);
                end
                [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, segLastOk, segTarget, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
                segElapsedTotal = segElapsedTotal + ptcElapsed;
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                segmentedAttempts(end+1) = make_segment_attempt(stageId, segTarget, 'ptc', ptcOk, ptcElapsed, ptcErrMsg); %#ok<AGROW>
                if ptcOk
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    post_onset segmented PTC OK at delta=%g um\n", segTarget);
                    fclose(fid);

                    hasSol = true;
                    prevDeltaUm = segTarget;
                    cntWasActive = (prevDeltaUm >= cntEnableUm);
                    deltaHistoryUm(end+1) = segTarget; %#ok<AGROW>
                    lastSuccessDeltaUm = segTarget;
                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, segTarget, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end
                    segLastOk = segTarget;
                    continue;
                end

                segOk = false;
                segErrMsg = ptcErrMsg;
                break;
            end

            endIso = now_iso();
            liveAttempt = make_live_attempt(tern(segOk,'ENDED','FAILED'), targetUm, 'SEGMENTED', startIso, endIso, endIso, segElapsedTotal, tern(segOk,'none',segErrMsg));
            microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'segmented', segOk, segElapsedTotal, segErrMsg); %#ok<AGROW>
            flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
            if toc(globalT0) > globalTimeoutS && ~segOk
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded after post_onset segmented: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < earlyExitTargetUm
                if segOk
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_BUDGET';
                    stopReason = sprintf('post_onset budget exhausted after successful target delta=%g um (stopping further targets)', targetUm);
                    break;
                end
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('post_onset budget exhausted before reaching %.6g', earlyExitTargetUm);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if segOk
                if toc(globalT0) > globalTimeoutS
                    stopRequested = true;
                    exitStatus = 'SUCCESS_BUT_TIMEOUT';
                    stopReason = sprintf('global_timeout_s exceeded after successful target delta=%g um: %.1fs > %.1fs', targetUm, toc(globalT0), globalTimeoutS);
                    break;
                end
                if lastSuccessDeltaUm >= earlyExitTargetUm
                    earlyExit = true;
                    earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                    break;
                end
                continue;
            end

            segmentedFailReason = segErrMsg;

            % TD_RELAX last fallback (avoid consistent-init loops)
            if disableTdRelax
                tdAttempted = false;
                tdFailReason = 'disabled by SIM_DISABLE_TD_RELAX';
            elseif ~is_consistent_init_error(segmentedFailReason) && ~is_consistent_init_error(ptcErrMsg)
                tdAttempted = true;
                bridgeAttemptId = bridgeAttemptId + 1;
                [budgetOk, budgetWhy] = ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                if ~budgetOk
                    if isfinite(lastSuccessDeltaUm)
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_BUDGET';
                        stopReason = budgetWhy;
                        break;
                    end
                    failReason = budgetWhy;
                    error('%s', budgetWhy);
                end
                [reserveGateTriggered, reserveGateReason] = check_reserve_gate(deltaPlanMode, globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                if reserveGateTriggered
                    failDeltaUm = targetUm;
                    failDeltaIndentUm = targetUm - deltaBaseUm;
                    failReason = reserveGateReason;
                    narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                    error('%s', reserveGateReason);
                end
                startIso = now_iso();
                liveAttempt = make_live_attempt('STARTED', targetUm, 'TD_RELAX', startIso, startIso, 'none', NaN, 'none');
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                [tdOk, tdElapsed, tdErrMsg] = attempt_td_bridge('TD_RELAX', model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, prevDeltaUm, targetUm, bridgeNT, bridgeDt, tdBridgePath, bridgeAttemptId);
                endIso = now_iso();
                liveAttempt = make_live_attempt(tern(tdOk,'ENDED','FAILED'), targetUm, 'TD_RELAX', startIso, endIso, endIso, tdElapsed, tern(tdOk,'none',tdErrMsg));
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, tdElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'TD_RELAX', tdOk, tdElapsed, tdErrMsg); %#ok<AGROW>
                flush_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
                if tdOk
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    post_onset TD_RELAX OK at delta=%g um\n", targetUm);
                    fclose(fid);
                    hasSol = true;
                    prevDeltaUm = targetUm;
                    cntWasActive = (prevDeltaUm >= cntEnableUm);
                    deltaHistoryUm(end+1) = targetUm; %#ok<AGROW>
                    lastSuccessDeltaUm = targetUm;
                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, targetUm, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end
                    if toc(globalT0) > globalTimeoutS
                        stopRequested = true;
                        exitStatus = 'SUCCESS_BUT_TIMEOUT';
                        stopReason = sprintf('global_timeout_s exceeded after successful target delta=%g um: %.1fs > %.1fs', targetUm, toc(globalT0), globalTimeoutS);
                        break;
                    end
                    if lastSuccessDeltaUm >= earlyExitTargetUm
                        earlyExit = true;
                        earlyExitReason = sprintf('Reached early-exit target >=%.6g', earlyExitTargetUm);
                        break;
                    end
                    continue;
                end
                if is_consistent_init_error(tdErrMsg)
                    error('td_relax_consistent_init at %.6g um: %s', targetUm, tdErrMsg);
                end
            end
            failDeltaUm = targetUm;
            failDeltaIndentUm = targetUm - deltaBaseUm;
            failReason = segErrMsg;
            narrowFailIntervalUm = [prevDeltaBefore, targetUm];
            error('post_onset_micro_failed at %.6g um: %s', targetUm, segErrMsg);
        end
    end
catch ME
    err = ME;
    if isfinite(candUm)
        failDeltaUm = candUm;
        failDeltaIndentUm = candUm - deltaBaseUm;
    end
    failReason = string(ME.message);

    % Treat time/budget exhaustion as a graceful stop if we already have a valid last_success.
    if isfinite(lastSuccessDeltaUm) && (contains(failReason, "Global timeout exceeded") || contains(lower(failReason), "budget exhausted") || contains(lower(failReason), "time reserve exhausted"))
        stopRequested = true;
        if contains(failReason, "Global timeout exceeded")
            exitStatus = 'SUCCESS_BUT_TIMEOUT';
        else
            exitStatus = 'SUCCESS_BUT_BUDGET';
        end
        stopReason = failReason;
        failDeltaUm = NaN;
        failDeltaIndentUm = NaN;
        failReason = 'none';
        err = [];
    else
        stopReason = failReason;
        try
            write_errors_json(errorsPath, err, deltaBaseUm, failDeltaIndentUm, failDeltaUm, deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
        catch
        end
    end
end

% Export summary (arrays over target deltas)
if isempty(err) && ~stopRequested && isfinite(lastSuccessDeltaUm)
    exitStatus = 'SUCCESS';
end
if isempty(err) && stopRequested && strcmp(exitStatus, 'FAIL') && isfinite(lastSuccessDeltaUm)
    if contains(lower(stopReason), 'budget')
        exitStatus = 'SUCCESS_BUT_BUDGET';
    else
        exitStatus = 'SUCCESS_BUT_TIMEOUT';
    end
end
if startsWith(exitStatus, 'SUCCESS_BUT_')
    failDeltaUm = NaN;
    failDeltaIndentUm = NaN;
    failReason = 'none';
end
fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "\nSolveTime_s_total: %.3f\n", solveTime);
fprintf(fid, "delta_history_um: %s\n", mat2str(deltaHistoryUm));
fprintf(fid, "w_rigid_top_min_(model_unit): %s\n", mat2str(wTop));
fprintf(fid, "w_rigid_bot_min_(model_unit): %s\n", mat2str(wBot));
fprintf(fid, "Tn_max_Pa: %s\n", mat2str(tnMax));
fprintf(fid, "Ac_m2: %s\n", mat2str(Ac));
fprintf(fid, "pn_avg_Pa: %s\n", mat2str(pnAvg));
fprintf(fid, "Fz_plate_top_int_N: %s\n", mat2str(FzPlate));
fprintf(fid, "last_success_delta_total_um: %s\n", num2str(lastSuccessDeltaUm));
fprintf(fid, "last_success_internal_um: %s\n", num2str(lastSuccessDeltaUm));
fprintf(fid, "last_success_from_metrics_um: %s\n", num2str(lastSuccessFromMetricsUm));
fprintf(fid, "checkpoint_source_dir: %s\n", string_or_none(checkpointSourceDir));
fprintf(fid, "fail_delta_total_um: %s\n", num2str(failDeltaUm));
fprintf(fid, "fail_reason: %s\n", string_or_none(failReason));
fprintf(fid, "exit_status: %s\n", string_or_none(exitStatus));
fprintf(fid, "stop_reason: %s\n", string_or_none(stopReason));
fprintf(fid, "bisect_levels_used: %g\n", bisectLevelsUsed);
fprintf(fid, "narrow_fail_interval_um: [%g, %g]\n", narrowFailIntervalUm(1), narrowFailIntervalUm(2));
fprintf(fid, "pre_bisect_last_ok_um: %s\n", num2str(preBisectLastOk));
fprintf(fid, "pre_bisect_fail_um: %s\n", num2str(preBisectFail));
fprintf(fid, "bridge_mode: %s\n", bridgeModeUsed);
fprintf(fid, "bridge_success: %d\n", bridgeSuccess);
fprintf(fid, "bridge_fail_reason: %s\n", string_or_none(bridgeFailReason));
fprintf(fid, "ptc_attempted: %d\n", ptcAttempted);
fprintf(fid, "ptc_fail_reason: %s\n", string_or_none(ptcFailReason));
fprintf(fid, "segmented_attempted: %d\n", segmentedAttempted);
fprintf(fid, "segmented_fail_reason: %s\n", string_or_none(segmentedFailReason));
fprintf(fid, "td_attempted: %d\n", tdAttempted);
fprintf(fid, "td_fail_reason: %s\n", string_or_none(tdFailReason));
fprintf(fid, "delta_plan_mode_final: %s\n", deltaPlanMode);
fprintf(fid, "micro_targets: %s\n", mat2str(microTargetsUm));
fprintf(fid, "Fz_eps: %g\n", FzEps);
fprintf(fid, "contact_onset_detected: %d\n", contactOnsetDetected);
fprintf(fid, "contact_onset_note: %s\n", string_or_none(contactNote));
fprintf(fid, "budget_used_pre_s: %.3f\n", budgetUsedPre);
fprintf(fid, "budget_used_post_s: %.3f\n", budgetUsedPost);
fprintf(fid, "time_elapsed_s: %.3f\n", toc(globalT0));
fprintf(fid, "time_remaining_s: %.3f\n", globalTimeoutS - toc(globalT0));
fprintf(fid, "reserve_gate_triggered: %d\n", reserveGateTriggered);
fprintf(fid, "reserve_gate_reason: %s\n", string_or_none(reserveGateReason));
fprintf(fid, "early_exit: %d\n", earlyExit);
fprintf(fid, "early_exit_reason: %s\n", string_or_none(earlyExitReason));
tnEpsPa = numeric_env('SIM_TN_EPS_PA', 1.0);
[okD, ~, ~, ~, whyD] = try_contact_metrics(model, bnd_eval, tnEpsPa);
fprintf(fid, "contact_field_used: dcnt1\n");
if okD
    fprintf(fid, "contact_field_reason: dcnt1_ok (%s)\n", string_or_none(whyD));
else
    fprintf(fid, "contact_field_reason: dcnt1_unavailable (%s)\n", string_or_none(whyD));
end
fprintf(fid, "dcnt1_only_ok: %d\n", dcntOnlyOk);
fprintf(fid, "dcnt1_only_note: %s\n", string_or_none(dcntOnlyNote));
fclose(fid);

% Ensure baseline_results.csv is written even on failure.
try
    write_baseline_results(baselineCsvPath, deltaBaseUm, deltaIndentUm, deltaTargetsUm, targetSolved, tnMax, Ac, pnAvg, FzPlate, failDeltaUm, failReason);
catch
end

if ~isempty(err)
    try
        write_errors_json(errorsPath, err, deltaBaseUm, failDeltaIndentUm, failDeltaUm, deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
    catch
    end
end
try
    write_fallback_report(fallbackReportPath, liveAttempt, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
catch
end
try
    enteredPostOnset = strcmp(deltaPlanMode, 'post_onset_micro') || ~isempty(microTargetAttempts);
    append_phase2_matrix_row(phase2MatrixPath, contactModeEffective, penaltyFactorMult, nuSupported, enteredPostOnset, lastSuccessDeltaUm, phase1TargetUm, failDeltaUm, failReason, simDir);
catch
end

% Save solved MPH only when fully successful.
if isempty(err)
    mphsave(model, outMph);
end

ModelUtil.disconnect();

% Best-effort cleanup
try
    if ~isempty(proc) && ~proc.HasExited
        proc.WaitForExit(5000);
    end
    if ~isempty(proc) && ~proc.HasExited
        proc.Kill();
        proc.WaitForExit();
    end
catch
end
if ~isempty(err)
    rethrow(err);
end
end

function out = tern(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function s = now_iso()
s = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
end

function attempt = make_live_attempt(status, targetUm, attemptMode, startIso, heartbeatIso, endIso, elapsedS, failReason)
attempt = struct();
attempt.status = string(status);
attempt.target_delta_total_um = targetUm;
attempt.attempt_mode = string(attemptMode);
attempt.start_time_iso = string(startIso);
attempt.heartbeat_time_iso = string(heartbeatIso);
attempt.end_time_iso = string(endIso);
attempt.elapsed_s = elapsedS;
attempt.fail_reason = string(failReason);
end

function vals = parse_num_list(s)
% Parse a numeric list from an env-var string, accepting "1.02,1.03" or "1.02 1.03" or "[1.02 1.03]".
if nargin < 1
    vals = [];
    return;
end
txt = strtrim(string(s));
txt = erase(txt, "[");
txt = erase(txt, "]");
txt = replace(txt, ",", " ");
vals = sscanf(char(txt), '%f');
end

function finalize_run_files(summaryPath, metricsPath, errorsPath, fallbackReportPath)
try
    if ~exist(summaryPath, 'file')
        lastSuccess = NaN;
        if exist(metricsPath, 'file')
            try
                lines = string(readlines(metricsPath));
                lines = lines(lines ~= "");
                if numel(lines) > 1
                    lastLine = lines(end);
                    parts = strsplit(lastLine, ',');
                    if numel(parts) >= 2
                        lastSuccess = str2double(parts{2});
                    end
                end
            catch
            end
        end
        fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
        fprintf(fid, "summary_note: summary missing; run aborted before finalize\n");
        fprintf(fid, "last_success_delta_total_um: %g\n", lastSuccess);
        fprintf(fid, "fail_reason: summary missing; run aborted before finalize\n");
        fclose(fid);
    end
    if ~exist(fallbackReportPath, 'file')
        payload = struct();
        payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
        payload.note = 'fallback_report missing; run aborted before finalize';
        txt = jsonencode(payload);
        fid = fopen(fallbackReportPath, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', txt);
        fclose(fid);
    end
    if ~exist(errorsPath, 'file')
        shouldWrite = true;
        try
            lines = string(readlines(summaryPath));
            lines = lines(lines ~= "");
            ex = lines(startsWith(lines, "exit_status:"));
            if ~isempty(ex)
                exv = lower(strtrim(extractAfter(ex(1), "exit_status:")));
                if contains(exv, "success")
                    shouldWrite = false;
                end
            end
            fr = lines(startsWith(lines, "fail_reason:"));
            if ~isempty(fr)
                if contains(lower(fr(1)), "none")
                    shouldWrite = false;
                end
            else
                shouldWrite = true;
            end
            if any(contains(lines, "summary missing; run aborted before finalize"))
                shouldWrite = true;
            end
        catch
            shouldWrite = true;
        end
        if shouldWrite
            payload = struct();
            payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
            payload.message = 'errors.json missing; run failed or aborted before errors capture';
            txt = jsonencode(payload);
            fid = fopen(errorsPath, 'w', 'n', 'UTF-8');
            fprintf(fid, '%s', txt);
            fclose(fid);
        end
    end
catch
end
end

function metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval)
metrics = struct('wTop', NaN, 'wBot', NaN, 'tnMax', NaN, 'Ac', 0, 'pnAvg', NaN, 'FzPlate', NaN);
try, metrics.wTop = mphmin(model, 'w', 'surface', 'selection', bnd_rigid_top); catch, end
try, metrics.wBot = mphmin(model, 'w', 'surface', 'selection', bnd_rigid_bot); catch, end
tnEpsPa = numeric_env('SIM_TN_EPS_PA', 1.0);
[metrics.tnMax, metrics.Ac, metrics.pnAvg] = eval_contact_metrics(model, bnd_eval, tnEpsPa);
try, metrics.FzPlate = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_rigid_top); catch, end
end

function [tnMax, Ac, pnAvg] = eval_contact_metrics(model, bnd_eval, tnEpsPa)
% Contact postprocessing (dcnt1-only):
% In this COMSOL setup, SolidContact (dcnt1) exposes unprefixed fields:
%   solid.Tn, solid.incontact, solid.gap
% (solid.dcnt1.* is often undefined even when the feature tag is dcnt1.)
%
% Priority for area:
% 1) solid.incontact (preferred)
% 2) solid.Tn threshold
% 3) solid.gap < 0
tnMax = NaN;
Ac = 0;
pnAvg = NaN;
if nargin < 3 || ~isfinite(tnEpsPa)
    tnEpsPa = 1.0;
end

[okD, tnD, acD, pnD] = try_contact_metrics(model, bnd_eval, tnEpsPa);
if okD
    tnMax = tnD; Ac = acD; pnAvg = pnD;
end
end

function [ok, tnMax, Ac, pnAvg, why] = try_contact_metrics(model, bnd_eval, tnEpsPa)
ok = false;
tnMax = NaN;
Ac = 0;
pnAvg = NaN;
why = 'none';
try
    tnVar = 'solid.Tn';
    incontactVar = 'solid.incontact';
    gapVar = 'solid.gap';

    tnMax = mphmax(model, tnVar, 'surface', 'selection', bnd_eval);

    % Prefer incontact if available, else use Tn threshold, else gap<0.
    try
        Ac = mphint2(model, sprintf('if(%s>0.5,1,0)', incontactVar), 'surface', 'selection', bnd_eval);
        why = 'incontact';
    catch
        try
            Ac = mphint2(model, sprintf('if(%s>%g,1,0)', tnVar, tnEpsPa), 'surface', 'selection', bnd_eval);
            why = sprintf('Tn>%gPa', tnEpsPa);
        catch
            Ac = mphint2(model, sprintf('if(%s<0,1,0)', gapVar), 'surface', 'selection', bnd_eval);
            why = 'gap<0';
        end
    end

    pnInt = mphint2(model, tnVar, 'surface', 'selection', bnd_eval);
    if isfinite(Ac) && Ac > 0
        pnAvg = pnInt ./ Ac;
    else
        Ac = 0;
        pnAvg = NaN;
    end
    ok = true;
catch
    ok = false;
    tnMax = NaN;
    Ac = 0;
    pnAvg = NaN;
    why = 'exception';
end
end

function write_metrics_header(path)
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'delta_indent_um,delta_total_um,w_top_min_modelunit,w_bot_min_modelunit,Tn_max_Pa,Ac_m2,pn_avg_Pa,Fz_plate_top_int_N\n');
fclose(fid);
end

function append_metrics_row(path, deltaUm, deltaBaseUm, metrics)
deltaIndent = deltaUm - deltaBaseUm;
fid = fopen(path, 'a', 'n', 'UTF-8');
fprintf(fid, '%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', ...
    deltaIndent, deltaUm, metrics.wTop, metrics.wBot, metrics.tnMax, metrics.Ac, metrics.pnAvg, metrics.FzPlate);
fclose(fid);
end

function write_baseline_results(path, deltaBaseUm, deltaIndentUm, deltaTargetsUm, targetSolved, tnMax, Ac, pnAvg, FzPlate, failDeltaUm, failReason)
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'delta_base,delta_indent,delta_total,success,Tn_max,Ac,pn_avg,Fz_plate,message\n');
for i = 1:numel(deltaIndentUm)
    if targetSolved(i)
        success = 1;
        msg = "ok";
    else
        success = 0;
        msg = "failed_before_target; fail_delta_total_um=" + num2str(failDeltaUm) + "; reason=" + string_or_none(failReason);
    end
    msg = sanitize_csv_text(msg);
    fprintf(fid, '%.6g,%.6g,%.6g,%d,%.6g,%.6g,%.6g,%.6g,%s\n', ...
        deltaBaseUm, deltaIndentUm(i), deltaTargetsUm(i), success, tnMax(i), Ac(i), pnAvg(i), FzPlate(i), msg);
end
fclose(fid);
end

function write_errors_json(path, err, deltaBaseUm, deltaIndentUm, deltaTotalUm, deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, timeElapsedS, timeRemainingS)
stack = [];
try
    if isfield(err, 'stack') && ~isempty(err.stack)
        stack = arrayfun(@(s) struct('file', s.file, 'name', s.name, 'line', s.line), err.stack);
    end
catch
    stack = [];
end
payload = struct();
payload.delta_base_um = deltaBaseUm;
payload.delta_indent_um = deltaIndentUm;
payload.delta_total_um = deltaTotalUm;
payload.delta_plan_mode = deltaPlanMode;
payload.budget_used_pre_s = budgetUsedPre;
payload.budget_used_post_s = budgetUsedPost;
payload.budget_pre_onset_s = budgetPreOnsetS;
payload.budget_post_onset_micro_s = budgetPostOnsetS;
payload.reserve_gate_triggered = reserveGateTriggered;
payload.time_elapsed_s = timeElapsedS;
payload.time_remaining_s = timeRemainingS;
payload.message = string(err.message);
payload.stack = stack;
payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
txt = jsonencode(payload);
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', txt);
fclose(fid);
end

function out = string_or_none(val)
out = "none";
try
    if isstring(val) || ischar(val)
        if strlength(string(val)) > 0
            out = string(val);
        end
    end
catch
end
end

function out = sanitize_csv_text(val)
out = string(val);
out = replace(out, [",", newline, char(13)], ";");
out = regexprep(out, '[^ -~]', '');
out = char(out);
end

function out = join_list(list)
out = '';
try
    if isstring(list)
        list = cellstr(list);
    end
    if isempty(list)
        out = '';
        return;
    end
    if iscell(list)
        out = strjoin(cellfun(@(s) char(string(s)), list, 'UniformOutput', false), ', ');
    else
        out = char(string(list));
    end
catch
    out = '';
end
end

function out = nu_supported_reason(nuNote)
out = "nu detection failed";
try
    if strlength(string(nuNote)) == 0 || string(nuNote) == "none"
        out = "nu detection failed";
        return;
    end
    if contains(lower(string(nuNote)), "no nu")
        out = "model has no nu parameter";
    else
        out = "nu detection failed: " + string(nuNote);
    end
catch
end
end

function append_phase2_matrix_row(path, contactMode, penaltyFactorMult, nuSupported, enteredPostOnset, lastSuccess, targetUm, failDelta, failReason, outDir)
header = 'timestamp,contact_mode,penalty_factor_mult,nu_supported,entered_post_onset_micro,last_success,fail_delta,fail_reason,out_dir';
if exist(path, 'file')
    try
        fid = fopen(path, 'r', 'n', 'UTF-8');
        firstLine = fgetl(fid);
        fclose(fid);
        if ~ischar(firstLine) || ~strcmp(strtrim(firstLine), header)
            legacy = fullfile(fileparts(path), ['phase2_matrix_legacy_' datestr(now, 'yyyymmdd_HHMMSS') '.csv']);
            try, movefile(path, legacy); catch, end
        end
    catch
    end
end
if ~exist(path, 'file')
    fid = fopen(path, 'w', 'n', 'UTF-8');
    fprintf(fid, '%s\n', header);
    fclose(fid);
end
timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
fid = fopen(path, 'a', 'n', 'UTF-8');
msg = sanitize_csv_text(string_or_none(failReason));
fprintf(fid, '%s,%s,%.6g,%d,%d,%.6g,%.6g,%s,%s\n', ...
    timestamp, contactMode, penaltyFactorMult, tern(nuSupported,1,0), tern(enteredPostOnset,1,0), lastSuccess, failDelta, msg, sanitize_csv_text(outDir));
fclose(fid);
end

function val = get_env_or_default(name, defaultVal)
val = getenv(name);
if isempty(val)
    val = defaultVal;
end
val = strtrim(val);
if isempty(val)
    val = defaultVal;
end
end

function out = get_env_bool(name, defaultVal)
val = getenv(name);
if isempty(val)
    out = defaultVal;
    return;
end
val = lower(strtrim(val));
if any(strcmp(val, {'1','true','yes','y','on'}))
    out = true;
elseif any(strcmp(val, {'0','false','no','n','off'}))
    out = false;
else
    out = defaultVal;
end
end

function out = numeric_env(name, defaultVal)
val = getenv(name);
if isempty(val)
    out = defaultVal;
    return;
end
v = str2double(strtrim(val));
if isfinite(v)
    out = v;
else
    out = defaultVal;
end
end

function p = resolve_path(baseDir, p0)
p = strtrim(string(p0));
if p == ""
    p = "";
    return;
end
pp = char(p);
if isfolder(pp) || isfile(pp)
    p = pp;
    return;
end
try
    p = fullfile(baseDir, pp);
catch
    p = pp;
end
end

function initialize_metrics_file(metricsPath, resumeMetricsPath, resumeEnabled)
if resumeEnabled && ~isempty(resumeMetricsPath) && exist(resumeMetricsPath, 'file')
    try
        if ~exist(metricsPath, 'file')
            copyfile(resumeMetricsPath, metricsPath, 'f');
        end
    catch
    end
end
if ~exist(metricsPath, 'file')
    write_metrics_header(metricsPath);
end
end

function [lastSuccessUm, lastFzPlateN] = read_last_metrics_delta(metricsPath)
lastSuccessUm = NaN;
lastFzPlateN = NaN;
if ~exist(metricsPath, 'file')
    return;
end
lines = string(readlines(metricsPath));
lines = lines(lines ~= "");
if numel(lines) < 2
    return;
end
% Use the maximum delta_total_um as "last_success" (more robust than the last row,
% and allows appending non-monotone sampling points for RP).
bestDelta = -Inf;
bestFz = NaN;
for i = 2:numel(lines)
    parts = strsplit(char(lines(i)), ',');
    if numel(parts) < 2
        continue;
    end
    d = str2double(parts{2});
    if ~isfinite(d)
        continue;
    end
    if d > bestDelta
        bestDelta = d;
        if numel(parts) >= 8
            bestFz = str2double(parts{8});
        else
            bestFz = NaN;
        end
    end
end
if isfinite(bestDelta)
    lastSuccessUm = bestDelta;
    lastFzPlateN = bestFz;
end
end

function [ok, note] = enforce_dcnt1_only(solid, dcntTag, cntTag, pairTag)
% Enforce a single SolidContact feature (dcnt1) as the only contact implementation.
% Best-effort across COMSOL versions: try remove obsolete Contact (cnt1), else deactivate it.
% Also bind dcnt1 explicitly to the contact pair tag (e.g. 'pc').
ok = false;
note = 'none';

% Disable/remove obsolete cnt1.
try
    solid.feature(cntTag);
    hasCnt = true;
catch
    hasCnt = false;
end
if hasCnt
    removed = false;
    try
        solid.feature.remove(cntTag);
        removed = true;
        note = 'cnt1_removed';
    catch
    end
    if ~removed
        try
            solid.feature(cntTag).active(false);
            note = 'cnt1_deactivated';
        catch ME
            note = "cnt1_disable_failed: " + string(ME.message);
        end
    end
end

% Ensure dcnt exists and is active.
try
    dcnt = solid.feature(dcntTag);
catch ME
    note = "missing_" + string(dcntTag) + ": " + string(ME.message);
    return;
end
try, dcnt.active(true); catch, end

% Bind explicitly to pair.
pairOk = bind_pair_to_feature(dcnt, pairTag);
ok = pairOk;
if ~pairOk
    note = "dcnt_pair_bind_failed(" + string(pairTag) + ")";
end
end

function ok = bind_pair_to_feature(feat, pairTag)
% Try several key spellings used across COMSOL feature types/versions.
ok = false;
try, feat.set('pairSelection', 'list'); catch, end
try, feat.set('pairselection', 'list'); catch, end
try
    feat.set('pairs', {pairTag});
catch
end
try
    arr = javaArray('java.lang.String', 1);
    arr(1) = java.lang.String(pairTag);
    feat.set('pairs', arr);
catch
end
try
    feat.set('pair', pairTag);
catch
end
try
    feat.set('pairname', pairTag);
catch
end

% Verify binding actually took effect.
ok = is_pair_bound(feat, pairTag);
end

function ok = is_pair_bound(feat, pairTag)
ok = false;
try
    arr = feat.getStringArray('pairs');
    c = cell(arr);
    c = cellfun(@char, c, 'UniformOutput', false);
    if any(strcmp(c, pairTag))
        ok = true;
        return;
    end
catch
end
try
    v = char(feat.getString('pairs'));
    if strcmp(strtrim(v), pairTag)
        ok = true;
        return;
    end
catch
end
end

function [switched, note] = configure_linear_solver_mode(model, mode)
% Best-effort linear solver switch for this run (does not change physics).
%
% Supported modes:
% - 'default' (no changes)
% - 'direct_pardiso' (force Direct solver nodes to use PARDISO where available)
switched = false;
note = 'none';
mm = lower(strtrim(string(mode)));
if mm == "" || mm == "default" || mm == "none"
    return;
end
if mm ~= "direct_pardiso"
    note = "unknown linear solver mode";
    return;
end

% Fast path for this model: sol1/s1/dDef is the default Direct node (often MUMPS).
try
    dDef = model.sol('sol1').feature('s1').feature('dDef');
    try
        dDef.set('linsolver', 'pardiso');
        try
            v = char(dDef.getString('linsolver'));
            if strcmpi(strtrim(v), 'pardiso')
                switched = true;
                note = 'sol1/s1/dDef linsolver=pardiso';
                return;
            end
        catch
            switched = true;
            note = 'sol1/s1/dDef linsolver set to pardiso (unverified)';
            return;
        end
    catch
    end
catch
end

try
    soltags = cell(model.sol.tags);
catch
    soltags = {};
end

setCount = 0;
for i = 1:numel(soltags)
    try
        sol = model.sol(soltags{i});
        ftags = cell(sol.feature.tags);
    catch
        continue;
    end
    for j = 1:numel(ftags)
        try
            top = sol.feature(ftags{j});
        catch
            continue;
        end
        setCount = setCount + set_direct_solver_pardiso(top);
        try
            subtags = cell(top.feature.tags);
        catch
            subtags = {};
        end
        for k = 1:numel(subtags)
            try
                sub = top.feature(subtags{k});
                setCount = setCount + set_direct_solver_pardiso(sub);
            catch
            end
        end
    end
end

if setCount > 0
    switched = true;
    note = sprintf('set PARDISO on %d Direct solver node(s)', setCount);
else
    switched = false;
    note = 'no Direct solver nodes were updated (PARDISO not available?)';
end
end

function n = set_direct_solver_pardiso(feat)
n = 0;
try
    t = char(feat.getType());
catch
    t = '';
end
if ~strcmpi(t, 'Direct')
    return;
end
try
    feat.set('linsolver', 'pardiso');
    n = 1;
catch
end
end

function [supported, effective, note] = apply_contact_mode(cnt, featureTag, contactMode, penaltyFactorMult, contactTolScale)
supported = true;
effective = contactMode;
note = 'none';
mode = lower(strtrim(contactMode));
try
    switch mode
        case 'penalty_soft'
            cnt.set('ContactMethodCtrl', 'Penalty');
            cnt.set('penaltyCtrl', 'userDefined');
            baseExpr = sprintf('0.01*solid.%s.E_char/solid.hmin_dst', featureTag);
            expr = sprintf('%g*(%s)', penaltyFactorMult, baseExpr);
            cnt.set('pn_penalty', expr);
        case 'penalty_auto'
            cnt.set('ContactMethodCtrl', 'Penalty');
            ok = try_set_penalty_ctrl(cnt, {'automatic','auto','default'});
            if ~ok
                supported = false;
                note = 'not supported: penaltyCtrl auto not available';
            end
        case 'augmented_lagrange'
            ok = try_set_contact_method(cnt, {'AugmentedLagrange','augmentedLagrange','Augmented Lagrange'});
            if ~ok
                supported = false;
                note = 'not supported: augmented lagrange not available';
            end
        otherwise
            supported = false;
            note = 'unknown contact_mode';
    end
    if contactTolScale ~= 1
        try, cnt.set('ContactTolType', 'Manual'); catch, end
        try, cnt.set('tolcontact', sprintf('%g', 1e-6 * contactTolScale)); catch, end
    end
catch ME
    supported = false;
    note = string(ME.message);
end
end

function ok = try_set_contact_method(cnt, candidates)
ok = false;
for i = 1:numel(candidates)
    try
        try
            cnt.set('ContactMethodCtrl', candidates{i});
            ok = true;
            return;
        catch
        end
        try
            cnt.set('method', candidates{i});
            ok = true;
            return;
        catch
        end
    catch
    end
end
end

function ok = try_set_penalty_ctrl(cnt, candidates)
ok = false;
for i = 1:numel(candidates)
    try
        cnt.set('penaltyCtrl', candidates{i});
        ok = true;
        return;
    catch
    end
end
end

function [supported, paramName, nuValue, note] = apply_nu_mode(model, nuMode)
supported = false;
paramName = '';
nuValue = NaN;
note = 'none';
candidates = {'nu','nu_mat','nu_s','nu_p','poisson','poissons'};
for i = 1:numel(candidates)
    try
        model.param.evaluate(candidates{i});
        paramName = candidates{i};
        supported = true;
        break;
    catch
    end
end
if ~supported
    note = 'not supported: no nu parameter';
    return;
end
targetVal = 0.49;
mode = lower(strtrim(nuMode));
if strcmp(mode, 'easy')
    targetVal = 0.48;
elseif strcmp(mode, 'prod')
    targetVal = 0.49;
else
    note = 'unknown nu_mode';
end
try
    model.param.set(paramName, sprintf('%.4g', targetVal));
catch ME
    supported = false;
    note = string(ME.message);
    return;
end
try
    nuValue = model.param.evaluate(paramName);
catch
    nuValue = targetVal;
end
end

function [tags, pdmsTag, propKeys, note] = collect_material_diagnostics(model)
tags = {};
pdmsTag = '';
propKeys = {};
note = 'none';
try
    comp = model.component('comp1');
    try
        raw = comp.material.tags;
        tags = cell(raw);
        tags = cellfun(@char, tags, 'UniformOutput', false);
    catch
        tags = {};
    end
    if ~isempty(tags)
        for i = 1:numel(tags)
            try
                mat = comp.material(tags{i});
                label = '';
                try, label = char(mat.label); catch, end
                if contains(lower(tags{i}), 'pdms') || contains(lower(label), 'pdms')
                    pdmsTag = tags{i};
                    break;
                end
            catch
            end
        end
        if isempty(pdmsTag)
            pdmsTag = tags{1};
        end
    end
    if ~isempty(pdmsTag)
        try
            mat = comp.material(pdmsTag);
            pg = mat.propertyGroup('def');
            try
                rawKeys = pg.getKeys;
                propKeys = cell(rawKeys);
                propKeys = cellfun(@char, propKeys, 'UniformOutput', false);
            catch
                propKeys = {};
            end
        catch ME
            note = string(ME.message);
        end
    end
catch ME
    note = string(ME.message);
end
end

function attempt = make_attempt(method, deltaBaseUm, deltaIndentUm, deltaTotalUm, elapsedS, ok, errMsg)
attempt = struct();
attempt.method = method;
attempt.delta_base_um = deltaBaseUm;
attempt.delta_indent_um = deltaIndentUm;
attempt.delta_total_um = deltaTotalUm;
attempt.elapsed_s = elapsedS;
attempt.outcome = tern(ok, 'success', 'fail');
attempt.error_summary = string_or_none(errMsg);
end

function out = is_consistent_init_error(msg)
out = false;
try
    s = lower(string(msg));
    if contains(s, "consistent") || contains(s, "一致") || contains(s, "initial")
        out = true;
    end
catch
end
end

function [ok, elapsed, errMsg] = run_td_with_init(model, stdTag, mode, useConsistent)
ok = false;
elapsed = 0;
errMsg = '';
try
    td = model.study(stdTag).feature('time');
    try, td.set('useConsistentInitialization', tern(useConsistent,'on','off')); catch, end
    try, td.set('useinitsol', 'on'); catch, end
    try, td.set('initmethod', 'sol'); catch, end
    try, td.set('solnum', 'last'); catch, end
    configure_td_solver(model, useConsistent);
    t0 = tic;
    model.study(stdTag).run();
    elapsed = toc(t0);
    ok = true;
catch ME
    errMsg = string(ME.message);
end

function configure_td_solver(model, useConsistent)
tdMaxIter = numeric_env('SIM_TD_MAXITER', 10);
tdMaxSteps = numeric_env('SIM_TD_MAXSTEPS', 200);
try
    soltags = cell(model.sol.tags);
catch
    return;
end
for i = 1:numel(soltags)
    try
        sol = model.sol(soltags{i});
        ftags = cell(sol.feature.tags);
    catch
        continue;
    end
    for j = 1:numel(ftags)
        try
            feat = sol.feature(ftags{j});
            try, feat.set('consistent', tern(useConsistent,'on','off')); catch, end
            try, feat.set('useConsistentInitialization', tern(useConsistent,'on','off')); catch, end
            try, feat.set('consistentinit', tern(useConsistent,'on','off')); catch, end
            try, feat.set('initmethod', 'sol'); catch, end
            try, feat.set('useinitsol', 'on'); catch, end
            try, feat.set('solnum', 'last'); catch, end
            try, feat.set('timestepping', 'bdf'); catch, end
            try, feat.set('maxorder', '1'); catch, end

            % Hard caps to avoid TD_RELAX "grinding" forever (best-effort across COMSOL versions).
            try, feat.set('maxiter', tdMaxIter); catch, end
            try, feat.set('maxit', tdMaxIter); catch, end
            try, feat.set('maxniter', tdMaxIter); catch, end
            try, feat.set('maxsteps', tdMaxSteps); catch, end
            try, feat.set('maxnsteps', tdMaxSteps); catch, end
            try, feat.set('maxnumsteps', tdMaxSteps); catch, end
        catch
        end
    end
end
end
end

function pre = make_pre_bisect_attempt(deltaTotalUm, ok, elapsedS, errMsg)
pre = struct();
pre.delta_total_um = deltaTotalUm;
pre.success = ok;
pre.elapsed_s = elapsedS;
pre.error_summary = string_or_none(errMsg);
end

function seg = make_segment_attempt(stageId, targetUm, method, ok, elapsedS, errMsg)
seg = struct();
seg.stage_id = stageId;
seg.target_um = targetUm;
seg.method = method;
seg.success = ok;
seg.elapsed_s = elapsedS;
seg.error_summary = string_or_none(errMsg);
end

function micro = make_micro_attempt(targetUm, order, ok, elapsedS, errMsg)
micro = struct();
micro.target_delta_total_um = targetUm;
micro.attempt_order = order;
micro.success = ok;
micro.elapsed_s = elapsedS;
micro.error_summary = string_or_none(errMsg);
end

function hash = get_git_hash()
hash = 'unknown';
try
    [st, out] = system('git rev-parse --short HEAD');
    if st == 0
        hash = strtrim(out);
    end
catch
end
hash = regexprep(hash, '[^ -~]', '');
if isempty(hash)
    hash = 'unknown';
end
end

function [usedPre, usedPost] = consume_budget(mode, elapsedS, usedPre, usedPost, budgetPre, budgetPost)
if elapsedS <= 0
    return;
end
if strcmp(mode, 'post_onset_micro')
    usedPost = usedPost + elapsedS;
else
    usedPre = usedPre + elapsedS;
end
end

function [ok, reason] = ensure_budget(mode, usedPre, usedPost, budgetPre, budgetPost)
ok = true;
reason = 'none';
if strcmp(mode, 'post_onset_micro')
    if usedPost >= budgetPost
        ok = false;
        reason = sprintf('post_onset budget exhausted (used_post_s=%.1f >= budget_post_s=%.1f)', usedPost, budgetPost);
    end
else
    if usedPre >= budgetPre
        ok = false;
        reason = sprintf('pre_onset budget exhausted (used_pre_s=%.1f >= budget_pre_s=%.1f)', usedPre, budgetPre);
    end
end
end

function [cntActive, useInit] = prepare_stationary_step(model, solid, st, candUm, cntEnableUm, cntWasActive, hasSol)
useInit = tern(hasSol,'on','off');
try, st.set('useinitsol', useInit); catch, end
try, solid.feature('dcnt1').active(false); catch, end
try, model.param.set('delta', sprintf('%g[um]', candUm)); catch, end
cntActive = (candUm >= cntEnableUm);
try, solid.feature('dcnt1').active(cntActive); catch, end
if cntActive && ~cntWasActive
    useInit = 'off';
end
try, st.set('useinitsol', useInit); catch, end
end

function perSolveTimeoutS = get_per_solve_timeout(mode, perSolveTimeoutPreS, perSolveTimeoutPostS)
if strcmp(mode, 'post_onset_micro')
    perSolveTimeoutS = perSolveTimeoutPostS;
else
    perSolveTimeoutS = perSolveTimeoutPreS;
end
end

function [triggered, reason] = check_reserve_gate(mode, globalT0, globalTimeoutS, budgetPostOnsetS, triggered, reason)
% Reserve gate protects post-onset budget by stopping *pre-onset* retries when
% the remaining global time cannot cover the post-onset budget allocation.
if strcmp(mode, 'post_onset_micro')
    return;
end
timeRemaining = globalTimeoutS - toc(globalT0);
if timeRemaining < budgetPostOnsetS
    triggered = true;
    reason = 'pre_onset time reserve exhausted (preserving post budget)';
end
end

function write_fallback_report(path, liveAttempt, attempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, timeElapsedS, timeRemainingS)
payload = struct();
payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
payload.status = string_or_none(liveAttempt.status);
payload.target_delta_total_um = liveAttempt.target_delta_total_um;
payload.attempt_mode = string_or_none(liveAttempt.attempt_mode);
payload.start_time_iso = string_or_none(liveAttempt.start_time_iso);
payload.heartbeat_time_iso = string_or_none(liveAttempt.heartbeat_time_iso);
payload.end_time_iso = string_or_none(liveAttempt.end_time_iso);
payload.elapsed_s = liveAttempt.elapsed_s;
payload.fail_reason = string_or_none(liveAttempt.fail_reason);
payload.live_attempt = liveAttempt;
payload.attempts = attempts;
payload.pre_bisect_attempts = preBisectAttempts;
payload.pre_bisect_last_ok_um = preBisectLastOk;
payload.pre_bisect_fail_um = preBisectFail;
payload.segmented_attempts = segmentedAttempts;
payload.micro_target_attempts = microTargetAttempts;
payload.bridge_policy = bridgePolicy;
payload.delta_plan_mode = deltaPlanMode;
payload.micro_targets = microTargetsUm;
payload.post_skip_stationary = get_env_bool('SIM_POST_SKIP_STATIONARY', false);
payload.ptc_first = payload.post_skip_stationary;
payload.disable_segmented = get_env_bool('SIM_DISABLE_SEGMENTED', false);
payload.disable_td_relax = get_env_bool('SIM_DISABLE_TD_RELAX', false);
try
    resumeFlag = ~isempty(strtrim(getenv('RESUME_POST_ONSET_ONLY'))) && any(strcmpi(strtrim(getenv('RESUME_POST_ONSET_ONLY')), {'1','true','on'}));
catch
    resumeFlag = false;
end
payload.ptc_only_mode = payload.disable_segmented && payload.disable_td_relax && resumeFlag && ~isempty(strtrim(getenv('SIM_SINGLE_TARGET_UM')));
payload.segmented_attempted = ~isempty(segmentedAttempts);
try
    tdOrders = {microTargetAttempts.attempt_order};
catch
    tdOrders = {};
end
payload.td_attempted = any(strcmpi(tdOrders, 'TD_RELAX')) || any(strcmpi({attempts.method}, 'td_relax'));
payload.budget_used_pre_s = budgetUsedPre;
payload.budget_used_post_s = budgetUsedPost;
payload.budget_pre_onset_s = budgetPreOnsetS;
payload.budget_post_onset_micro_s = budgetPostOnsetS;
payload.reserve_gate_triggered = reserveGateTriggered;
payload.time_elapsed_s = timeElapsedS;
payload.time_remaining_s = timeRemainingS;
txt = jsonencode(payload);
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', txt);
fclose(fid);
end

function flush_fallback_report(path, liveAttempt, attempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, timeElapsedS, timeRemainingS)
try
    write_fallback_report(path, liveAttempt, attempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, timeElapsedS, timeRemainingS);
catch
end
end

function [ok, elapsed, errMsg] = attempt_td_bridge(mode, model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, deltaStartUm, deltaEndUm, bridgeNT, bridgeDt, tdBridgePath, attemptId)
ok = false;
elapsed = 0;
errMsg = '';
ensure_td_bridge_header(tdBridgePath);

prevDeltaExpr = '';
try, prevDeltaExpr = char(model.param.get('delta')); catch, end

try
    stdTag = 'std_td';
    try
        model.study(stdTag);
        hasTd = true;
    catch
        hasTd = false;
    end
    if ~hasTd
        model.study.create(stdTag);
        model.study(stdTag).create('time', 'Transient');
    end
    td = model.study(stdTag).feature('time');
    if bridgeNT < 2
        bridgeNT = 2;
    end
    if bridgeDt <= 0
        bridgeDt = 1 / (bridgeNT - 1);
    end
    td.set('tlist', sprintf('range(0,%g,1)', bridgeDt));
    try, td.set('useinitsol', 'on'); catch, end
    try, td.set('initmethod', 'sol'); catch, end
    try, td.set('solnum', 'last'); catch, end
    try, td.set('timestepping', 'bdf'); catch, end
    try, td.set('maxorder', '1'); catch, end
    try, td.set('useConsistentInitialization', 'on'); catch, end

    if strcmpi(mode, 'TD_RELAX')
        model.param.set('delta', sprintf('%g[um]', deltaEndUm));
    else
        deltaExpr = sprintf('%g[um] + (%g[um]-%g[um])*(0.5*(1-cos(pi*t)))', deltaStartUm, deltaEndUm, deltaStartUm);
        model.param.set('delta', deltaExpr);
    end

    [ok, elapsed, errMsg] = run_td_with_init(model, stdTag, mode, true);
    if ~ok && strcmpi(mode, 'TD_RELAX') && is_consistent_init_error(errMsg)
        [ok, elapsed, errMsg] = run_td_with_init(model, stdTag, mode, false);
    end

    if ok
        write_td_bridge_results(tdBridgePath, attemptId, mode, deltaStartUm, deltaEndUm, bridgeDt, bnd_eval, bnd_rigid_top, model);
    else
        append_td_bridge_failure(tdBridgePath, attemptId, mode, deltaEndUm);
    end
catch ME
    errMsg = string(ME.message);
    append_td_bridge_failure(tdBridgePath, attemptId, mode, deltaEndUm);
end

try
    model.param.set('delta', sprintf('%g[um]', deltaEndUm));
catch
end

if ~ok && strlength(string(errMsg)) == 0
    errMsg = 'td_bridge_failed';
end
end

function [ok, elapsed, errMsg] = attempt_ptc_bridge(model, solid, st, deltaStartUm, deltaEndUm, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, attemptId, bnd_eval, bnd_rigid_top)
ok = false;
elapsed = 0;
errMsg = '';
ensure_ptc_bridge_header(ptcBridgePath);
try
    model.param.set('delta', sprintf('%g[um]', deltaEndUm));
    try, solid.feature('dcnt1').active(true); catch, end
    try, st.set('useinitsol', 'on'); catch, end
    try, st.set('initmethod', 'sol'); catch, end
    try, st.set('initsol', 'current'); catch, end
    configure_ptc_solver(model, ptcTimeStep, ptcMaxSteps, ptcDamping);
    t0 = tic;
    model.study('std1').run();
    elapsed = toc(t0);
    ok = true;
catch ME
    errMsg = string(ME.message);
end

write_ptc_bridge_results(ptcBridgePath, attemptId, ok, bnd_eval, bnd_rigid_top, model);
if ~ok && strlength(string(errMsg)) == 0
    errMsg = 'ptc_bridge_failed';
end
end

function configure_ptc_solver(model, ptcTimeStep, ptcMaxSteps, ptcDamping)
try
    soltags = cell(model.sol.tags);
catch
    return;
end
for i = 1:numel(soltags)
    try
        sol = model.sol(soltags{i});
        ftags = cell(sol.feature.tags);
    catch
        continue;
    end
    for j = 1:numel(ftags)
        try
            feat = sol.feature(ftags{j});
            try, feat.set('pseudotime', 'on'); catch, end
            try, feat.set('ptc', 'on'); catch, end
            try, feat.set('ptctime', ptcTimeStep); catch, end
            try, feat.set('ptcmaxsteps', ptcMaxSteps); catch, end
            try, feat.set('ptcdamp', ptcDamping); catch, end
            try, feat.set('dampctrl', 'constant'); catch, end
            try, feat.set('damp', ptcDamping); catch, end
            try, feat.set('nlin', 'damped'); catch, end
        catch
        end
    end
end
end

function ensure_ptc_bridge_header(path)
if exist(path, 'file')
    return;
end
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'attempt_id,step_idx,residual_norm,Ac_m2,Fz_plate_top_int_N\n');
fclose(fid);
end

function write_ptc_bridge_results(path, attemptId, ok, bnd_eval, bnd_rigid_top, model)
resNorm = NaN;
Ac = NaN;
Fz = NaN;
try
    Ac = mphint2(model, 'if(solid.incontact>0.5,1,0)', 'surface', 'selection', bnd_eval);
catch
    Ac = NaN;
end
try, Fz = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_rigid_top); catch, end
fid = fopen(path, 'a', 'n', 'UTF-8');
fprintf(fid, '%d,%d,%.6g,%.6g,%.6g\n', attemptId, tern(ok,1,0), resNorm, Ac, Fz);
fclose(fid);
end

function ensure_td_bridge_header(path)
if exist(path, 'file')
    return;
end
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'attempt_id,bridge_mode,t,delta_target_um,Ac_m2,pn_avg_Pa,Fz_plate_top_int_N\n');
fclose(fid);
end

function write_td_bridge_results(path, attemptId, mode, deltaStartUm, deltaEndUm, bridgeDt, bnd_eval, bnd_rigid_top, model)
if ~isfinite(bridgeDt) || bridgeDt <= 0
    bridgeDt = 0.05;
end
tlist = 0:bridgeDt:1;
if isempty(tlist) || abs(tlist(end) - 1) > 1e-12
    tlist = [tlist, 1];
end
fid = fopen(path, 'a', 'n', 'UTF-8');
for i = 1:numel(tlist)
    t = tlist(i);
    ramp = 0.5 * (1 - cos(pi * t));
    deltaUm = deltaStartUm + (deltaEndUm - deltaStartUm) * ramp;
    Ac = NaN;
    pnAvg = NaN;
    Fz = NaN;
    try
        Ac = mphint2(model, 'if(solid.incontact>0.5,1,0)', 'surface', 'selection', bnd_eval, 't', t);
        pnInt = mphint2(model, 'solid.Tn', 'surface', 'selection', bnd_eval, 't', t);
        if isfinite(Ac) && Ac > 0
            pnAvg = pnInt ./ Ac;
        end
    catch
        Ac = NaN;
        pnAvg = NaN;
    end
    try, Fz = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_rigid_top, 't', t); catch, end
    if strcmpi(mode, 'TD_RELAX')
        deltaUm = deltaEndUm;
    end
    fprintf(fid, '%d,%s,%.6g,%.6g,%.6g,%.6g,%.6g\n', attemptId, mode, t, deltaUm, Ac, pnAvg, Fz);
end
fclose(fid);
end

function append_td_bridge_failure(path, attemptId, mode, deltaEndUm)
fid = fopen(path, 'a', 'n', 'UTF-8');
fprintf(fid, '%d,%s,NaN,%.6g,NaN,NaN,NaN\n', attemptId, mode, deltaEndUm);
fclose(fid);
end

function [bndFiltered, zTopUm, zThresholdUm] = filter_boundaries_by_centroid_z(model, bndIds, zWindowUm)
%FILTER_BOUNDARIES_BY_CENTROID_Z Keep only boundaries with centroid_z within z_window of the topmost centroid.
% Uses coordinate-only integrals, so it does not depend on solution fields.

if isempty(bndIds)
    bndFiltered = [];
    zTopUm = NaN;
    zThresholdUm = NaN;
    return;
end

centZ = nan(size(bndIds));
area = nan(size(bndIds));
for i = 1:numel(bndIds)
    bid = bndIds(i);
    try
        a = mphint2(model, '1', 'surface', 'selection', bid);
        area(i) = a;
    catch
        area(i) = NaN;
    end
    try
        zint = mphint2(model, 'z', 'surface', 'selection', bid);
    catch
        zint = mphint2(model, 'comp1.z', 'surface', 'selection', bid);
    end
    if isfinite(area(i)) && area(i) > 0
        centZ(i) = zint / area(i);
    else
        centZ(i) = NaN;
    end
end

zTop = max(centZ(isfinite(centZ)));
if isempty(zTop) || ~isfinite(zTop)
    bndFiltered = [];
    zTopUm = NaN;
    zThresholdUm = NaN;
    return;
end

zThreshold = zTop - (zWindowUm * 1e-6);
keep = isfinite(centZ) & centZ >= zThreshold;
bndFiltered = bndIds(keep);
zTopUm = zTop * 1e6;
zThresholdUm = zThreshold * 1e6;
end

function [hmaxM, hminM] = extract_mesh_hmax_hmin(ms)
hmaxM = NaN;
hminM = NaN;
try
    if isfield(ms, 'hmax') && isfinite(ms.hmax)
        hmaxM = ms.hmax;
    end
    if isfield(ms, 'hmin') && isfinite(ms.hmin)
        hminM = ms.hmin;
    end
catch
end
end

function [minQ, nInv] = extract_mesh_quality(ms)
minQ = NaN;
nInv = NaN;
try
    if isfield(ms, 'minquality')
        minQ = ms.minquality;
    elseif isfield(ms, 'minqual')
        minQ = ms.minqual;
    elseif isfield(ms, 'qualitymin')
        minQ = ms.qualitymin;
    end
catch
end
try
    if isfield(ms, 'ninverted')
        nInv = ms.ninverted;
    elseif isfield(ms, 'inverted')
        nInv = ms.inverted;
    elseif isfield(ms, 'numinverted')
        nInv = ms.numinverted;
    end
catch
end
end

function rebuild_mesh_with_local_size(mesh1, bndFiltered, hgrad, hmaxM, hminM)
%REBUILD_MESH_WITH_LOCAL_SIZE Rebuild a stable mesh sequence with a global Size(hauto=3)
% plus a local Size on selected boundaries. This avoids empty-mesh issues seen when adding
% a size feature into a purely automatic mesh sequence.

% Clear existing mesh features (best-effort; template may be automatic-generated).
try
    tags = cell(mesh1.feature.tags);
    for i = 1:numel(tags)
        try, mesh1.feature.remove(tags{i}); catch, end
    end
catch
end

% Global size (keep baseline comparable to autoMeshSize(3)).
try
    mesh1.feature.create('size1', 'Size');
    try, mesh1.feature('size1').selection.all; catch, end
    try, mesh1.feature('size1').set('hauto', 3); catch, end
catch
end

% Local size on filtered contact-adjacent boundaries.
try
    mesh1.feature.create('size_local_contact', 'Size');
catch
end
try
    sz = mesh1.feature('size_local_contact');
    try, sz.selection.geom('geom1', 2); catch, end
    try, sz.selection.set(bndFiltered(:)'); catch, end
    try, sz.set('hgradactive', 1); catch, end
    try, sz.set('hgrad', hgrad); catch, end
    try, sz.set('hmaxactive', 1); catch, end
    try, sz.set('hmax', sprintf('%.6g[m]', hmaxM)); catch, end
    try, sz.set('hminactive', 1); catch, end
    try, sz.set('hmin', sprintf('%.6g[m]', hminM)); catch, end
catch
end

% Free tetrahedral mesh (with boundary triangulation).
try
    mesh1.feature.create('ftri1', 'FreeTri');
    try, mesh1.feature('ftri1').selection.geom('geom1', 2); catch, end
    try, mesh1.feature('ftri1').selection.all; catch, end
catch
end
try
    mesh1.feature.create('ftet1', 'FreeTet');
    try, mesh1.feature('ftet1').selection.geom('geom1', 3); catch, end
    try, mesh1.feature('ftet1').selection.all; catch, end
    % Improve quality / smoothing (best-effort; not all COMSOL versions expose these keys).
    try, mesh1.feature('ftet1').set('improve', 1); catch, end
    try, mesh1.feature('ftet1').set('optimize', 1); catch, end
    try, mesh1.feature('ftet1').set('smoothing', 1); catch, end
catch
end
end

function [nLt0p1, nLt0p01] = extract_mesh_quality_counts(ms)
nLt0p1 = NaN;
nLt0p01 = NaN;

% COMSOL mesh stats structures vary by version; attempt best-effort extraction.
try
    if isfield(ms, 'quality') && ~isempty(ms.quality)
        q = ms.quality;
        q = q(:);
        q = q(isfinite(q));
        nLt0p1 = sum(q < 0.1);
        nLt0p01 = sum(q < 0.01);
        return;
    end
catch
end

try
    if isfield(ms, 'qualities') && ~isempty(ms.qualities)
        q = ms.qualities;
        q = q(:);
        q = q(isfinite(q));
        nLt0p1 = sum(q < 0.1);
        nLt0p01 = sum(q < 0.01);
        return;
    end
catch
end
end
