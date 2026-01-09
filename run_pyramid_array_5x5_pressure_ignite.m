function run_pyramid_array_5x5_pressure_ignite()
%RUN_PYRAMID_ARRAY_5X5_PRESSURE_IGNITE Pressure-control "ignite" from an existing contact checkpoint.
%
% Goal: validate that pressure-control is feasible without over/under-constraint by:
% - Loading an existing displacement-control contact solution checkpoint as initial values
% - Disabling z-displacement prescription on the rigid plate top (avoid over-constraint)
% - Applying pressure load P_load (downwards) on the rigid plate top boundary
% - Switching to fully-coupled (disable segregated) + Direct(PARDISO) linear solver
% - Producing a single-row metrics file for RP: Ac(P), etc.
% - Ignite-v2: optional pressure ramp s=0.25->1.0 and fully-coupled path
% - Ignite-v3: two-stage (single then ramp), custom ramp list, robust FC options, and internal budget
%
% Outputs:
%   out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/metrics_pressure.csv
%   out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/summary.txt
%   out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/errors.json (on FAIL)
%   out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/checkpoint_last_ok.mph (on SUCCESS)

import com.comsol.model.util.*

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>

outRoot = fullfile(pwd, 'out', 'pyramid_5x5');
if ~exist(outRoot, 'dir')
    mkdir(outRoot);
end
outDir = fullfile(outRoot, ['pressure_ignite_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

summaryPath = fullfile(outDir, 'summary.txt');
metricsPath = fullfile(outDir, 'metrics_pressure.csv');
errorsPath = fullfile(outDir, 'errors.json');
checkpointOut = fullfile(outDir, 'checkpoint_last_ok.mph');

fromSimDir = getenv('PRESSURE_IGNITE_FROM_SIM_DIR');
if isempty(strtrim(fromSimDir))
    fromSimDir = fullfile(pwd, 'out', 'pyramid_5x5', 'sim_20260107_152913');
end
fromSimDir = resolve_path(pwd, fromSimDir);
ckIn = fullfile(fromSimDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
if ~exist(ckIn, 'file')
    error('Missing resume checkpoint: %s', ckIn);
end

PloadKPa = numeric_env('SIM_P_LOAD_KPA', 0.6);
tnEpsPa = numeric_env('SIM_TN_EPS_PA', 1.0);
contactMode = get_env_or_default('PHASE2_CONTACT_MODE', 'augmented_lagrange');
linearSolverMode = get_env_or_default('SIM_LINEAR_SOLVER_MODE', 'direct_pardiso');
solverCoupling = get_env_or_default('SIM_SOLVER_COUPLING', 'segregated');
igniteRamp = numeric_env('SIM_IGNITE_RAMP', 0);
plateRbmFixRequested = numeric_env('SIM_PLATE_RBM_FIX', 1) > 0.5;

igniteRampListStr = get_env_or_default('SIM_IGNITE_RAMP_LIST', '');
igniteRampDefaultMode = get_env_or_default('SIM_IGNITE_RAMP_DEFAULT_MODE', 'legacy');
igniteTwoStage = numeric_env('SIM_IGNITE_TWO_STAGE', 0);
igniteBudgetS = numeric_env('SIM_IGNITE_BUDGET_S', 1500);

fcMode = get_env_or_default('SIM_FC_MODE', 'fast_fail');
fcMaxIterFast = numeric_env('SIM_FC_MAXITER_FAST', 12);
fcMaxIterRobust = numeric_env('SIM_FC_MAXITER_ROBUST', 30);
fcMaxIterOverride = numeric_env('SIM_FC_MAXITER', NaN); % backwards-compat override
fcDampedRequested = numeric_env('SIM_FC_DAMPED', 0);
fcLineSearchRequested = numeric_env('SIM_FC_LINESEARCH', 0);
stabilizationRequested = numeric_env('SIM_SOLVER_STABILIZATION', 0);

disableSegregated = strcmpi(strtrim(solverCoupling), 'fully_coupled') || strcmpi(strtrim(solverCoupling), 'fully');
maxSegIter = numeric_env('SIM_MAXSEGITER', 6);
maxSubIter = numeric_env('SIM_MAXSUBITER', 4);
if strcmpi(strtrim(fcMode), 'robust')
    fcMaxIterTarget = fcMaxIterRobust;
else
    fcMaxIterTarget = fcMaxIterFast;
end
if isfinite(fcMaxIterOverride)
    fcMaxIterTarget = fcMaxIterOverride;
end

runId = datestr(now, 'yyyymmdd_HHMMSS');

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "summary_stage: START\n");
fprintf(fid, "pressure_ignite_from_sim_dir: %s\n", string(fromSimDir));
fprintf(fid, "resume_checkpoint: %s\n", string(ckIn));
fprintf(fid, "P_load_kPa: %.6g\n", PloadKPa);
fprintf(fid, "contact_mode_requested: %s\n", string(contactMode));
fprintf(fid, "linear_solver_mode_requested: %s\n", string(linearSolverMode));
fprintf(fid, "solver_coupling: %s\n", string(solverCoupling));
fprintf(fid, "plate_rbm_fix_requested: %d\n", tern(plateRbmFixRequested, 1, 0));
fprintf(fid, "ignite_ramp_enabled: %d\n", tern(igniteRamp > 0.5, 1, 0));
fprintf(fid, "ignite_ramp_list_str: %s\n", string_or_none(igniteRampListStr));
fprintf(fid, "ignite_ramp_default_mode: %s\n", string_or_none(igniteRampDefaultMode));
fprintf(fid, "ignite_two_stage: %d\n", tern(igniteTwoStage > 0.5, 1, 0));
fprintf(fid, "ignite_budget_s: %g\n", igniteBudgetS);
fprintf(fid, "disable_segregated: %d\n", disableSegregated);
fprintf(fid, "segregated_maxsegiter: %g\n", maxSegIter);
fprintf(fid, "segregated_maxsubiter: %g\n", maxSubIter);
fprintf(fid, "fc_mode: %s\n", string_or_none(fcMode));
fprintf(fid, "fc_maxiter_target: %g\n", fcMaxIterTarget);
fprintf(fid, "fc_damped_requested: %d\n", tern(fcDampedRequested > 0.5, 1, 0));
fprintf(fid, "fc_linesearch_requested: %d\n", tern(fcLineSearchRequested > 0.5, 1, 0));
fprintf(fid, "stabilization_requested: %d\n", tern(stabilizationRequested > 0.5, 1, 0));
fprintf(fid, "run_id: %s\n", string(runId));
fclose(fid);

write_metrics_pressure_header(metricsPath);

runT0 = tic;
model = mphload(ckIn);
model.hist.disable();

comp = model.component('comp1');
solid = comp.physics('solid');
pc = comp.pair('pc');

% Enforce dcnt1-only and bind to pair 'pc'.
dcntOnlyOk = false;
dcntOnlyNote = 'none';
try
    [dcntOnlyOk, dcntOnlyNote] = enforce_dcnt1_only(solid, 'dcnt1', 'cnt1', 'pc');
catch ME
    dcntOnlyOk = false;
    dcntOnlyNote = string(ME.message);
end

% Contact preset (augmented lagrange).
contactSupported = true;
contactEffective = contactMode;
contactNote = 'none';
try
    dcnt = solid.feature('dcnt1');
    [contactSupported, contactEffective, contactNote] = apply_contact_mode(dcnt, 'dcnt1', contactMode, 1.0, 1.0);
catch ME
    contactSupported = false;
    contactEffective = 'unknown';
    contactNote = string(ME.message);
end

% Boundary selections
bnd_rigid_top = solid.feature('bndl1').selection.entities;
bnd_eval = pc.destination.entities;

% Plate rigid-body-mode suppression (minimal modeling fix) for pressure-control ignite:
% Prefer point-based constraints (x/y only) vs constraining x/y over the whole top boundary.
plateRbmFixEffective = false;
plateRbmFixNote = "none";
plateRbmFixVtx = [];
dispTopOk = false;
dispTopNote = "none";
if plateRbmFixRequested
    try
        [plateRbmFixEffective, plateRbmFixNote, plateRbmFixVtx] = apply_plate_rbm_fix(model, solid, bnd_rigid_top);
    catch ME
        plateRbmFixEffective = false;
        plateRbmFixNote = string(ME.message);
        plateRbmFixVtx = [];
    end
end

% Displacement-control: ensure z is free (avoid over-constraint). If RBM fix is effective, disable disp_top.
if plateRbmFixEffective
    try
        solid.feature('disp_top').active(false);
    catch
    end
    dispTopOk = true;
    dispTopNote = "disp_top_disabled_due_to_plate_rbm_fix";
else
    try
        solid.feature('disp_top').active(true);
        solid.feature('disp_top').selection.set(bnd_rigid_top);
        solid.feature('disp_top').set('Direction', {'prescribed','prescribed','free'});
        solid.feature('disp_top').set('U0', {'0','0','0'});
        dispTopOk = true;
        dispTopNote = "disp_top_xy_prescribed";
    catch ME
        dispTopOk = false;
        dispTopNote = string(ME.message);
    end
end

% Apply pressure load in -z via P_load parameter.
pressureOk = false;
try
    bndl = solid.feature('bndl1');
    bndl.set('forceType', 'FollowerPressure');
    bndl.set('pressure', '-P_load');
    bndl.active(true);
    pressureOk = true;
catch
end

% Solver: force Direct(PARDISO) where possible, then configure coupling.
linearSwitched = false;
linearNote = 'none';
try
    [linearSwitched, linearNote] = configure_linear_solver_mode_scan(model, linearSolverMode);
catch ME
    linearSwitched = false;
    linearNote = string(ME.message);
end

segDisabled = false;
segNote = 'none';
fcEnabled = false;
fcNote = 'none';
fcMaxIterSet = NaN;
fcDampedEffective = false;
fcDampedNote = 'none';
fcLineSearchEffective = false;
fcLineSearchNote = 'none';
stabilizationEffective = false;
stabilizationNote = 'none';
try
    [segDisabled, segNote, fcEnabled, fcNote, fcMaxIterSet, fcDampedEffective, fcDampedNote, fcLineSearchEffective, fcLineSearchNote, stabilizationEffective, stabilizationNote] = ...
        configure_solver_coupling(model, solverCoupling, maxSegIter, maxSubIter, fcMaxIterTarget, fcMode, fcDampedRequested > 0.5, fcLineSearchRequested > 0.5, stabilizationRequested > 0.5);
catch ME
    segDisabled = false;
    segNote = string(ME.message);
    fcEnabled = false;
    fcNote = string(ME.message);
    fcMaxIterSet = NaN;
    fcDampedEffective = false;
    fcDampedNote = string(ME.message);
    fcLineSearchEffective = false;
    fcLineSearchNote = string(ME.message);
    stabilizationEffective = false;
    stabilizationNote = string(ME.message);
end

% Ensure we actually use initial values from the loaded checkpoint.
st = model.study('std1').feature('stat');
try, st.set('initmethod', 'sol'); catch, end
try, st.set('initsol', 'current'); catch, end
try, st.set('useinitsol', 'on'); catch, end

% Ignite-v3 step planning:
% - optional two-stage: first try s=1.0 once; only if that fails, run ramp list
% - ramp list can be overridden; default behavior remains legacy unless explicitly requested
[sRamp, sRampNote] = resolve_ramp_list(igniteRamp > 0.5, igniteRampListStr, igniteRampDefaultMode);

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "ramp_list_effective_s: %s\n", s_list_to_string(sRamp));
fprintf(fid, "ramp_list_note: %s\n", sanitize_csv_text(string_or_none(sRampNote)));
fclose(fid);

budgetS = igniteBudgetS;
if ~isfinite(budgetS) || budgetS <= 0
    budgetS = inf;
end
budgetMinNextS = 60;

steps = struct('stage', {}, 'step_index', {}, 's', {}, 'P_load_kPa', {}, 'start_time_iso', {}, 'end_time_iso', {}, 'elapsed_s', {}, 'exit_status', {}, 'reason', {}, 'comsol_message_excerpt', {}, 'checkpoint_saved', {});
hadAnySuccess = false;
lastOkGlobal = 0;
lastOkStage = "none";
lastOkS = NaN;
lastOkP = NaN;
stage1Failed = false;
stage1FailReason = "none";
failReason = "none";
failStage = "none";
failStepIdx = NaN;
failS = NaN;
failP = NaN;
stopReason = "none";

% Stage sequencing
stagePlan = {};
if igniteTwoStage > 0.5
    stagePlan = {'single','ramp'};
else
    if igniteRamp > 0.5
        stagePlan = {'ramp'};
    else
        stagePlan = {'single'};
    end
end

globalStep = 0;
for iStage = 1:numel(stagePlan)
    stageName = string(stagePlan{iStage});
    if stageName == "single"
        sList = 1.0;
    else
        sList = sRamp;
        if igniteTwoStage > 0.5
            % Avoid duplicating s=1.0 after stage-1 already attempted.
            sList = sList(abs(sList - 1.0) > 1e-12);
        end
    end

    if isempty(sList)
        continue;
    end

    % If stage-1 succeeded in two-stage, stop immediately (do not run ramp).
    if igniteTwoStage > 0.5 && stageName == "ramp" && hadAnySuccess
        break;
    end

    for stepIdx = 1:numel(sList)
        globalStep = globalStep + 1;
        s = sList(stepIdx);
        PstepKPa = PloadKPa * s;
        nowIso = datestr(now, 'yyyy-mm-ddTHH:MM:SS');

        budgetRemainingBefore = budgetS - toc(runT0);
        if budgetRemainingBefore < budgetMinNextS
            stopReason = "BUDGET_EXCEEDED_BEFORE_STEP";
            failReason = stopReason;
            failStage = stageName;
            failStepIdx = stepIdx;
            failS = s;
            failP = PstepKPa;

            % Record a budget skip row (deterministic early exit).
            append_metrics_pressure_row(metricsPath, globalStep, s, PstepKPa, NaN, NaN, NaN, 0, NaN, NaN, "none", false, "BUDGET_SKIP", stopReason, 0, ...
                linearSolverMode, linearSwitched, linearNote, solverCoupling, segDisabled, segNote, fcEnabled, fcMaxIterSet, fcNote, contactMode, contactEffective, contactSupported, contactNote, false, ...
                runId, stageName, fcMode, fcMaxIterTarget, fcDampedRequested > 0.5, fcDampedEffective, fcLineSearchRequested > 0.5, fcLineSearchEffective, stabilizationRequested > 0.5, stabilizationEffective, budgetS, budgetRemainingBefore);

            steps(end+1) = make_step(stageName, stepIdx, s, PstepKPa, nowIso, nowIso, 0, "BUDGET_SKIP", stopReason, stopReason, false); %#ok<AGROW>
            break;
        end

        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "\nStepStart: stage=%s idx=%d global=%d s=%.6g P_load_kPa=%.6g budget_remaining_s=%.1f\n", stageName, stepIdx, globalStep, s, PstepKPa, budgetRemainingBefore);
        fclose(fid);

        % Set P_load for this step.
        try
            model.param.set('P_load', sprintf('%.6g[kPa]', PstepKPa));
        catch
            try, model.param.set('P_load', sprintf('%.6g*1e3[Pa]', PstepKPa)); catch, end
        end

        stepT0 = tic;
        ok = false;
        errMsg = 'none';
        try
            model.study('std1').run();
            ok = true;
        catch ME
            ok = false;
            errMsg = string(ME.message);
        end
        elapsedS = toc(stepT0);
        endIso = datestr(now, 'yyyy-mm-ddTHH:MM:SS');

        ATop = NaN;
        Fz = NaN;
        PEff = NaN;
        tnMax = NaN;
        Ac = 0;
        pnAvg = NaN;
        acWhy = 'none';
        checkpointSaved = false;
        if ok
            try, ATop = mphint2(model, '1', 'surface', 'selection', bnd_rigid_top); catch, end
            try, Fz = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_rigid_top); catch, end
            if isfinite(ATop) && ATop > 0 && isfinite(Fz)
                PEff = abs(Fz) ./ ATop;
            end
            try
                [tnMax, Ac, pnAvg, acWhy] = eval_contact_metrics_with_why(model, bnd_eval, tnEpsPa);
            catch
            end
            try
                model.save(checkpointOut);
                checkpointSaved = true;
            catch
                checkpointSaved = false;
            end
        end

        exitStatusStep = tern(ok, "OK", "FAIL");
        append_metrics_pressure_row(metricsPath, globalStep, s, PstepKPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, exitStatusStep, errMsg, elapsedS, ...
            linearSolverMode, linearSwitched, linearNote, solverCoupling, segDisabled, segNote, fcEnabled, fcMaxIterSet, fcNote, contactMode, contactEffective, contactSupported, contactNote, checkpointSaved, ...
            runId, stageName, fcMode, fcMaxIterTarget, fcDampedRequested > 0.5, fcDampedEffective, fcLineSearchRequested > 0.5, fcLineSearchEffective, stabilizationRequested > 0.5, stabilizationEffective, budgetS, budgetRemainingBefore);

        steps(end+1) = make_step(stageName, stepIdx, s, PstepKPa, nowIso, endIso, elapsedS, exitStatusStep, errMsg, excerpt(string_or_none(errMsg), 300), checkpointSaved); %#ok<AGROW>

        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "StepEnd: stage=%s idx=%d global=%d status=%s elapsed_s=%.6g checkpoint_saved=%d\n", stageName, stepIdx, globalStep, exitStatusStep, elapsedS, tern(checkpointSaved,1,0));
        fclose(fid);

        if ok
            hadAnySuccess = true;
            lastOkGlobal = globalStep;
            lastOkStage = stageName;
            lastOkS = s;
            lastOkP = PstepKPa;
            continue;
        end

        % Failure handling:
        % - In two-stage mode, stage-1 failure proceeds to stage-2 ramp.
        % - Otherwise (or ramp stage), fail-fast immediately after first failed step.
        if igniteTwoStage > 0.5 && stageName == "single"
            stage1Failed = true;
            stage1FailReason = errMsg;
            break;
        end

        failReason = errMsg;
        failStage = stageName;
        failStepIdx = stepIdx;
        failS = s;
        failP = PstepKPa;
        stopReason = "STEP_FAILED";
        break;
    end

    if stopReason == "BUDGET_EXCEEDED_BEFORE_STEP" || stopReason == "STEP_FAILED"
        break;
    end
end

if isempty(strtrim(stopReason)) || stopReason == "none"
    if hadAnySuccess
        stopReason = "COMPLETED";
    else
        stopReason = "FAILED";
    end
end

if ~hadAnySuccess && stage1Failed && igniteTwoStage > 0.5
    % Stage-1 failed and no ramp success => mark failure reason.
    failReason = stage1FailReason;
    failStage = "single";
    failStepIdx = 1;
    failS = 1.0;
    failP = PloadKPa;
    stopReason = "STAGE1_FAILED_NO_RAMP_SUCCESS";
end

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "\nsummary_stage: END\n");
fprintf(fid, "dcnt1_only_ok: %d\n", dcntOnlyOk);
fprintf(fid, "dcnt1_only_note: %s\n", sanitize_csv_text(string_or_none(dcntOnlyNote)));
fprintf(fid, "disp_top_xy_only_ok: %d\n", dispTopOk);
fprintf(fid, "disp_top_note: %s\n", sanitize_csv_text(string_or_none(dispTopNote)));
fprintf(fid, "plate_rbm_fix_requested: %d\n", tern(plateRbmFixRequested, 1, 0));
fprintf(fid, "plate_rbm_fix_effective: %d\n", tern(plateRbmFixEffective, 1, 0));
fprintf(fid, "plate_rbm_fix_note: %s\n", sanitize_csv_text(string_or_none(plateRbmFixNote)));
fprintf(fid, "plate_rbm_fix_vertices: %s\n", sanitize_csv_text(string_or_none(mat2str(plateRbmFixVtx))));
fprintf(fid, "pressure_load_ok: %d\n", pressureOk);
fprintf(fid, "solver_coupling: %s\n", sanitize_csv_text(string_or_none(solverCoupling)));
fprintf(fid, "ignite_ramp_enabled: %d\n", tern(igniteRamp > 0.5, 1, 0));
fprintf(fid, "ignite_two_stage: %d\n", tern(igniteTwoStage > 0.5, 1, 0));
fprintf(fid, "ignite_ramp_default_mode: %s\n", sanitize_csv_text(string_or_none(igniteRampDefaultMode)));
fprintf(fid, "ignite_ramp_list_str: %s\n", sanitize_csv_text(string_or_none(igniteRampListStr)));
fprintf(fid, "budget_s: %g\n", budgetS);
fprintf(fid, "budget_used_s: %.6g\n", toc(runT0));
fprintf(fid, "stop_reason: %s\n", sanitize_csv_text(string_or_none(stopReason)));
fprintf(fid, "last_success_global_step: %d\n", lastOkGlobal);
fprintf(fid, "last_success_stage: %s\n", sanitize_csv_text(string_or_none(lastOkStage)));
fprintf(fid, "last_success_s: %s\n", num2str(lastOkS));
fprintf(fid, "last_success_P_load_kPa: %s\n", num2str(lastOkP));
fprintf(fid, "linear_solver_mode: %s\n", sanitize_csv_text(string_or_none(linearSolverMode)));
fprintf(fid, "linear_solver_switched: %d\n", tern(linearSwitched,1,0));
fprintf(fid, "linear_solver_note: %s\n", sanitize_csv_text(string_or_none(linearNote)));
fprintf(fid, "segregated_disabled: %d\n", tern(segDisabled,1,0));
fprintf(fid, "segregated_note: %s\n", sanitize_csv_text(string_or_none(segNote)));
fprintf(fid, "fully_coupled_enabled: %d\n", tern(fcEnabled,1,0));
fprintf(fid, "fully_coupled_maxiter_set: %s\n", num2str(fcMaxIterSet));
fprintf(fid, "fully_coupled_note: %s\n", sanitize_csv_text(string_or_none(fcNote)));
fprintf(fid, "fc_mode: %s\n", sanitize_csv_text(string_or_none(fcMode)));
fprintf(fid, "fc_maxiter_target: %g\n", fcMaxIterTarget);
fprintf(fid, "fc_damped_requested: %d\n", tern(fcDampedRequested > 0.5, 1, 0));
fprintf(fid, "fc_damped_effective: %d\n", tern(fcDampedEffective, 1, 0));
fprintf(fid, "fc_damped_note: %s\n", sanitize_csv_text(string_or_none(fcDampedNote)));
fprintf(fid, "fc_linesearch_requested: %d\n", tern(fcLineSearchRequested > 0.5, 1, 0));
fprintf(fid, "fc_linesearch_effective: %d\n", tern(fcLineSearchEffective, 1, 0));
fprintf(fid, "fc_linesearch_note: %s\n", sanitize_csv_text(string_or_none(fcLineSearchNote)));
fprintf(fid, "stabilization_requested: %d\n", tern(stabilizationRequested > 0.5, 1, 0));
fprintf(fid, "stabilization_effective: %d\n", tern(stabilizationEffective, 1, 0));
fprintf(fid, "stabilization_note: %s\n", sanitize_csv_text(string_or_none(stabilizationNote)));
fprintf(fid, "contact_mode_effective: %s\n", sanitize_csv_text(string_or_none(contactEffective)));
fprintf(fid, "contact_mode_supported: %d\n", tern(contactSupported,1,0));
fprintf(fid, "contact_mode_note: %s\n", sanitize_csv_text(string_or_none(contactNote)));
exitStatus = tern(hadAnySuccess && (stopReason == "COMPLETED" || (igniteTwoStage > 0.5 && stage1Failed && stopReason == "COMPLETED")), "SUCCESS", "FAIL");
fprintf(fid, "exit_status: %s\n", sanitize_csv_text(exitStatus));
if exitStatus == "FAIL"
    fprintf(fid, "fail_stage: %s\n", sanitize_csv_text(string_or_none(failStage)));
    fprintf(fid, "fail_step_idx: %s\n", num2str(failStepIdx));
    fprintf(fid, "fail_step_s: %s\n", num2str(failS));
    fprintf(fid, "fail_step_P_load_kPa: %s\n", num2str(failP));
    fprintf(fid, "fail_reason: %s\n", sanitize_csv_text(string_or_none(failReason)));
else
    fprintf(fid, "fail_reason: none\n");
end
fprintf(fid, "checkpoint_last_ok_exists: %d\n", tern(exist(checkpointOut,'file')==2,1,0));
fclose(fid);

% Always write errors.json (even on SUCCESS) for deterministic post-mortem.
try
    payload = struct();
    payload.run_id = runId;
    payload.exit_status = char(exitStatus);
    payload.stop_reason = char(string_or_none(stopReason));
    payload.P_load_kPa = PloadKPa;
    payload.ignite_ramp_enabled = tern(igniteRamp > 0.5, true, false);
    payload.ignite_ramp_list_str = char(string_or_none(igniteRampListStr));
    payload.ignite_ramp_default_mode = char(string_or_none(igniteRampDefaultMode));
    payload.ignite_two_stage = tern(igniteTwoStage > 0.5, true, false);
    payload.fc_mode = char(string_or_none(fcMode));
    payload.fc_maxiter_target = fcMaxIterTarget;
    payload.fc_damped_requested = tern(fcDampedRequested > 0.5, true, false);
    payload.fc_damped_effective = tern(fcDampedEffective, true, false);
    payload.fc_linesearch_requested = tern(fcLineSearchRequested > 0.5, true, false);
    payload.fc_linesearch_effective = tern(fcLineSearchEffective, true, false);
    payload.stabilization_requested = tern(stabilizationRequested > 0.5, true, false);
    payload.stabilization_effective = tern(stabilizationEffective, true, false);
    payload.budget_s = budgetS;
    payload.budget_used_s = toc(runT0);

    payload.last_success_global_step = lastOkGlobal;
    payload.last_success_stage = char(string_or_none(lastOkStage));
    payload.last_success_s = lastOkS;
    payload.last_success_P_load_kPa = lastOkP;

    payload.fail_stage = char(string_or_none(failStage));
    payload.fail_step_idx = failStepIdx;
    payload.fail_step_s = failS;
    payload.fail_step_P_load_kPa = failP;
    payload.fail_reason = char(string_or_none(failReason));

    payload.linear_solver_mode = char(string_or_none(linearSolverMode));
    payload.linear_solver_switched = linearSwitched;
    payload.linear_solver_note = char(string_or_none(linearNote));
    payload.solver_coupling = char(string_or_none(solverCoupling));
    payload.segregated_disabled = segDisabled;
    payload.segregated_note = char(string_or_none(segNote));
    payload.fully_coupled_enabled = fcEnabled;
    payload.fully_coupled_maxiter_set = fcMaxIterSet;
    payload.fully_coupled_note = char(string_or_none(fcNote));
    payload.contact_mode_requested = char(string_or_none(contactMode));
    payload.contact_mode_effective = char(string_or_none(contactEffective));
    payload.contact_mode_supported = contactSupported;
    payload.contact_mode_note = char(string_or_none(contactNote));

    payload.steps = steps_to_cell(steps);

    txt = jsonencode(payload);
    fid = fopen(errorsPath, 'w', 'n', 'UTF-8');
    fprintf(fid, '%s', txt);
    fclose(fid);
catch
end

function c = steps_to_cell(steps)
% Ensure JSON encodes steps as an array even when there is a single step.
if isempty(steps)
    c = {};
    return;
end
try
    c = num2cell(steps);
catch
    c = {steps};
end
end

try, ModelUtil.remove('model'); catch, end %#ok<TRYNC>
end

function write_metrics_pressure_header(path)
if exist(path, 'file')
    return;
end
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'timestamp_iso,step_index,s,P_load_kPa,P_eff_Pa,Fz_top_int_N,A_top_m2,Ac_m2,Tn_max_Pa,pn_avg_Pa,Ac_method,success,exit_status,fail_reason,elapsed_s,linear_solver_mode,linear_solver_switched,linear_solver_note,solver_coupling,segregated_disabled,segregated_note,fully_coupled_enabled,fully_coupled_maxiter_set,fully_coupled_note,contact_mode_requested,contact_mode_effective,contact_mode_supported,contact_mode_note,checkpoint_saved,run_id,stage,fc_mode,fc_maxiter_target,fc_damped_requested,fc_damped_effective,fc_linesearch_requested,fc_linesearch_effective,stabilization_requested,stabilization_effective,budget_s,budget_remaining_s_before_step,fail_reason_short\n');
fclose(fid);
end

function append_metrics_pressure_row(path, stepIdx, s, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, exitStatus, errMsg, elapsedS, linearSolverMode, switched, switchNote, solverCoupling, segDisabled, segNote, fcEnabled, fcMaxIterSet, fcNote, contactMode, contactEffective, contactSupported, contactNote, checkpointSaved, runId, stageName, fcMode, fcMaxIterTarget, fcDampedRequested, fcDampedEffective, fcLineSearchRequested, fcLineSearchEffective, stabilizationRequested, stabilizationEffective, budgetS, budgetRemainingBefore)
fid = fopen(path, 'a', 'n', 'UTF-8');
ts = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
shortMsg = excerpt(string_or_none(errMsg), 140);
fprintf(fid, '%s,%d,%.6g,%.6g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s,%d,%s,%s,%.6g,%s,%d,%s,%s,%d,%s,%d,%.6g,%s,%s,%s,%d,%s,%d,%s,%s,%s,%.6g,%d,%d,%d,%d,%d,%d,%.6g,%.6g,%s\n', ...
    ts, stepIdx, s, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, sanitize_csv_text(string_or_none(acWhy)), tern(ok,1,0), sanitize_csv_text(string_or_none(exitStatus)), sanitize_csv_text(string_or_none(errMsg)), elapsedS, ...
    sanitize_csv_text(string_or_none(linearSolverMode)), tern(switched,1,0), sanitize_csv_text(string_or_none(switchNote)), ...
    sanitize_csv_text(string_or_none(solverCoupling)), tern(segDisabled,1,0), sanitize_csv_text(string_or_none(segNote)), ...
    tern(fcEnabled,1,0), fcMaxIterSet, sanitize_csv_text(string_or_none(fcNote)), ...
    sanitize_csv_text(string_or_none(contactMode)), sanitize_csv_text(string_or_none(contactEffective)), tern(contactSupported,1,0), sanitize_csv_text(string_or_none(contactNote)), tern(checkpointSaved,1,0), ...
    sanitize_csv_text(string_or_none(runId)), sanitize_csv_text(string_or_none(stageName)), sanitize_csv_text(string_or_none(fcMode)), fcMaxIterTarget, ...
    tern(fcDampedRequested,1,0), tern(fcDampedEffective,1,0), tern(fcLineSearchRequested,1,0), tern(fcLineSearchEffective,1,0), tern(stabilizationRequested,1,0), tern(stabilizationEffective,1,0), ...
    budgetS, budgetRemainingBefore, sanitize_csv_text(shortMsg));
fclose(fid);
end

function [segDisabled, segNote, fcEnabled, fcNote, fcMaxIterSet, dampEff, dampNote, lsEff, lsNote, stabEff, stabNote] = configure_solver_coupling(model, solverCoupling, maxSegIter, maxSubIter, fcMaxIterTarget, fcMode, dampReq, lsReq, stabReq)
segDisabled = false;
segNote = 'none';
fcEnabled = false;
fcNote = 'none';
fcMaxIterSet = NaN;
dampEff = false;
dampNote = 'none';
lsEff = false;
lsNote = 'none';
stabEff = false;
stabNote = 'none';
mode = lower(strtrim(string(solverCoupling)));
if mode == "" || mode == "default"
    mode = "segregated";
end

s1 = model.sol('sol1').feature('s1');
se1 = s1.feature('se1');

if mode == "fully_coupled" || mode == "fully"
    % Disable segregated.
    try, se1.active(false); catch, end
    segDisabled = true;
    segNote = 'sol1/s1/se1 deactivated';

    % Ensure FullyCoupled feature exists and is active.
    try
        s1.feature('fc1');
        hasFc = true;
    catch
        hasFc = false;
    end
    if ~hasFc
        try
            s1.feature.create('fc1', 'FullyCoupled');
            hasFc = true;
        catch ME
            fcEnabled = false;
            fcNote = string(ME.message);
            fcMaxIterSet = NaN;
            dampEff = false; dampNote = 'create_fc1_failed';
            lsEff = false; lsNote = 'create_fc1_failed';
            stabEff = false; stabNote = 'create_fc1_failed';
            return;
        end
    end
    fc1 = s1.feature('fc1');
    try, fc1.active(true); catch, end
    fcEnabled = true;
    fcNote = 'sol1/s1/fc1 enabled';

    % Cap nonlinear iterations for deterministic failure.
    try
        fc1.set('maxiter', fcMaxIterTarget);
        fcMaxIterSet = fcMaxIterTarget;
    catch
        fcMaxIterSet = NaN;
    end
    % Prefer direct solver definition.
    try, fc1.set('linsolver', 'dDef'); catch, end

    % Robust/fast-fail mode knobs (best-effort).
    fcModeStr = lower(strtrim(string(fcMode)));
    if fcModeStr == "" || fcModeStr == "default"
        fcModeStr = "fast_fail";
    end
    if fcModeStr == "robust"
        % Slightly more conservative defaults (best-effort; ignore if unsupported).
        try, fc1.set('adapttol', 'on'); catch, end
        try, fc1.set('initiallintol', 1e-3); catch, end
        try, fc1.set('etamax', 0.9); catch, end
    end

    % Damped Newton (best-effort).
    if dampReq
        [dampEff, dampNote] = try_set_any(fc1, {'dtech','nlin','dampexponent'}, {'damped','dampedNewton','on'});
        if ~dampEff
            [dampEff, dampNote] = try_set_any(fc1, {'damp','dampfactor'}, {0.7, 0.5});
        end
        if ~dampEff
            dampNote = 'not_supported';
        end
    else
        dampEff = false;
        dampNote = 'not_requested';
    end

    % Line search / backtracking (best-effort).
    if lsReq
        [lsEff, lsNote] = try_set_any(fc1, {'backmethod'}, {'auto','linesearch','lineSearch','backtracking'});
        if ~lsEff
            [lsEff, lsNote] = try_set_any(fc1, {'backtrackonce'}, {'on', 1});
        end
        if ~lsEff
            lsNote = 'not_supported';
        end
    else
        lsEff = false;
        lsNote = 'not_requested';
    end

    % Solver stabilization (best-effort; stationary).
    if stabReq
        [stabEff, stabNote] = try_set_any(fc1, {'ressmooth','relaxationressmooth'}, {'on', 1});
        if ~stabEff
            [stabEff, stabNote] = try_set_any(fc1, {'stabacc'}, {'on', 1});
        end
        if ~stabEff
            stabNote = 'not_supported';
        end
    else
        stabEff = false;
        stabNote = 'not_requested';
    end
else
    % Segregated (legacy).
    try, se1.active(true); catch, end
    try, se1.set('maxsegiter', maxSegIter); catch, end
    try
        ss1 = se1.feature('ss1');
        try, ss1.set('maxsubiter', maxSubIter); catch, end
        try, ss1.set('linsolver', 'dDef'); catch, end
    catch
    end
    segDisabled = false;
    segNote = sprintf('sol1/s1/se1 active; maxsegiter=%g, maxsubiter=%g', maxSegIter, maxSubIter);
    % Ensure fc1 (if present) is not the active path.
    try
        s1.feature('fc1').active(false);
    catch
    end
    fcEnabled = false;
    fcNote = 'disabled';
    fcMaxIterSet = NaN;
    dampEff = false; dampNote = 'n/a (segregated)';
    lsEff = false; lsNote = 'n/a (segregated)';
    stabEff = false; stabNote = 'n/a (segregated)';
end
end

function [sList, note] = resolve_ramp_list(rampEnabled, listStr, defaultMode)
sList = [];
note = 'none';
if ~rampEnabled
    sList = [];
    note = 'ramp_disabled';
    return;
end

if ~isempty(strtrim(string(listStr)))
    [vals, ok] = parse_num_list(listStr);
    if ok && ~isempty(vals)
        sList = vals(:).';
        note = 'from_SIM_IGNITE_RAMP_LIST';
        return;
    end
    note = 'invalid_SIM_IGNITE_RAMP_LIST_fallback';
end

mode = lower(strtrim(string(defaultMode)));
if mode == "" || mode == "default"
    mode = "legacy";
end
if mode == "high_to_low"
    sList = [1.0, 0.75, 0.5, 0.25];
    note = 'default_high_to_low';
else
    % legacy (backwards compatible)
    sList = [0.25, 0.5, 0.75, 1.0];
    note = 'default_legacy';
end
end

function [vals, ok] = parse_num_list(s)
ok = false;
vals = [];
try
    parts = split(string(s), {',',';',' ','\t'});
    parts = parts(parts ~= "");
    vv = nan(size(parts));
    for i = 1:numel(parts)
        vv(i) = str2double(parts(i));
    end
    vv = vv(isfinite(vv));
    if isempty(vv)
        ok = false;
        vals = [];
        return;
    end
    ok = true;
    vals = vv;
catch
    ok = false;
    vals = [];
end
end

function s = s_list_to_string(v)
if isempty(v)
    s = "[]";
    return;
end
try
    s = mat2str(v, 6);
catch
    s = "[]";
end
end

function st = make_step(stage, stepIdx, s, PkPa, startIso, endIso, elapsedS, exitStatus, reason, excerptMsg, checkpointSaved)
st = struct();
st.stage = char(string(stage));
st.step_index = stepIdx;
st.s = s;
st.P_load_kPa = PkPa;
st.start_time_iso = char(string_or_none(startIso));
st.end_time_iso = char(string_or_none(endIso));
st.elapsed_s = elapsedS;
st.exit_status = char(string_or_none(exitStatus));
st.reason = char(string_or_none(reason));
st.comsol_message_excerpt = char(string_or_none(excerptMsg));
st.checkpoint_saved = tern(checkpointSaved, true, false);
end

function [ok, note] = try_set_any(feat, propNames, candidates)
ok = false;
note = 'none';
if ischar(propNames) || isstring(propNames)
    propNames = {char(propNames)};
end
if ~iscell(candidates)
    candidates = {candidates};
end
for iP = 1:numel(propNames)
    prop = propNames{iP};
    for iC = 1:numel(candidates)
        val = candidates{iC};
        try
            feat.set(prop, val);
            ok = true;
            note = sprintf('%s set', prop);
            return;
        catch
        end
    end
end
end

function s = excerpt(s, maxLen)
s = string_or_none(s);
if nargin < 2 || ~isfinite(maxLen) || maxLen <= 0
    return;
end
if strlength(s) > maxLen
    s = extractBefore(s, maxLen+1);
end
end

function [tnMax, Ac, pnAvg, why] = eval_contact_metrics_with_why(model, bnd_eval, tnEpsPa)
tnMax = NaN;
Ac = 0;
pnAvg = NaN;
why = 'none';
if nargin < 3 || ~isfinite(tnEpsPa)
    tnEpsPa = 1.0;
end
[ok, tnMax, Ac, pnAvg, why] = try_contact_metrics(model, bnd_eval, tnEpsPa);
if ~ok
    tnMax = NaN;
    Ac = 0;
    pnAvg = NaN;
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

function [ok, note] = enforce_dcnt1_only(solid, dcntTag, cntTag, pairTag)
ok = false;
note = 'none';
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
try
    dcnt = solid.feature(dcntTag);
catch ME
    note = "missing_" + string(dcntTag) + ": " + string(ME.message);
    return;
end
try, dcnt.active(true); catch, end
pairOk = bind_pair_to_feature(dcnt, pairTag);
ok = pairOk;
if ~pairOk
    note = "dcnt_pair_bind_failed(" + string(pairTag) + ")";
end
end

function ok = bind_pair_to_feature(feat, pairTag)
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
ok = is_pair_bound(feat, pairTag);
end

function ok = is_pair_bound(feat, pairTag)
ok = false;
try
    v = feat.getStringArray('pairs');
    if ~isempty(v)
        for i = 1:numel(v)
            if strcmp(strtrim(char(v(i))), pairTag)
                ok = true;
                return;
            end
        end
    end
catch
end
try
    v = char(feat.getString('pair'));
    if strcmp(strtrim(v), pairTag)
        ok = true;
        return;
    end
catch
end
try
    v = char(feat.getString('pairname'));
    if strcmp(strtrim(v), pairTag)
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

function [switched, note] = configure_linear_solver_mode_scan(model, mode)
% Best-effort linear solver switch for this run (does not change physics).
% Supported:
% - default/none: no changes
% - direct_pardiso: set linsolver=pardiso on Direct nodes (prefer sol1/s1/dDef)
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

% Prefer the known default direct node.
try
    dDef = model.sol('sol1').feature('s1').feature('dDef');
    dDef.set('linsolver', 'pardiso');
    switched = true;
    note = 'sol1/s1/dDef linsolver=pardiso';
    return;
catch
end

% Fallback: scan all solver features and set PARDISO on Direct nodes.
setCount = 0;
try
    soltags = cell(model.sol.tags);
catch
    soltags = {};
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

function v = numeric_env(name, defaultValue)
v = defaultValue;
txt = getenv(name);
if isempty(txt)
    return;
end
vv = str2double(txt);
if isfinite(vv)
    v = vv;
end
end

function s = get_env_or_default(name, defaultValue)
val = getenv(name);
if isempty(val)
    s = defaultValue;
else
    s = val;
end
end

function p = resolve_path(baseDir, p)
% Resolve a possibly relative path to an absolute filesystem path.
if isempty(strtrim(p))
    p = '';
    return;
end
pp = string(p);
if is_absolute_path(pp)
    p = char(pp);
else
    p = char(fullfile(baseDir, char(pp)));
end
end

function ok = is_absolute_path(p)
pp = char(p);
ok = false;
if numel(pp) >= 2 && pp(2) == ':'
    ok = true;
    return;
end
if startsWith(pp, filesep)
    ok = true;
end
end

function out = tern(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function s = string_or_none(x)
if isstring(x) || ischar(x)
    s = string(x);
else
    try
        s = string(x);
    catch
        s = "none";
    end
end
if strlength(s) == 0
    s = "none";
end
end

function t = sanitize_csv_text(t)
t = string(t);
t = replace(t, newline, ' ');
t = replace(t, char(13), ' ');
t = replace(t, ',', ';');
end

function [ok, note, vtxPicked] = apply_plate_rbm_fix(model, solid, bndRigidTop)
%APPLY_PLATE_RBM_FIX Minimal rigid-body-mode suppression for pressure plate:
% - select 3 top-plane vertices adjacent to the pressure boundary
% - constrain:
%   v1: x=0, y=0 (reference)
%   v2: x=0 (remove in-plane rotation about z)
%   v3: y=0 (remove in-plane rotation about z)
% - never constrain z here
%
% If vertex selection or feature creation is unsupported, return ok=false with reason.

ok = false;
note = "none";
vtxPicked = [];

if isempty(bndRigidTop)
    note = "bndRigidTop_empty";
    return;
end

geomTag = "geom1";

% Find points adjacent to top boundary selection (best-effort).
vtx = [];
try
    % NOTE: mphgetadj signature is (returntype, adjtype, adjnumber).
    % We want points adjacent to boundary entities.
    % Some LiveLink versions don't reliably accept an array for ADJNUMBER, so union per-boundary.
    vtxAll = [];
    for k = 1:numel(bndRigidTop)
        b = bndRigidTop(k);
        try
            vtxK = mphgetadj(model, geomTag, 'point', 'boundary', b);
            vtxAll = [vtxAll, vtxK(:)']; %#ok<AGROW>
        catch
            % continue best-effort
        end
    end
    vtx = unique(vtxAll);
catch ME
    note = "mphgetadj_failed:" + string(ME.message);
    return;
end

vtx = unique(vtx(:)');
if isempty(vtx)
    note = "no_vertices_adjacent";
    return;
end

coords = [];
try
    coords = mphgetcoords(model, geomTag, 'point', vtx);
catch ME
    note = "mphgetcoords_failed:" + string(ME.message);
    return;
end

% Expect coords as 3xN (x;y;z). Handle transposed cases.
if size(coords, 1) ~= 3 && size(coords, 2) == 3
    coords = coords.';
end
if size(coords, 1) ~= 3
    note = "unexpected_coords_shape:" + string(mat2str(size(coords)));
    return;
end

x = coords(1, :);
y = coords(2, :);
z = coords(3, :);

zTop = max(z);
zTol = 1e-9; % 1 nm in meters; robust enough for planar top surface
topMask = abs(z - zTop) <= zTol;
if ~any(topMask)
    % Fall back: take all vertices if we can't identify top plane.
    topMask = true(size(z));
end

vtxTop = vtx(topMask);
xTop = x(topMask);
yTop = y(topMask);

% Pick v1 near (xmin,ymin), v2 near (xmax,ymin), v3 near (xmin,ymax).
[xmin, ~] = min(xTop);
[xmax, ~] = max(xTop);
[ymin, ~] = min(yTop);
[ymax, ~] = max(yTop);

dist2 = @(xx, yy, x0, y0) (xx - x0).^2 + (yy - y0).^2;

[~, i1] = min(dist2(xTop, yTop, xmin, ymin));
[~, i2] = min(dist2(xTop, yTop, xmax, ymin));
[~, i3] = min(dist2(xTop, yTop, xmin, ymax));

v1 = vtxTop(i1);
v2 = vtxTop(i2);
v3 = vtxTop(i3);

vtxPicked = [v1, v2, v3];
vtxPicked = unique(vtxPicked, 'stable');

if numel(vtxPicked) < 2
    note = "insufficient_unique_vertices";
    return;
end

% Create/enable point-wise prescribed displacement features.
try
    % v1: x,y fixed
    tag1 = "disp_plate_rbm_ref";
    ensure_prescribed_disp_point(solid, tag1, v1, {'prescribed','prescribed','free'}, {'0','0','0'});

    if numel(vtxPicked) >= 2
        % v2: x fixed
        tag2 = "disp_plate_rbm_x";
        ensure_prescribed_disp_point(solid, tag2, v2, {'prescribed','free','free'}, {'0','0','0'});
    end
    if numel(vtxPicked) >= 3
        % v3: y fixed
        tag3 = "disp_plate_rbm_y";
        ensure_prescribed_disp_point(solid, tag3, v3, {'free','prescribed','free'}, {'0','0','0'});
    end
catch ME
    ok = false;
    note = "create_point_disp_failed:" + string(ME.message);
    return;
end

ok = true;
note = "point_constraints_applied";
end

function ensure_prescribed_disp_point(solid, tag, vtxId, direction, u0)
%ENSURE_PRESCRIBED_DISP_POINT Ensure a point prescribed displacement exists and is configured.
%
% COMSOL feature types vary by version; we only use PrescribedDisplacement and fail
% gracefully to caller if creation isn't supported.

try
    f = solid.feature(tag);
catch
    solid.feature.create(tag, 'PrescribedDisplacement', 0);
    f = solid.feature(tag);
end
f.active(true);
f.selection.set(vtxId);
try, f.set('Direction', direction); catch, end
try, f.set('U0', u0); catch, end
end
