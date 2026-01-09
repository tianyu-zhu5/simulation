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

disableSegregated = strcmpi(strtrim(solverCoupling), 'fully_coupled') || strcmpi(strtrim(solverCoupling), 'fully');
maxSegIter = numeric_env('SIM_MAXSEGITER', 6);
maxSubIter = numeric_env('SIM_MAXSUBITER', 4);
fcMaxIter = numeric_env('SIM_FC_MAXITER', 12);

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "summary_stage: START\n");
fprintf(fid, "pressure_ignite_from_sim_dir: %s\n", string(fromSimDir));
fprintf(fid, "resume_checkpoint: %s\n", string(ckIn));
fprintf(fid, "P_load_kPa: %.6g\n", PloadKPa);
fprintf(fid, "contact_mode_requested: %s\n", string(contactMode));
fprintf(fid, "linear_solver_mode_requested: %s\n", string(linearSolverMode));
fprintf(fid, "solver_coupling: %s\n", string(solverCoupling));
fprintf(fid, "ignite_ramp_enabled: %d\n", tern(igniteRamp > 0.5, 1, 0));
fprintf(fid, "disable_segregated: %d\n", disableSegregated);
fprintf(fid, "segregated_maxsegiter: %g\n", maxSegIter);
fprintf(fid, "segregated_maxsubiter: %g\n", maxSubIter);
fprintf(fid, "fully_coupled_maxiter: %g\n", fcMaxIter);
fclose(fid);

write_metrics_pressure_header(metricsPath);

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

% Disable displacement-control in z (avoid over-constraint) but keep x/y fixed.
dispTopOk = false;
try
    solid.feature('disp_top').active(true);
    solid.feature('disp_top').selection.set(bnd_rigid_top);
    solid.feature('disp_top').set('Direction', {'prescribed','prescribed','free'});
    solid.feature('disp_top').set('U0', {'0','0','0'});
    dispTopOk = true;
catch
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
try
    [segDisabled, segNote, fcEnabled, fcNote, fcMaxIterSet] = configure_solver_coupling(model, solverCoupling, maxSegIter, maxSubIter, fcMaxIter);
catch ME
    segDisabled = false;
    segNote = string(ME.message);
    fcEnabled = false;
    fcNote = string(ME.message);
    fcMaxIterSet = NaN;
end

% Ensure we actually use initial values from the loaded checkpoint.
st = model.study('std1').feature('stat');
try, st.set('initmethod', 'sol'); catch, end
try, st.set('initsol', 'current'); catch, end
try, st.set('useinitsol', 'on'); catch, end

% Ignite steps: either one shot (s=1.0) or a ramp s=[0.25 0.5 0.75 1.0]
if igniteRamp > 0.5
    sList = [0.25, 0.5, 0.75, 1.0];
else
    sList = 1.0;
end

exitStatus = "FAIL";
lastOkStepIdx = 0;
stepFailReason = "none";
stepFailIdx = NaN;
stepFailS = NaN;
stepFailP = NaN;

for stepIdx = 1:numel(sList)
    s = sList(stepIdx);
    PstepKPa = PloadKPa * s;
    stepT0 = tic;

    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
    fprintf(fid, "\nStepStart: idx=%d s=%.6g P_load_kPa=%.6g\n", stepIdx, s, PstepKPa);
    fclose(fid);

    % Set P_load for this step.
    try
        model.param.set('P_load', sprintf('%.6g[kPa]', PstepKPa));
    catch
        try, model.param.set('P_load', sprintf('%.6g*1e3[Pa]', PstepKPa)); catch, end
    end

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

    append_metrics_pressure_row(metricsPath, stepIdx, s, PstepKPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, tern(ok,'SUCCESS','FAIL'), errMsg, elapsedS, ...
        linearSolverMode, linearSwitched, linearNote, solverCoupling, segDisabled, segNote, fcEnabled, fcMaxIterSet, fcNote, contactMode, contactEffective, contactSupported, contactNote, checkpointSaved);

    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
    fprintf(fid, "StepEnd: idx=%d status=%s elapsed_s=%.6g checkpoint_saved=%d\n", stepIdx, tern(ok,'SUCCESS','FAIL'), elapsedS, tern(checkpointSaved,1,0));
    fclose(fid);

    if ok
        lastOkStepIdx = stepIdx;
        exitStatus = "SUCCESS";
        continue;
    end

    % First failed step => stop immediately (deterministic termination) after writing artifacts.
    stepFailReason = errMsg;
    stepFailIdx = stepIdx;
    stepFailS = s;
    stepFailP = PstepKPa;
    exitStatus = "FAIL";
    break;
end

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "\nsummary_stage: END\n");
fprintf(fid, "dcnt1_only_ok: %d\n", dcntOnlyOk);
fprintf(fid, "dcnt1_only_note: %s\n", sanitize_csv_text(string_or_none(dcntOnlyNote)));
fprintf(fid, "disp_top_xy_only_ok: %d\n", dispTopOk);
fprintf(fid, "pressure_load_ok: %d\n", pressureOk);
fprintf(fid, "solver_coupling: %s\n", sanitize_csv_text(string_or_none(solverCoupling)));
fprintf(fid, "ignite_ramp_enabled: %d\n", tern(igniteRamp > 0.5, 1, 0));
fprintf(fid, "last_success_step_idx: %d\n", lastOkStepIdx);
fprintf(fid, "linear_solver_mode: %s\n", sanitize_csv_text(string_or_none(linearSolverMode)));
fprintf(fid, "linear_solver_switched: %d\n", tern(linearSwitched,1,0));
fprintf(fid, "linear_solver_note: %s\n", sanitize_csv_text(string_or_none(linearNote)));
fprintf(fid, "segregated_disabled: %d\n", tern(segDisabled,1,0));
fprintf(fid, "segregated_note: %s\n", sanitize_csv_text(string_or_none(segNote)));
fprintf(fid, "fully_coupled_enabled: %d\n", tern(fcEnabled,1,0));
fprintf(fid, "fully_coupled_maxiter_set: %s\n", num2str(fcMaxIterSet));
fprintf(fid, "fully_coupled_note: %s\n", sanitize_csv_text(string_or_none(fcNote)));
fprintf(fid, "contact_mode_effective: %s\n", sanitize_csv_text(string_or_none(contactEffective)));
fprintf(fid, "contact_mode_supported: %d\n", tern(contactSupported,1,0));
fprintf(fid, "contact_mode_note: %s\n", sanitize_csv_text(string_or_none(contactNote)));
fprintf(fid, "exit_status: %s\n", sanitize_csv_text(exitStatus));
if exitStatus == "FAIL"
    fprintf(fid, "fail_step_idx: %s\n", num2str(stepFailIdx));
    fprintf(fid, "fail_step_s: %s\n", num2str(stepFailS));
    fprintf(fid, "fail_step_P_load_kPa: %s\n", num2str(stepFailP));
    fprintf(fid, "fail_reason: %s\n", sanitize_csv_text(string_or_none(stepFailReason)));
else
    fprintf(fid, "fail_reason: none\n");
end
fprintf(fid, "checkpoint_last_ok_exists: %d\n", tern(exist(checkpointOut,'file')==2,1,0));
fclose(fid);

if exitStatus == "FAIL"
    try
        payload = struct();
        payload.exit_status = 'FAIL';
        payload.P_load_kPa = PloadKPa;
        payload.ignite_ramp_enabled = tern(igniteRamp > 0.5, true, false);
        payload.last_success_step_idx = lastOkStepIdx;
        payload.fail_step_idx = stepFailIdx;
        payload.fail_step_s = stepFailS;
        payload.fail_step_P_load_kPa = stepFailP;
        payload.fail_reason = string_or_none(stepFailReason);
        payload.linear_solver_mode = string_or_none(linearSolverMode);
        payload.linear_solver_switched = linearSwitched;
        payload.linear_solver_note = string_or_none(linearNote);
        payload.solver_coupling = string_or_none(solverCoupling);
        payload.segregated_disabled = segDisabled;
        payload.segregated_note = string_or_none(segNote);
        payload.fully_coupled_enabled = fcEnabled;
        payload.fully_coupled_maxiter_set = fcMaxIterSet;
        payload.fully_coupled_note = string_or_none(fcNote);
        payload.contact_mode_requested = string_or_none(contactMode);
        payload.contact_mode_effective = string_or_none(contactEffective);
        payload.contact_mode_supported = contactSupported;
        payload.contact_mode_note = string_or_none(contactNote);
        txt = jsonencode(payload);
        fid = fopen(errorsPath, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', txt);
        fclose(fid);
    catch
    end
end

try, ModelUtil.remove('model'); catch, end %#ok<TRYNC>
end

function write_metrics_pressure_header(path)
if exist(path, 'file')
    return;
end
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'timestamp_iso,step_index,s,P_load_kPa,P_eff_Pa,Fz_top_int_N,A_top_m2,Ac_m2,Tn_max_Pa,pn_avg_Pa,Ac_method,success,exit_status,fail_reason,elapsed_s,linear_solver_mode,linear_solver_switched,linear_solver_note,solver_coupling,segregated_disabled,segregated_note,fully_coupled_enabled,fully_coupled_maxiter_set,fully_coupled_note,contact_mode_requested,contact_mode_effective,contact_mode_supported,contact_mode_note,checkpoint_saved\n');
fclose(fid);
end

function append_metrics_pressure_row(path, stepIdx, s, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, exitStatus, errMsg, elapsedS, linearSolverMode, switched, switchNote, solverCoupling, segDisabled, segNote, fcEnabled, fcMaxIterSet, fcNote, contactMode, contactEffective, contactSupported, contactNote, checkpointSaved)
fid = fopen(path, 'a', 'n', 'UTF-8');
ts = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
fprintf(fid, '%s,%d,%.6g,%.6g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s,%d,%s,%s,%.6g,%s,%d,%s,%s,%d,%s,%d,%.6g,%s,%s,%s,%d,%s,%d\n', ...
    ts, stepIdx, s, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, sanitize_csv_text(string_or_none(acWhy)), tern(ok,1,0), sanitize_csv_text(string_or_none(exitStatus)), sanitize_csv_text(string_or_none(errMsg)), elapsedS, ...
    sanitize_csv_text(string_or_none(linearSolverMode)), tern(switched,1,0), sanitize_csv_text(string_or_none(switchNote)), ...
    sanitize_csv_text(string_or_none(solverCoupling)), tern(segDisabled,1,0), sanitize_csv_text(string_or_none(segNote)), ...
    tern(fcEnabled,1,0), fcMaxIterSet, sanitize_csv_text(string_or_none(fcNote)), ...
    sanitize_csv_text(string_or_none(contactMode)), sanitize_csv_text(string_or_none(contactEffective)), tern(contactSupported,1,0), sanitize_csv_text(string_or_none(contactNote)), tern(checkpointSaved,1,0));
fclose(fid);
end

function [segDisabled, segNote, fcEnabled, fcNote, fcMaxIterSet] = configure_solver_coupling(model, solverCoupling, maxSegIter, maxSubIter, fcMaxIter)
segDisabled = false;
segNote = 'none';
fcEnabled = false;
fcNote = 'none';
fcMaxIterSet = NaN;
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
            return;
        end
    end
    fc1 = s1.feature('fc1');
    try, fc1.active(true); catch, end
    fcEnabled = true;
    fcNote = 'sol1/s1/fc1 enabled';

    % Cap nonlinear iterations for deterministic failure.
    try
        fc1.set('maxiter', fcMaxIter);
        fcMaxIterSet = fcMaxIter;
    catch
        fcMaxIterSet = NaN;
    end
    % Prefer direct solver definition.
    try, fc1.set('linsolver', 'dDef'); catch, end
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
