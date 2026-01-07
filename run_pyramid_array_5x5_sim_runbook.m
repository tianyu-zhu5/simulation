function run_pyramid_array_5x5_sim_runbook()
%RUN_PYRAMID_ARRAY_5X5_SIM_RUNBOOK Solve the 5x5 pyramid array model using the RUNBOOK strategy.
%
% Strategy (see RUNBOOK_PYRAMID_SOLVE.md + suggestion.md):
% - Use a single Solid Mechanics contact feature cnt1 (Contact) explicitly bound to contact pair pc
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

summaryPath = fullfile(simDir, 'Pyramid_5x5_summary.txt');
metricsPath = fullfile(simDir, 'Pyramid_5x5_metrics.csv');
checkpointMph = fullfile(simDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
errorsPath = fullfile(simDir, 'errors.json');
fallbackReportPath = fullfile(simDir, 'fallback_report.json');
tdBridgePath = fullfile(simDir, 'td_bridge_results.csv');
ptcBridgePath = fullfile(simDir, 'ptc_bridge_results.csv');
baselineCsvPath = fullfile(pwd, 'baseline_results.csv');
outMph = fullfile(simDir, 'Pyramid_5x5_solved.mph');

model = mphload(tpl);
model.hist.disable();

comp = model.component('comp1');
solid = comp.physics('solid');

% Representative geometry (keep Lpyr fixed for now; sweep later)
try, model.param.set('Lpyr', '10[um]'); catch, end

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
perSolveTimeoutPostS = 240;
segmentedMaxStepUmPost = 0.0005;
segmentedMaxStagesPost = 5;
globalTimeoutS = 2400;
onsetBridgeMaxGapUm = 0.01;
preBisectLevels = 2;
ptcTimeStep = 0.05;
ptcMaxSteps = 50;
ptcDamping = 0.5;
bridgeNT = 25;
bridgeDt = 0.05;
bridgeModeDefault = 'PTC';
allowRampFallback = false;
budgetPreOnsetS = 300;
budgetPostOnsetS = 900;
FzEps = 1e-9;
phase1TargetUm = 1.0227;
microTargetsUm = [1.0205 1.0210 1.0215 1.0220 1.0223 1.0225 1.0227 1.0230 1.0235 1.0240 1.0250];
try, model.param.set('delta', '0[um]'); catch, end
try, model.param.set('P_load', '0[Pa]'); catch, end

% Ensure contact uses cnt1 over pair pc (single source of truth)
try
    cnt = solid.feature('cnt1');
    cnt.set('pairSelection', 'list');
    cnt.set('pairs', {'pc'});
    cnt.set('ContactMethodCtrl', 'Penalty');
    cnt.set('useCutback', 1);
    cnt.set('useRelaxation', 'Conditional');
    cnt.set('penaltyCtrl', 'userDefined');
    cnt.set('pn_penalty', '0.01*solid.cnt1.E_char/solid.hmin_dst');
    cnt.set('ContactTolType', 'Manual');
    cnt.set('tolcontact', '1e-6');
catch
end
% Neutralize dcnt1 (it exists by default and cannot be disabled in this COMSOL setup).
try
    dcnt0 = solid.feature('dcnt1');
    dcnt0.set('pairSelection', 'list');
    dcnt0.set('pairs', javaArray('java.lang.String', 0));
catch
end

% Practical continuation trick:
% keep contact disabled during the initial approach, then enable it slightly after first touch.
% (cnt1 can be hard to initialize when enabled too early with a positive gap.)
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

% Mesh: keep the template default (the 5x5 model can be very large).
% IMPORTANT: explicitly build the mesh and fail fast if meshing is incomplete.
try
    comp.mesh('mesh1').run;
    ms = mphmeshstats(model);
    if isfield(ms, 'isempty') && ms.isempty
        error('Mesh is empty after mesh1.run().');
    end
    if isfield(ms, 'hasproblems') && ms.hasproblems
        error('Mesh has problems after mesh1.run().');
    end
catch ME
    error('Mesh build failed: %s', string(ME.message));
end

% Solve via continuation
% Evaluate contact quantities on the contact destination boundaries.
bnd_eval = pc.destination.entities;

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
deltaPlanMode = 'pre_onset';
contactOnsetDetected = false;
contactNote = '';
budgetUsedPre = 0;
budgetUsedPost = 0;
earlyExit = false;
earlyExitReason = '';
reserveGateTriggered = false;
reserveGateReason = '';

wTop = nan(size(deltaTargetsUm));
wBot = nan(size(deltaTargetsUm));
tnMax = nan(size(deltaTargetsUm));
Ac = nan(size(deltaTargetsUm));
pnAvg = nan(size(deltaTargetsUm));
FzPlate = nan(size(deltaTargetsUm));
targetSolved = false(size(deltaTargetsUm));

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "Template: %s\n", tpl);
fprintf(fid, "git_commit_hash: %s\n", gitCommitHash);
fprintf(fid, "bridge_policy: %s\n", bridgePolicy);
fprintf(fid, "delta_plan_mode: %s\n", deltaPlanMode);
fprintf(fid, "micro_targets: %s\n", mat2str(microTargetsUm));
fprintf(fid, "Fz_eps: %g\n", FzEps);
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
fprintf(fid, "allow_ramp_fallback: %d\n", allowRampFallback);
fprintf(fid, "MechanicsNote: plate driven by top-face displacement (bottom contact face not prescribed).\n");
fprintf(fid, "\nProgressLog:\n");
fclose(fid);

write_metrics_header(metricsPath);

% Start from a guaranteed-easy state: delta=0 (no contact), then continue upwards.
candUm = NaN;
cntWasActive = false;
switchToMicro = false;
lastFzPlate = NaN;
try
    try, st.set('useinitsol', 'off'); catch, end
    model.param.set('delta', '0[um]');
    try, solid.feature('cnt1').active(false); catch, end
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

metrics0 = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
append_metrics_row(metricsPath, 0.0, deltaBaseUm, metrics0);
    lastSuccessDeltaUm = 0.0;
    try, mphsave(model, checkpointMph); catch, end

    % Add a coarse ramp up to the first target to improve robustness.
    deltaRampUm = unique([0, 0.5*gap0_um, 0.8*gap0_um, gap0_um, cntEnableUm, deltaBaseUm, deltaTargetsUm], 'stable');

    for iT = 1:numel(deltaRampUm)
        targetUm = deltaRampUm(iT);
        pending = targetUm; % queue of deltas to solve (midpoints inserted on failure)
        while ~isempty(pending)
            candUm = pending(1);
            stepUm = candUm - prevDeltaUm;
            if stepUm < 0
                error('Non-monotone continuation: prev=%g um, cand=%g um', prevDeltaUm, candUm);
            end

        [cntActive, useInit] = prepare_stationary_step(model, solid, st, candUm, cntEnableUm, cntWasActive, hasSol);

        if toc(globalT0) > globalTimeoutS
            error('Global timeout exceeded before solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
        end
        ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);

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

        if toc(globalT0) > globalTimeoutS
            error('Global timeout exceeded after solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
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
                        contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                    end
                end
                if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                    deltaPlanMode = 'post_onset_micro';
                    switchToMicro = true;
                end
                if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= phase1TargetUm
                    earlyExit = true;
                    earlyExitReason = 'Reached Phase1 target >=1.0227';
                end
                if switchToMicro || earlyExit
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
                    error('Global timeout exceeded during pre-bisect: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                end

                [cntActive, useInit] = prepare_stationary_step(model, solid, st, midUm, cntEnableUm, cntWasActive, hasSol);
                ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
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
                    try, solid.feature('cnt1').active(true); catch, end
                    try, st.set('useinitsol', 'off'); catch, end
                    ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    t0 = tic;
                    model.study('std1').run();
                    elapsedPreAct = toc(t0);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsedPreAct, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    fallbackAttempts(end+1) = make_attempt('pre_activate', deltaBaseUm, preBisectLastOk - deltaBaseUm, preBisectLastOk, elapsedPreAct, true, ''); %#ok<AGROW>
                    cntWasActive = true;
                catch ME
                    elapsedPreAct = 0;
                    fallbackAttempts(end+1) = make_attempt('pre_activate', deltaBaseUm, preBisectLastOk - deltaBaseUm, preBisectLastOk, elapsedPreAct, false, string(ME.message)); %#ok<AGROW>
                end
            end

            % Onset-cross fallback: PTC -> segmented stationary -> TD_RELAX (last fallback).
            if abs(preBisectFail - preBisectLastOk) <= onsetBridgeMaxGapUm
                % Bridge Mode 1: Stationary PTC
                bridgeModeUsed = 'PTC';
                ptcAttempted = true;
                bridgeAttemptId = bridgeAttemptId + 1;
                ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                [reserveGateTriggered, reserveGateReason] = check_reserve_gate(globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                if reserveGateTriggered
                    failDeltaUm = preBisectFail;
                    failDeltaIndentUm = preBisectFail - deltaBaseUm;
                    failReason = reserveGateReason;
                    error('%s', reserveGateReason);
                end
                [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, preBisectLastOk, preBisectFail, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
                [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                fallbackAttempts(end+1) = make_attempt('ptc', deltaBaseUm, preBisectFail - deltaBaseUm, preBisectFail, ptcElapsed, ptcOk, ptcErrMsg); %#ok<AGROW>
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
                            contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                        end
                    end
                    if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                        deltaPlanMode = 'post_onset_micro';
                        switchToMicro = true;
                    end
                    if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= phase1TargetUm
                        earlyExit = true;
                        earlyExitReason = 'Reached Phase1 target >=1.0227';
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

                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [cntActive, useInit] = prepare_stationary_step(model, solid, st, segTarget, cntEnableUm, cntWasActive, hasSol);
                    ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
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
                    ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, segLastOk, segTarget, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    segmentedAttempts(end+1) = make_segment_attempt(stageId, segTarget, 'ptc', ptcOk, ptcElapsed, ptcErrMsg); %#ok<AGROW>
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
                                contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                            end
                        end
                    catch
                    end
                    if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                        deltaPlanMode = 'post_onset_micro';
                        switchToMicro = true;
                    end
                    if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= phase1TargetUm
                        earlyExit = true;
                        earlyExitReason = 'Reached Phase1 target >=1.0227';
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
                    ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    [reserveGateTriggered, reserveGateReason] = check_reserve_gate(globalT0, globalTimeoutS, budgetPostOnsetS, reserveGateTriggered, reserveGateReason);
                    if reserveGateTriggered
                        failDeltaUm = preBisectFail;
                        failDeltaIndentUm = preBisectFail - deltaBaseUm;
                        failReason = reserveGateReason;
                        error('%s', reserveGateReason);
                    end
                    [tdOk, tdElapsed, tdErrMsg] = attempt_td_bridge('TD_RELAX', model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, segLastOk, preBisectFail, bridgeNT, bridgeDt, tdBridgePath, bridgeAttemptId);
                    [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, tdElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
                    fallbackAttempts(end+1) = make_attempt('td_relax', deltaBaseUm, preBisectFail - deltaBaseUm, preBisectFail, tdElapsed, tdOk, tdErrMsg); %#ok<AGROW>
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
                                contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                            end
                        end
                        if strcmp(deltaPlanMode, 'pre_onset') && (lastSuccessDeltaUm >= 1.02 || contactOnsetDetected)
                            deltaPlanMode = 'post_onset_micro';
                            switchToMicro = true;
                        end
                        if strcmp(deltaPlanMode, 'post_onset_micro') && lastSuccessDeltaUm >= phase1TargetUm
                            earlyExit = true;
                            earlyExitReason = 'Reached Phase1 target >=1.0227';
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
    if ~earlyExit && strcmp(deltaPlanMode, 'post_onset_micro')
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "  - switch to post_onset_micro targets\n");
        fclose(fid);
        for iM = 1:numel(microTargetsUm)
            targetUm = microTargetsUm(iM);
            if earlyExit
                break;
            end
            if targetUm <= lastSuccessDeltaUm + 1e-12
                continue;
            end
            prevDeltaBefore = prevDeltaUm;
            candUm = targetUm;
            if toc(globalT0) > globalTimeoutS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded before post_onset solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);

            [cntActive, useInit] = prepare_stationary_step(model, solid, st, targetUm, cntEnableUm, cntWasActive, hasSol);
            fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
            fprintf(fid, "  - post_onset try delta=%g um (useinitsol=%s)\n", targetUm, useInit);
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
            elapsed = toc(t0);
            solveTime = solveTime + elapsed;
            perSolveTimeoutS = get_per_solve_timeout(deltaPlanMode, perSolveTimeoutPreS, perSolveTimeoutPostS);
            if elapsed > perSolveTimeoutS
                ok = false;
                errMsg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', elapsed, perSolveTimeoutS);
            end
            [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, elapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'stationary', ok, elapsed, errMsg); %#ok<AGROW>
            if toc(globalT0) > globalTimeoutS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded after post_onset solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if ~ok && strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = 'post_onset budget exhausted before reaching 1.0227';
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('post_onset budget exhausted before reaching 1.0227');
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
                        contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                    end
                end
                if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < phase1TargetUm
                    failDeltaUm = targetUm;
                    failDeltaIndentUm = targetUm - deltaBaseUm;
                    failReason = 'post_onset budget exhausted before reaching 1.0227';
                    narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                    error('post_onset budget exhausted before reaching 1.0227');
                end
                if lastSuccessDeltaUm >= phase1TargetUm
                    earlyExit = true;
                    earlyExitReason = 'Reached Phase1 target >=1.0227';
                    break;
                end
                continue;
            end

            % Stationary failed -> PTC
            ptcAttempted = true;
            bridgeAttemptId = bridgeAttemptId + 1;
            ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            [ptcOk, ptcElapsed, ptcErrMsg] = attempt_ptc_bridge(model, solid, st, prevDeltaUm, targetUm, cntEnableUm, ptcTimeStep, ptcMaxSteps, ptcDamping, ptcBridgePath, bridgeAttemptId, bnd_eval, bnd_rigid_top);
            [budgetUsedPre, budgetUsedPost] = consume_budget(deltaPlanMode, ptcElapsed, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
            microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'PTC', ptcOk, ptcElapsed, ptcErrMsg); %#ok<AGROW>
            if toc(globalT0) > globalTimeoutS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded after post_onset PTC: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if ~ptcOk && strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = 'post_onset budget exhausted before reaching 1.0227';
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('post_onset budget exhausted before reaching 1.0227');
            end

            if ptcOk
                fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                fprintf(fid, "    post_onset PTC OK at delta=%g um\n", targetUm);
                fclose(fid);

                hasSol = true;
                prevDeltaUm = targetUm;
                cntWasActive = (prevDeltaUm >= cntEnableUm);
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
                        contactNote = 'Force indicates onset but dcnt1 undefined/NaN';
                    end
                end
                if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < phase1TargetUm
                    failDeltaUm = targetUm;
                    failDeltaIndentUm = targetUm - deltaBaseUm;
                    failReason = 'post_onset budget exhausted before reaching 1.0227';
                    narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                    error('post_onset budget exhausted before reaching 1.0227');
                end
                if lastSuccessDeltaUm >= phase1TargetUm
                    earlyExit = true;
                    earlyExitReason = 'Reached Phase1 target >=1.0227';
                    break;
                end
                continue;
            end

            % Segmented fallback (stationary + per-stage PTC)
            segmentedAttempted = true;
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
                ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
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
                ensure_budget(deltaPlanMode, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS);
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

            microTargetAttempts(end+1) = make_micro_attempt(targetUm, 'segmented', segOk, segElapsedTotal, segErrMsg); %#ok<AGROW>
            if toc(globalT0) > globalTimeoutS
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = sprintf('Global timeout exceeded after post_onset segmented: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('%s', failReason);
            end
            if strcmp(deltaPlanMode, 'post_onset_micro') && budgetUsedPost > budgetPostOnsetS && lastSuccessDeltaUm < phase1TargetUm
                failDeltaUm = targetUm;
                failDeltaIndentUm = targetUm - deltaBaseUm;
                failReason = 'post_onset budget exhausted before reaching 1.0227';
                narrowFailIntervalUm = [prevDeltaBefore, targetUm];
                error('post_onset budget exhausted before reaching 1.0227');
            end
            if segOk
                if lastSuccessDeltaUm >= phase1TargetUm
                    earlyExit = true;
                    earlyExitReason = 'Reached Phase1 target >=1.0227';
                    break;
                end
                continue;
            end

            segmentedFailReason = segErrMsg;
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
end

% Export summary (arrays over target deltas)
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
fprintf(fid, "fail_delta_total_um: %s\n", num2str(failDeltaUm));
fprintf(fid, "fail_reason: %s\n", string_or_none(failReason));
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
fprintf(fid, "early_exit: %d\n", earlyExit);
fprintf(fid, "early_exit_reason: %s\n", string_or_none(earlyExitReason));
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
    write_fallback_report(fallbackReportPath, fallbackAttempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, toc(globalT0), globalTimeoutS - toc(globalT0));
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

function metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval)
metrics = struct('wTop', NaN, 'wBot', NaN, 'tnMax', NaN, 'Ac', 0, 'pnAvg', NaN, 'FzPlate', NaN);
try, metrics.wTop = mphmin(model, 'w', 'surface', 'selection', bnd_rigid_top); catch, end
try, metrics.wBot = mphmin(model, 'w', 'surface', 'selection', bnd_rigid_bot); catch, end
try
    metrics.tnMax = mphmax(model, 'solid.dcnt1.Tn', 'surface', 'selection', bnd_eval);
    metrics.Ac = mphint2(model, 'if(solid.dcnt1.Tn>0,1,0)', 'surface', 'selection', bnd_eval);
    pnInt = mphint2(model, 'solid.dcnt1.Tn', 'surface', 'selection', bnd_eval);
    if isfinite(metrics.Ac) && metrics.Ac > 0
        metrics.pnAvg = pnInt ./ metrics.Ac;
    else
        metrics.pnAvg = NaN;
    end
catch
    metrics.tnMax = NaN;
    metrics.Ac = 0;
    metrics.pnAvg = NaN;
end
try, metrics.FzPlate = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_rigid_top); catch, end
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

function ensure_budget(mode, usedPre, usedPost, budgetPre, budgetPost)
if strcmp(mode, 'post_onset_micro')
    if usedPost >= budgetPost
        error('post_onset budget exhausted before reaching 1.0227');
    end
end
end

function [cntActive, useInit] = prepare_stationary_step(model, solid, st, candUm, cntEnableUm, cntWasActive, hasSol)
useInit = tern(hasSol,'on','off');
try, st.set('useinitsol', useInit); catch, end
try, solid.feature('cnt1').active(false); catch, end
try, model.param.set('delta', sprintf('%g[um]', candUm)); catch, end
cntActive = (candUm >= cntEnableUm);
try, solid.feature('cnt1').active(cntActive); catch, end
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

function [triggered, reason] = check_reserve_gate(globalT0, globalTimeoutS, budgetPostOnsetS, triggered, reason)
timeRemaining = globalTimeoutS - toc(globalT0);
if timeRemaining < budgetPostOnsetS
    triggered = true;
    reason = 'pre_onset time reserve exhausted (preserving post budget)';
end
end

function write_fallback_report(path, attempts, preBisectAttempts, preBisectLastOk, preBisectFail, segmentedAttempts, microTargetAttempts, bridgePolicy, deltaPlanMode, microTargetsUm, budgetUsedPre, budgetUsedPost, budgetPreOnsetS, budgetPostOnsetS, reserveGateTriggered, timeElapsedS, timeRemainingS)
payload = struct();
payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
payload.attempts = attempts;
payload.pre_bisect_attempts = preBisectAttempts;
payload.pre_bisect_last_ok_um = preBisectLastOk;
payload.pre_bisect_fail_um = preBisectFail;
payload.segmented_attempts = segmentedAttempts;
payload.micro_target_attempts = microTargetAttempts;
payload.bridge_policy = bridgePolicy;
payload.delta_plan_mode = deltaPlanMode;
payload.micro_targets = microTargetsUm;
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
        write_td_bridge_results(tdBridgePath, attemptId, mode, deltaStartUm, deltaEndUm, bridgeNT, bnd_eval, bnd_rigid_top, model);
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
    try, solid.feature('cnt1').active(true); catch, end
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
    Ac = mphint2(model, 'if(solid.dcnt1.Tn>0,1,0)', 'surface', 'selection', bnd_eval);
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

function write_td_bridge_results(path, attemptId, mode, deltaStartUm, deltaEndUm, bridgeNT, bnd_eval, bnd_rigid_top, model)
tlist = linspace(0, 1, bridgeNT);
fid = fopen(path, 'a', 'n', 'UTF-8');
for i = 1:numel(tlist)
    t = tlist(i);
    ramp = 0.5 * (1 - cos(pi * t));
    deltaUm = deltaStartUm + (deltaEndUm - deltaStartUm) * ramp;
    Ac = NaN;
    pnAvg = NaN;
    Fz = NaN;
    try
        Ac = mphint2(model, 'if(solid.dcnt1.Tn>0,1,0)', 'surface', 'selection', bnd_eval, 't', t);
        pnInt = mphint2(model, 'solid.dcnt1.Tn', 'surface', 'selection', bnd_eval, 't', t);
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
