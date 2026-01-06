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
perSolveTimeoutS = 240;
globalTimeoutS = 900;
onsetBridgeMaxGapUm = 0.005;
bridgeNT = 6;
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

wTop = nan(size(deltaTargetsUm));
wBot = nan(size(deltaTargetsUm));
tnMax = nan(size(deltaTargetsUm));
Ac = nan(size(deltaTargetsUm));
pnAvg = nan(size(deltaTargetsUm));
FzPlate = nan(size(deltaTargetsUm));
targetSolved = false(size(deltaTargetsUm));

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "Template: %s\n", tpl);
try, fprintf(fid, "Lpyr: %s\n", char(model.param.get('Lpyr'))); catch, end
fprintf(fid, "delta_base_um: %g\n", deltaBaseUm);
fprintf(fid, "delta_indent_list_um: %s\n", mat2str(deltaIndentUm));
fprintf(fid, "delta_total_list_um: %s\n", mat2str(deltaTargetsUm));
fprintf(fid, "MinStep_um: %g\n", minStepUm);
fprintf(fid, "max_bisect_levels: %g\n", maxBisectLevels);
fprintf(fid, "min_indent_step_um: %g\n", minIndentStepUm);
fprintf(fid, "per_solve_timeout_s: %g\n", perSolveTimeoutS);
fprintf(fid, "global_timeout_s: %g\n", globalTimeoutS);
fprintf(fid, "onset_bridge_max_gap_um: %g\n", onsetBridgeMaxGapUm);
fprintf(fid, "MechanicsNote: plate driven by top-face displacement (bottom contact face not prescribed).\n");
fprintf(fid, "\nProgressLog:\n");
fclose(fid);

write_metrics_header(metricsPath);

% Start from a guaranteed-easy state: delta=0 (no contact), then continue upwards.
candUm = NaN;
cntWasActive = false;
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

            useInit = tern(hasSol,'on','off');
            model.param.set('delta', sprintf('%g[um]', candUm));
            cntActive = (candUm >= cntEnableUm);
            try, solid.feature('cnt1').active(cntActive); catch, end
            if cntActive && ~cntWasActive
                % Equation set changes when enabling contact; do not reuse the previous solution as-is.
                useInit = 'off';
            end
            try, st.set('useinitsol', useInit); catch, end %#ok<*TRYNC>

        if toc(globalT0) > globalTimeoutS
            error('Global timeout exceeded before solve: %.1fs > %.1fs', toc(globalT0), globalTimeoutS);
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

        if solveElapsed > perSolveTimeoutS
            ok = false;
            errMsg = sprintf('per_solve_timeout_s exceeded: %.1fs > %.1fs', solveElapsed, perSolveTimeoutS);
        end

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

            % Onset-cross fallback: try short TD bridge if gap is narrow.
            if abs(candUm - prevDeltaUm) <= onsetBridgeMaxGapUm
                bridgeAttemptId = bridgeAttemptId + 1;
                [tdOk, tdElapsed, tdErrMsg] = attempt_td_bridge(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, prevDeltaUm, candUm, bridgeNT, tdBridgePath, bridgeAttemptId);
                fallbackAttempts(end+1) = make_attempt('td_bridge', deltaBaseUm, candIndent, candUm, tdElapsed, tdOk, tdErrMsg); %#ok<AGROW>
                if tdOk
                    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
                    fprintf(fid, "    td-bridge OK from %g to %g um\n", prevDeltaUm, candUm);
                    fclose(fid);

                    hasSol = true;
                    prevDeltaUm = candUm;
                    cntWasActive = (candUm >= cntEnableUm);
                    deltaHistoryUm(end+1) = candUm; %#ok<AGROW>
                    lastSuccessDeltaUm = candUm;
                    bisectLevelsUsed = 0;

                    metrics = collect_metrics(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval);
                    append_metrics_row(metricsPath, candUm, deltaBaseUm, metrics);
                    try, mphsave(model, checkpointMph); catch, end

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
                    pending(1) = [];
                    continue;
                else
                    error('td_bridge_failed: %s', tdErrMsg);
                end
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
fclose(fid);

% Ensure baseline_results.csv is written even on failure.
try
    write_baseline_results(baselineCsvPath, deltaBaseUm, deltaIndentUm, deltaTargetsUm, targetSolved, tnMax, Ac, pnAvg, FzPlate, failDeltaUm, failReason);
catch
end

if ~isempty(err)
    try
        write_errors_json(errorsPath, err, deltaBaseUm, failDeltaIndentUm, failDeltaUm);
    catch
    end
end
try
    write_fallback_report(fallbackReportPath, fallbackAttempts);
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

function write_errors_json(path, err, deltaBaseUm, deltaIndentUm, deltaTotalUm)
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

function write_fallback_report(path, attempts)
payload = struct();
payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
payload.attempts = attempts;
txt = jsonencode(payload);
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', txt);
fclose(fid);
end

function [ok, elapsed, errMsg] = attempt_td_bridge(model, bnd_rigid_top, bnd_rigid_bot, bnd_eval, deltaStartUm, deltaEndUm, bridgeNT, tdBridgePath, attemptId)
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
    dt = 1 / (bridgeNT - 1);
    td.set('tlist', sprintf('range(0,%g,1)', dt));
    try, td.set('useinitsol', 'on'); catch, end
    try, td.set('initmethod', 'sol'); catch, end
    try, td.set('solnum', 'last'); catch, end

    deltaExpr = sprintf('%g[um] + (%g[um]-%g[um])*(0.5*(1-cos(pi*t)))', deltaStartUm, deltaEndUm, deltaStartUm);
    model.param.set('delta', deltaExpr);

    t0 = tic;
    model.study(stdTag).run();
    elapsed = toc(t0);
    ok = true;

    write_td_bridge_results(tdBridgePath, attemptId, deltaStartUm, deltaEndUm, bridgeNT, bnd_eval, bnd_rigid_top, model);
catch ME
    errMsg = string(ME.message);
    append_td_bridge_failure(tdBridgePath, attemptId, deltaEndUm);
end

try
    model.param.set('delta', sprintf('%g[um]', deltaEndUm));
catch
end

if ~ok && strlength(string(errMsg)) == 0
    errMsg = 'td_bridge_failed';
end
end

function ensure_td_bridge_header(path)
if exist(path, 'file')
    return;
end
fid = fopen(path, 'w', 'n', 'UTF-8');
fprintf(fid, 'attempt_id,t,delta_total_um,Ac_m2,pn_avg_Pa,Fz_plate_top_int_N\n');
fclose(fid);
end

function write_td_bridge_results(path, attemptId, deltaStartUm, deltaEndUm, bridgeNT, bnd_eval, bnd_rigid_top, model)
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
    fprintf(fid, '%d,%.6g,%.6g,%.6g,%.6g,%.6g\n', attemptId, t, deltaUm, Ac, pnAvg, Fz);
end
fclose(fid);
end

function append_td_bridge_failure(path, attemptId, deltaEndUm)
fid = fopen(path, 'a', 'n', 'UTF-8');
fprintf(fid, '%d,NaN,%.6g,NaN,NaN,NaN\n', attemptId, deltaEndUm);
fclose(fid);
end
