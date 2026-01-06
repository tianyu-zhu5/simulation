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
    solveTime = solveTime + toc(t0);
    hasSol = true;
    prevDeltaUm = 0.0;
    cntWasActive = false;
    deltaHistoryUm(end+1) = 0.0; %#ok<AGROW>
    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
    fprintf(fid, "    initial OK at delta=0 um\n");
    fclose(fid);

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

            fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
            fprintf(fid, "  - try delta=%g um (step=%g um, useinitsol=%s)\n", candUm, stepUm, useInit);
            fclose(fid);

            t0 = tic;
            ok = true;
            try
                model.study('std1').run();
            catch ME
                ok = false;
            end
            solveTime = solveTime + toc(t0);

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
                fprintf(fid, "    solve FAILED at delta=%g um: %s\n", candUm, string(ME.message));
                fclose(fid);

                if stepUm <= minStepUm
                    rethrow(ME);
                end
                midUm = prevDeltaUm + stepUm/2;
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
