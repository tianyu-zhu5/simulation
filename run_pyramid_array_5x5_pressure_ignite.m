function run_pyramid_array_5x5_pressure_ignite()
%RUN_PYRAMID_ARRAY_5X5_PRESSURE_IGNITE Pressure-control "ignite" from an existing contact checkpoint.
%
% Goal: validate that pressure-control is feasible without over/under-constraint by:
% - Loading an existing displacement-control contact solution checkpoint as initial values
% - Disabling z-displacement prescription on the rigid plate top (avoid over-constraint)
% - Applying pressure load P_load (downwards) on the rigid plate top boundary
% - Switching to fully-coupled (disable segregated) + Direct(PARDISO) linear solver
% - Producing a single-row metrics file for RP: Ac(P), etc.
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
disableSegregated = true;
try
    ds = getenv('SIM_DISABLE_SEGREGATED');
    if ~isempty(strtrim(ds))
        disableSegregated = strcmpi(strtrim(ds), '1') || strcmpi(strtrim(ds), 'true');
    end
catch
end

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "summary_stage: START\n");
fprintf(fid, "pressure_ignite_from_sim_dir: %s\n", string(fromSimDir));
fprintf(fid, "resume_checkpoint: %s\n", string(ckIn));
fprintf(fid, "P_load_kPa: %.6g\n", PloadKPa);
fprintf(fid, "contact_mode_requested: %s\n", string(contactMode));
fprintf(fid, "linear_solver_mode_requested: %s\n", string(linearSolverMode));
fprintf(fid, "disable_segregated: %d\n", disableSegregated);
fclose(fid);

write_metrics_pressure_header(metricsPath);

t0 = tic;
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
    model.param.set('P_load', sprintf('%.6g[kPa]', PloadKPa));
catch
    try, model.param.set('P_load', sprintf('%.6g*1e3[Pa]', PloadKPa)); catch, end
end
try
    bndl = solid.feature('bndl1');
    bndl.set('forceType', 'FollowerPressure');
    bndl.set('pressure', '-P_load');
    bndl.active(true);
    pressureOk = true;
catch
end

% Solver: attempt to disable segregated and force Direct(PARDISO).
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
try
    if disableSegregated
        s1 = model.sol('sol1').feature('s1');
        try
            s1.feature('se1').active(false);
            segDisabled = true;
            segNote = 'sol1/s1/se1 deactivated';
        catch ME
            segDisabled = false;
            segNote = string(ME.message);
        end
    end
catch ME
    segDisabled = false;
    segNote = string(ME.message);
end

% Ensure we actually use initial values from the loaded checkpoint.
st = model.study('std1').feature('stat');
try, st.set('initmethod', 'sol'); catch, end
try, st.set('initsol', 'current'); catch, end
try, st.set('useinitsol', 'on'); catch, end

ok = false;
errMsg = 'none';
try
    model.study('std1').run();
    ok = true;
catch ME
    ok = false;
    errMsg = string(ME.message);
end
elapsedS = toc(t0);

ATop = NaN;
Fz = NaN;
PEff = NaN;
tnMax = NaN;
Ac = 0;
pnAvg = NaN;
acWhy = 'none';
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
end

append_metrics_pressure_row(metricsPath, PloadKPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, errMsg, elapsedS, linearSolverMode, linearSwitched, linearNote, contactMode, contactEffective, contactSupported, contactNote);

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "\nsummary_stage: END\n");
fprintf(fid, "dcnt1_only_ok: %d\n", dcntOnlyOk);
fprintf(fid, "dcnt1_only_note: %s\n", sanitize_csv_text(string_or_none(dcntOnlyNote)));
fprintf(fid, "disp_top_xy_only_ok: %d\n", dispTopOk);
fprintf(fid, "pressure_load_ok: %d\n", pressureOk);
fprintf(fid, "segregated_disabled: %d\n", segDisabled);
fprintf(fid, "segregated_note: %s\n", sanitize_csv_text(string_or_none(segNote)));
fprintf(fid, "linear_solver_mode: %s\n", sanitize_csv_text(string_or_none(linearSolverMode)));
fprintf(fid, "linear_solver_switched: %d\n", tern(linearSwitched,1,0));
fprintf(fid, "linear_solver_note: %s\n", sanitize_csv_text(string_or_none(linearNote)));
fprintf(fid, "contact_mode_effective: %s\n", sanitize_csv_text(string_or_none(contactEffective)));
fprintf(fid, "contact_mode_supported: %d\n", tern(contactSupported,1,0));
fprintf(fid, "contact_mode_note: %s\n", sanitize_csv_text(string_or_none(contactNote)));
fprintf(fid, "exit_status: %s\n", tern(ok,"SUCCESS","FAIL"));
fprintf(fid, "elapsed_s: %.6g\n", elapsedS);
fprintf(fid, "P_eff_Pa: %.9g\n", PEff);
fprintf(fid, "Fz_top_int_N: %.9g\n", Fz);
fprintf(fid, "Ac_m2: %.9g\n", Ac);
fprintf(fid, "Tn_max_Pa: %.9g\n", tnMax);
fprintf(fid, "pn_avg_Pa: %.9g\n", pnAvg);
fprintf(fid, "Ac_method: %s\n", sanitize_csv_text(string_or_none(acWhy)));
fprintf(fid, "fail_reason: %s\n", sanitize_csv_text(string_or_none(tern(ok,"none",errMsg))));
fclose(fid);

if ok
    try, model.save(checkpointOut); catch, end
else
    try
        payload = struct();
        payload.exit_status = 'FAIL';
        payload.P_load_kPa = PloadKPa;
        payload.elapsed_s = elapsedS;
        payload.fail_reason = string_or_none(errMsg);
        payload.linear_solver_mode = string_or_none(linearSolverMode);
        payload.linear_solver_switched = linearSwitched;
        payload.linear_solver_note = string_or_none(linearNote);
        payload.segregated_disabled = segDisabled;
        payload.segregated_note = string_or_none(segNote);
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
fprintf(fid, 'timestamp_iso,P_load_kPa,P_eff_Pa,Fz_top_int_N,A_top_m2,Ac_m2,Tn_max_Pa,pn_avg_Pa,Ac_method,success,fail_reason,elapsed_s,linear_solver_mode,linear_solver_switched,linear_solver_note,contact_mode_requested,contact_mode_effective,contact_mode_supported,contact_mode_note\n');
fclose(fid);
end

function append_metrics_pressure_row(path, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, errMsg, elapsedS, linearSolverMode, switched, switchNote, contactMode, contactEffective, contactSupported, contactNote)
fid = fopen(path, 'a', 'n', 'UTF-8');
ts = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
fprintf(fid, '%s,%.6g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s,%d,%s,%.6g,%s,%d,%s,%s,%s,%d,%s\n', ...
    ts, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, sanitize_csv_text(string_or_none(acWhy)), tern(ok,1,0), sanitize_csv_text(string_or_none(errMsg)), elapsedS, ...
    sanitize_csv_text(string_or_none(linearSolverMode)), tern(switched,1,0), sanitize_csv_text(string_or_none(switchNote)), ...
    sanitize_csv_text(string_or_none(contactMode)), sanitize_csv_text(string_or_none(contactEffective)), tern(contactSupported,1,0), sanitize_csv_text(string_or_none(contactNote)));
fclose(fid);
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

