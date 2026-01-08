function run_pyramid_array_5x5_pressure_sweep()
%RUN_PYRAMID_ARRAY_5X5_PRESSURE_SWEEP Minimal pressure-controlled sweep prototype (3 points).
%
% This script is intended to generate Ac(P) data without attempting displacement-control
% continuation beyond the delta ~ 1.023 onset wall.
%
% Outputs (by default):
%   out/pyramid_5x5/pressure_sweep_YYYYMMDD_HHMMSS/metrics_pressure.csv
%   out/pyramid_5x5/pressure_sweep_YYYYMMDD_HHMMSS/errors_P*.json (on failure)
%   out/pyramid_5x5/pressure_sweep_YYYYMMDD_HHMMSS/checkpoint_P*.mph (optional)
%   out/pyramid_5x5/pressure_sweep_YYYYMMDD_HHMMSS/checkpoint_last_ok.mph (on success)

import com.comsol.model.util.*

tpl = fullfile(pwd, 'out', 'pyramid_5x5', 'Pyramid_5x5_Mech.mph');
tplEnv = getenv('SIM_PRESSURE_TEMPLATE_MPH');
if ~isempty(tplEnv)
    tpl = tplEnv;
end
if ~exist(tpl, 'file')
    error('Missing template: %s', tpl);
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>

outRoot = fullfile(pwd, 'out', 'pyramid_5x5');
if ~exist(outRoot, 'dir')
    mkdir(outRoot);
end

sweepDir = getenv('PRESSURE_SWEEP_DIR');
if isempty(sweepDir)
    sweepDir = fullfile(outRoot, ['pressure_sweep_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~exist(sweepDir, 'dir')
    mkdir(sweepDir);
end

metricsPath = fullfile(sweepDir, 'metrics_pressure.csv');

linearSolverMode = get_env_or_default('SIM_LINEAR_SOLVER_MODE', 'default');
contactMode = get_env_or_default('PHASE2_CONTACT_MODE', 'augmented_lagrange');
tnEpsPa = numeric_env('SIM_TN_EPS_PA', 1.0);

pointKpaEnv = getenv('SIM_P_LOAD_KPA');
if ~isempty(pointKpaEnv)
    PkPaList = str2double(pointKpaEnv);
    if ~isfinite(PkPaList)
        error('Invalid SIM_P_LOAD_KPA: %s', pointKpaEnv);
    end
else
    PkPaList = [0.3, 0.6, 1.0];
end

write_metrics_pressure_header(metricsPath);

for i = 1:numel(PkPaList)
    PkPa = PkPaList(i);
    attemptTag = pressure_tag(PkPa);
    pointT0 = tic;

    % Best-effort resume from last successful checkpoint within this sweepDir.
    modelSource = tpl;
    ckLastOk = fullfile(sweepDir, 'checkpoint_last_ok.mph');
    if exist(ckLastOk, 'file')
        modelSource = ckLastOk;
    end

    model = mphload(modelSource);
    model.hist.disable();

    switched = false;
    switchNote = 'none';
    try
        [switched, switchNote] = configure_linear_solver_mode(model, linearSolverMode);
    catch ME
        switched = false;
        switchNote = string(ME.message);
    end

    comp = model.component('comp1');
    solid = comp.physics('solid');
    pc = comp.pair('pc');

    % Enforce dcnt1-only and bind to pair 'pc'.
    try
        enforce_dcnt1_only(solid, 'dcnt1', 'cnt1', 'pc');
    catch
    end

    % Apply contact preset.
    contactSupported = true;
    contactEffective = contactMode;
    contactNote = 'none';
    try
        dcnt = solid.feature('dcnt1');
        [contactSupported, contactEffective, contactNote] = apply_contact_mode(dcnt, 'dcnt1', contactMode, 1.0, 1.0);
    catch ME
        contactSupported = false;
        contactNote = string(ME.message);
    end

    % Boundary selections
    bnd_rigid_top = solid.feature('bndl1').selection.entities;
    bnd_eval = pc.destination.entities;

    % Pressure load on the rigid plate top: use P_load parameter (kPa).
    try
        model.param.set('P_load', sprintf('%.6g[kPa]', PkPa));
    catch
        try
            model.param.set('P_load', sprintf('%.6g*1e3[Pa]', PkPa));
        catch
        end
    end
    try
        bndl = solid.feature('bndl1');
        bndl.set('forceType', 'FollowerPressure');
        % Apply negative sign to push along -z (for typical +z top-face normal).
        bndl.set('pressure', '-P_load');
        bndl.active(true);
    catch
    end

    % Disable displacement-control in z; keep x/y locked to avoid lateral rigid motion.
    try, solid.feature('disp_top').active(true); catch, end
    try
        solid.feature('disp_top').set('Direction', {'prescribed','prescribed','free'});
        solid.feature('disp_top').set('U0', {'0','0','0'});
    catch
    end
    try, model.param.set('delta', '0[um]'); catch, end

    % Study (stationary) settings + PTC configuration.
    st = model.study('std1').feature('stat');
    try, st.set('geometricNonlinearity', 'on'); catch, end
    try, st.set('geometricNonlinearityActive', 'on'); catch, end
    try, st.set('useparam', 'off'); catch, end
    try, st.set('initmethod', 'sol'); catch, end
    try, st.set('initsol', 'current'); catch, end
    try, st.set('useinitsol', 'on'); catch, end

    ptcTimeStep = numeric_env('SIM_PTC_DT', 0.05);
    ptcMaxSteps = numeric_env('SIM_PTC_MAX_STEPS', 80);
    ptcDamping = numeric_env('SIM_PTC_DAMPING', 0.5);
    try
        configure_ptc_solver(model, ptcTimeStep, ptcMaxSteps, ptcDamping);
    catch
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
    elapsedS = toc(pointT0);

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

    append_metrics_pressure_row(metricsPath, PkPa, PEff, Fz, ATop, Ac, tnMax, pnAvg, acWhy, ok, errMsg, elapsedS, linearSolverMode, switched, switchNote, contactMode, contactEffective, contactSupported, contactNote);

    % Save checkpoints (optional; do not commit to git).
    if ok
        try
            ckPoint = fullfile(sweepDir, sprintf('checkpoint_%s.mph', attemptTag));
            model.save(ckPoint);
        catch
        end
        try
            model.save(ckLastOk);
        catch
        end
    else
        try
            errPath = fullfile(sweepDir, sprintf('errors_%s.json', attemptTag));
            payload = struct();
            payload.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
            payload.P_load_kPa = PkPa;
            payload.linear_solver_mode = linearSolverMode;
            payload.linear_solver_switched = switched;
            payload.linear_solver_note = string_or_none(switchNote);
            payload.contact_mode_requested = contactMode;
            payload.contact_mode_effective = contactEffective;
            payload.contact_mode_supported = contactSupported;
            payload.contact_mode_note = string_or_none(contactNote);
            payload.message = string_or_none(errMsg);
            txt = jsonencode(payload);
            fid = fopen(errPath, 'w', 'n', 'UTF-8');
            fprintf(fid, '%s', txt);
            fclose(fid);
        catch
        end
    end

    try, ModelUtil.remove('model'); catch, end %#ok<TRYNC>
end
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

function s = pressure_tag(PkPa)
% E.g. 0.3 -> P0p3kPa
s = sprintf('P%.6gkPa', PkPa);
s = strrep(s, '.', 'p');
s = strrep(s, '-', 'm');
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

function [switched, note] = configure_linear_solver_mode(model, mode)
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
note = 'no Direct solver nodes were updated (PARDISO not available?)';
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

