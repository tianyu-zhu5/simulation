function run_pyramid_array_5x5_rp()
%RUN_PYRAMID_ARRAY_5X5_RP Compute R–P, ΔR/R0–P, and S(P) using the agreed series model.
%
% Inputs (from demand.md + answer.md):
% - Geometry: 100×100 um base made of 5×5 unit cells (pitch=20 um), each cell has one Lpyr×Lpyr pyramid
%   located in the top-left quadrant.
% - PDMS: truncated thickness t_pdms=50 um (fixed bottom, side faces free)
% - CNT/Conductor-A: thickness t_cnt=3 um over the entire base top surface + pyramid faces, and on the
%   upper pressure plate (assumed symmetric for the series model).
% - Pressure range of interest: 0.28–1 kPa; R0 taken at 0.28 kPa.
% - Contact resistance model: rho_c = rho_A * t_cnt  (Ω·m²), Rc = rho_c / Ac
% - Bulk resistances: R1=R2=rho_A*L/(t_cnt*W) with L=W=100 um  => R1=R2=rho_A/t_cnt
%
% IMPORTANT:
% - Mechanical solve uses displacement-control (delta) for robustness, then computes an effective pressure
%   P_eff = |Fz_top| / A_top, where A_top = W*W.
% - Contact quantities use solid.dcnt1.Tn (single source of truth; explicitly bound to pair pc).

import com.comsol.model.util.*

tpl = fullfile(pwd, 'out', 'pyramid_5x5', 'Pyramid_5x5_Mech.mph');
if ~exist(tpl, 'file')
    error('Missing 5x5 template: %s (run build_pyramid_array_5x5_mech first)', tpl);
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>

simDir = fullfile(pwd, 'out', 'pyramid_5x5', ['rp_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(simDir, 'dir')
    mkdir(simDir);
end

summaryPath = fullfile(simDir, 'RP_summary.txt');
rawCsvPath = fullfile(simDir, 'RP_raw.csv');
interpCsvPath = fullfile(simDir, 'RP_interp.csv');
outMph = fullfile(simDir, 'Pyramid_5x5_RP_solved.mph');

model = mphload(tpl);
model.hist.disable();

comp = model.component('comp1');
solid = comp.physics('solid');
pc = comp.pair('pc');

% IMPORTANT (per suggestion.md): do NOT prescribe displacement on the contact face.
% Use top-face displacement control; the builder increases t_rigid to reduce compression strain.
bnd_rigid_top = solid.feature('bndl1').selection.entities;
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

% Disable pressure load for this run (we derive P_eff from reaction force).
try
    solid.feature('bndl1').set('forceType', 'FollowerPressure');
    solid.feature('bndl1').set('pressure', '0[Pa]');
catch
end
try, model.param.set('P_load', '0[Pa]'); catch, end

% Mesh: keep template default (the 5x5 model can be very large).

% Study settings
st = model.study('std1').feature('stat');
st.set('geometricNonlinearity', 'on');
try, st.set('geometricNonlinearityActive', 'on'); catch, end
st.set('useparam', 'off');

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

% Selections
% Reaction force is taken on the prescribed displacement boundary (plate top).
bnd_force = bnd_rigid_top;
bnd_dst = pc.destination.entities;

% Geometry size (SI)
try
    W_si = model.param.evaluate('W'); % meters
catch
    W_si = 100e-6;
end
A_top = W_si * W_si;

% Conductor-A parameters (SI)
try
    rhoA = model.param.evaluate('rho_A'); % Ω·m
catch
    rhoA = 5e-4;
end
try
    tA = model.param.evaluate('t_cnt'); % meters
catch
    tA = 3e-6;
end

% Target pressure range (Pa)
Pmin = 0.28e3;
Pmax = 1.00e3;

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "Template: %s\n", tpl);
fprintf(fid, "W_m: %.6g\n", W_si);
fprintf(fid, "A_top_m2: %.6g\n", A_top);
fprintf(fid, "rho_A_Ohm_m: %.6g\n", rhoA);
fprintf(fid, "t_cnt_m: %.6g\n", tA);
fprintf(fid, "Assumption: Rc = (rho_A*t_cnt)/Ac; R1=R2=rho_A/t_cnt; R=R1+R2+Rc.\n");
fprintf(fid, "TargetPressureRange_Pa: [%g, %g]\n", Pmin, Pmax);
fprintf(fid, "MechanicsNote: plate driven by top-face displacement (bottom contact face not prescribed).\n");
fprintf(fid, "\nProgressLog:\n");
fclose(fid);

lastErrorMsg = '';

% Delta sweep (um). With gap0=1[um], contact typically starts near delta≈1[um].
% We start a bit below and sweep beyond to cover ~0.28–1.0 kPa via P_eff.
deltaList = 0.85:0.05:1.35;
hasSol = false;

P_eff = nan(size(deltaList));
FzTop = nan(size(deltaList));
tnMax = nan(size(deltaList));
Ac = nan(size(deltaList));

R1 = rhoA / tA; % Ω (since L=W=100 um)
R2 = R1;
rho_c = rhoA * tA; % Ω·m² (assumption A)
Rc = nan(size(deltaList));
R = nan(size(deltaList));

for i = 1:numel(deltaList)
    model.param.set('delta', sprintf('%.6g[um]', deltaList(i)));
    try, st.set('useinitsol', tern(hasSol,'on','off')); catch, end

    fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
    fprintf(fid, "  - solve delta=%.6g um (useinitsol=%s)\n", deltaList(i), tern(hasSol,'on','off'));
    fclose(fid);

    try
        model.study('std1').run();
        hasSol = true;
    catch ME
        lastErrorMsg = string(ME.message);
        fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
        fprintf(fid, "    solve FAILED at delta=%.6g um: %s\n", deltaList(i), lastErrorMsg);
        fclose(fid);
        break;
    end

    try, FzTop(i) = mphint2(model, 'solid.RFz', 'surface', 'selection', bnd_force); catch, end
    P_eff(i) = abs(FzTop(i)) / max(A_top, eps);

    try, tnMax(i) = mphmax(model, 'solid.dcnt1.Tn', 'surface', 'selection', bnd_dst); catch, end
    try, Ac(i) = mphint2(model, 'if(solid.dcnt1.Tn>0,1,0)', 'surface', 'selection', bnd_dst); catch, end

    if isfinite(Ac(i)) && Ac(i) > 0
        Rc(i) = rho_c / Ac(i);
    else
        Rc(i) = NaN;
    end
    R(i) = R1 + R2 + Rc(i);
end

% Save solved model (last state)
mphsave(model, outMph);

% Write raw CSV (delta-sweep)
try
    Traw = table(deltaList(:), P_eff(:), FzTop(:), tnMax(:), Ac(:), repmat(R1, numel(deltaList), 1), repmat(R2, numel(deltaList), 1), Rc(:), R(:), ...
        'VariableNames', {'delta_um','P_eff_Pa','Fz_top_int_N','Tn_max_Pa','Ac_m2','R1_ohm','R2_ohm','Rc_ohm','R_total_ohm'});
    writetable(Traw, rawCsvPath);
catch
end

% Interpolate to a uniform P-grid within [0.28, 1.0] kPa (if covered)
mask = isfinite(P_eff) & isfinite(R) & P_eff > 0;
P_sorted = P_eff(mask);
R_sorted = R(mask);
[P_sorted, ord] = sort(P_sorted);
R_sorted = R_sorted(ord);

Pgrid = linspace(Pmin, Pmax, 15).';
Rgrid = nan(size(Pgrid));

interpStatus = "not_enough_data";
if numel(P_sorted) >= 2 && Pmin >= min(P_sorted) && Pmax <= max(P_sorted)
    Rgrid = interp1(P_sorted, R_sorted, Pgrid, 'linear');
    interpStatus = "ok";
end

% R0 at P=0.28 kPa (by interpolation on Pgrid if possible, else nearest)
R0 = NaN;
if any(isfinite(Rgrid))
    R0 = Rgrid(1);
elseif ~isempty(P_sorted)
    [~, idx0] = min(abs(P_sorted - Pmin));
    if ~isempty(idx0)
        R0 = R_sorted(idx0);
    end
end

if isempty(R0) || ~isfinite(R0)
    dRR0 = nan(size(Pgrid));
else
    dRR0 = (Rgrid - R0) ./ R0;
end

% Sensitivity: finite difference on the P-grid
S = nan(size(Pgrid));
if numel(dRR0) >= 3
    for i = 2:numel(Pgrid)-1
        if isfinite(dRR0(i-1)) && isfinite(dRR0(i+1))
            S(i) = (dRR0(i+1) - dRR0(i-1)) / (Pgrid(i+1) - Pgrid(i-1));
        end
    end
end

try
    Tinterp = table(Pgrid, Rgrid, repmat(R0, numel(Pgrid), 1), dRR0, S, ...
        'VariableNames', {'P_Pa','R_total_ohm','R0_ohm','dR_over_R0','S_per_Pa'});
    writetable(Tinterp, interpCsvPath);
catch
end

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "\nResults:\n");
fprintf(fid, "R1=R2=rho_A/t_cnt [ohm]: %.6g\n", R1);
fprintf(fid, "rho_c=rho_A*t_cnt [ohm*m^2]: %.6g\n", rho_c);
fprintf(fid, "R0 at P=%.6g Pa [ohm]: %.6g\n", Pmin, R0);
fprintf(fid, "valid_points_for_RP: %d\n", sum(mask));
if any(mask)
    fprintf(fid, "P_eff_coverage_Pa: [%.6g, %.6g]\n", min(P_sorted), max(P_sorted));
else
    fprintf(fid, "P_eff_coverage_Pa: [NaN, NaN]\n");
end
fprintf(fid, "interp_status: %s\n", interpStatus);
if strlength(string(lastErrorMsg)) > 0
    fprintf(fid, "last_solve_error: %s\n", lastErrorMsg);
end
fprintf(fid, "raw_csv: %s\n", rawCsvPath);
fprintf(fid, "interp_csv: %s\n", interpCsvPath);
fprintf(fid, "solved_mph: %s\n", outMph);
fclose(fid);

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
end

function out = tern(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end
