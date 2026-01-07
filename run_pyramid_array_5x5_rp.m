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

outPyramidDir = fullfile(pwd, 'out', 'pyramid_5x5');
rpFromSimDir = getenv('RP_FROM_SIM_DIR');
if isempty(strtrim(rpFromSimDir))
    rpFromSimDir = find_latest_sim_dir(outPyramidDir);
end
rpFromSimDir = resolve_path(pwd, rpFromSimDir);
if isempty(strtrim(rpFromSimDir)) || ~exist(rpFromSimDir, 'dir')
    error('RP_FROM_SIM_DIR not set and no out/pyramid_5x5/sim_* found.');
end

simDir = fullfile(outPyramidDir, ['rp_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(simDir, 'dir')
    mkdir(simDir);
end

summaryPath = fullfile(simDir, 'RP_summary.txt');
rawCsvPath = fullfile(simDir, 'RP_raw.csv');
interpCsvPath = fullfile(simDir, 'RP_interp.csv');
simMetricsPath = fullfile(rpFromSimDir, 'Pyramid_5x5_metrics.csv');
simCheckpointPath = fullfile(rpFromSimDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
if ~exist(simCheckpointPath, 'file')
    simCheckpointPath = 'NA';
end
if ~exist(simMetricsPath, 'file')
    error('Missing mechanical metrics: %s', simMetricsPath);
end

% Geometry size (SI)
W_si = 100e-6;
A_top = W_si * W_si;

% Conductor-A parameters (SI)
rhoA = 5e-4;
tA = 3e-6;

% Target pressure range (Pa)
Pmin = 0.28e3;
Pmax = 1.00e3;

fid = fopen(summaryPath, 'w', 'n', 'UTF-8');
fprintf(fid, "rp_from_sim_dir: %s\n", rpFromSimDir);
fprintf(fid, "sim_metrics_path: %s\n", simMetricsPath);
fprintf(fid, "sim_checkpoint_path: %s\n", simCheckpointPath);
fprintf(fid, "W_m: %.6g\n", W_si);
fprintf(fid, "A_top_m2: %.6g\n", A_top);
fprintf(fid, "rho_A_Ohm_m: %.6g\n", rhoA);
fprintf(fid, "t_cnt_m: %.6g\n", tA);
fprintf(fid, "Assumption: Rc = (rho_A*t_cnt)/Ac; R1=R2=rho_A/t_cnt; R=R1+R2+Rc.\n");
fprintf(fid, "TargetPressureRange_Pa: [%g, %g]\n", Pmin, Pmax);
fprintf(fid, "MechanicsNote: RP reads mechanical outputs from sim metrics (no new mechanical solves).\n");
fprintf(fid, "\nProgressLog:\n");
fclose(fid);

% Load mechanical metrics from the specified sim output directory.
Tsim = readtable(simMetricsPath, 'PreserveVariableNames', true);
needCols = {'delta_total_um','Fz_plate_top_int_N','Tn_max_Pa','Ac_m2'};
for iC = 1:numel(needCols)
    if ~any(strcmp(Tsim.Properties.VariableNames, needCols{iC}))
        error('sim_metrics missing column: %s', needCols{iC});
    end
end

deltaList = Tsim.delta_total_um(:);
FzTop = Tsim.Fz_plate_top_int_N(:);
tnMax = Tsim.Tn_max_Pa(:);
Ac = Tsim.Ac_m2(:);
P_eff = abs(FzTop) ./ max(A_top, eps);

R1 = rhoA / tA; % Ω (since L=W=100 um)
R2 = R1;
rho_c = rhoA * tA; % Ω·m² (assumption A)
Rc = nan(size(deltaList));
R = nan(size(deltaList));
for i = 1:numel(deltaList)
    if isfinite(Ac(i)) && Ac(i) > 0
        Rc(i) = rho_c / Ac(i);
    else
        Rc(i) = NaN;
    end
    R(i) = R1 + R2 + Rc(i);
end

fid = fopen(summaryPath, 'a', 'n', 'UTF-8');
fprintf(fid, "  - loaded mechanical metrics rows: %d\n", numel(deltaList));
fclose(fid);

lastErrorMsg = '';
if false

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
end

% Write raw CSV (delta-sweep)
try
    Traw = table(deltaList(:), P_eff(:), FzTop(:), tnMax(:), Ac(:), repmat(R1, numel(deltaList), 1), repmat(R2, numel(deltaList), 1), Rc(:), R(:), ...
        'VariableNames', {'delta_total_um','P_eff_Pa','Fz_top_int_N','Tn_max_Pa','Ac_m2','R1_ohm','R2_ohm','Rc_ohm','R_total_ohm'});
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
fprintf(fid, "raw_csv: %s\n", rawCsvPath);
fprintf(fid, "interp_csv: %s\n", interpCsvPath);
fclose(fid);
end

function out = tern(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function dirPath = find_latest_sim_dir(outPyramidDir)
dirPath = '';
try
    d = dir(fullfile(outPyramidDir, 'sim_*'));
    d = d([d.isdir]);
    if isempty(d)
        return;
    end
    [~, idx] = sort({d.name});
    dirPath = fullfile(outPyramidDir, d(idx(end)).name);
catch
    dirPath = '';
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
