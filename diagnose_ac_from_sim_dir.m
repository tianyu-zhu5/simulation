function diagnose_ac_from_sim_dir(simDir)
%DIAGNOSE_AC_FROM_SIM_DIR Diagnose contact area (Ac) extraction for a solved sim output directory.
%
% Writes:
%   out/pyramid_5x5/diag_ac_YYYYMMDD_HHMMSS/Ac_diagnose.csv
%
% Intended usage:
%   matlab -batch "diagnose_ac_from_sim_dir('out/pyramid_5x5/sim_YYYYMMDD_HHMMSS/')"

import com.comsol.model.util.*

if nargin < 1 || strlength(string(simDir)) == 0
    error('simDir is required');
end

rootDir = pwd;
simDir = char(resolve_path(rootDir, simDir));
if ~exist(simDir, 'dir')
    error('Missing sim dir: %s', simDir);
end

outRoot = fullfile(rootDir, 'out', 'pyramid_5x5');
diagDir = fullfile(outRoot, ['diag_ac_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(diagDir, 'dir')
    mkdir(diagDir);
end
csvPath = fullfile(diagDir, 'Ac_diagnose.csv');

checkpointPath = fullfile(simDir, 'Pyramid_5x5_checkpoint_last_ok.mph');
if ~exist(checkpointPath, 'file')
    fallback = fullfile(simDir, 'Pyramid_5x5_solved.mph');
    if exist(fallback, 'file')
        checkpointPath = fallback;
    else
        error('Missing checkpoint mph: %s', checkpointPath);
    end
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>

model = mphload(checkpointPath);
model.hist.disable();

comp = model.component('comp1');
pc = comp.pair('pc');
bndDst = pc.destination.entities;
bndCount = numel(bndDst);
bndPreview = preview_ids(bndDst, 24);

sampleNote = 'mpheval(boundary, selection=pc.destination.entities)';

% Candidate expressions (3–5+; we include both dcnt1 and cnt1 families).
exprs = {
    struct('expr', 'solid.dcnt1.Tn', 'kind', 'pressure', 'thr', 0, 'op', '>')
    struct('expr', 'solid.dcnt1.gap', 'kind', 'gap', 'thr', 0, 'op', '<')
    struct('expr', 'solid.cnt1.Tn',  'kind', 'pressure', 'thr', 0, 'op', '>')
    struct('expr', 'solid.cnt1.gap', 'kind', 'gap', 'thr', 0, 'op', '<')
    struct('expr', 'solid.cnt1.incontact', 'kind', 'status', 'thr', 0.5, 'op', '>')
    struct('expr', 'Tn_contact', 'kind', 'pressure', 'thr', 0, 'op', '>')
    struct('expr', 'gap_contact', 'kind', 'gap', 'thr', 0, 'op', '<')
    };

rows = [];
for i = 1:numel(exprs)
    e = exprs{i};
    rows = [rows; eval_expr(model, e.expr, e.kind, e.thr, e.op, bndDst, bndCount, bndPreview, sampleNote)]; %#ok<AGROW>
end

T = struct2table(rows);
try
    writetable(T, csvPath);
catch
    % fallback: manual CSV
    fid = fopen(csvPath, 'w', 'n', 'UTF-8');
    fprintf(fid, '%s\n', strjoin(T.Properties.VariableNames, ','));
    for i = 1:height(T)
        vals = cell(1, width(T));
        for j = 1:width(T)
            v = T{i, j};
            if iscell(v), v = v{1}; end
            if isstring(v) || ischar(v)
                vals{j} = sanitize_csv_text(string(v));
            elseif isnumeric(v)
                vals{j} = num2str(v, '%.12g');
            elseif islogical(v)
                vals{j} = tern(v, '1', '0');
            else
                vals{j} = sanitize_csv_text(string(v));
            end
        end
        fprintf(fid, '%s\n', strjoin(vals, ','));
    end
    fclose(fid);
end

ModelUtil.disconnect();
try
    if ~isempty(proc) && ~proc.HasExited
        proc.WaitForExit(2000);
    end
    if ~isempty(proc) && ~proc.HasExited
        proc.Kill();
        proc.WaitForExit();
    end
catch
end

fprintf('Ac diagnosis written: %s\n', csvPath);
end

function row = eval_expr(model, expr, kind, thr, op, bndSel, bndCount, bndPreview, sampleNote)
row = struct();
row.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
row.expr = string(expr);
row.kind = string(kind);
row.threshold = thr;
row.op = string(op);
row.selection = "pc.destination.entities";
row.selection_boundary_count = bndCount;
row.selection_boundary_ids_preview = string(bndPreview);
row.sample_note = string(sampleNote);

row.sample_n = NaN;
row.nonNaN_count = 0;
row.min = NaN;
row.max = NaN;
row.Ac_candidate_m2 = NaN;
row.error = "none";

% Sample values on boundary evaluation points via mpheval (gives us count/min/max).
try
    data = mpheval(model, expr, 'edim', 'boundary', 'selection', bndSel, 'dataonly', 'on', 'matherr', 'off');
    v = double(data(:));
    row.sample_n = numel(v);
    good = isfinite(v);
    row.nonNaN_count = sum(good);
    if any(good)
        row.min = min(v(good));
        row.max = max(v(good));
    end
catch ME
    row.error = "mpheval: " + string(ME.message);
end

% candidate Ac definition
try
    row.Ac_candidate_m2 = mphint2(model, sprintf('if(%s%s%g,1,0)', expr, op, thr), 'surface', 'selection', bndSel);
catch ME
    if row.error == "none"
        row.error = "mphint2(Ac): " + string(ME.message);
    else
        row.error = row.error + " | mphint2(Ac): " + string(ME.message);
    end
end
end

function out = preview_ids(ids, n)
try
    v = double(ids(:));
catch
    try
        v = cellfun(@double, cell(ids));
    catch
        out = 'NA';
        return;
    end
end
if isempty(v)
    out = '[]';
    return;
end
v = v(:).';
if numel(v) > n
    v = v(1:n);
end
out = ['[' strjoin(arrayfun(@(x) num2str(x), v, 'UniformOutput', false), ' ') ']'];
end

function out = tern(cond, a, b)
if cond
    out = a;
else
    out = b;
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

function s = sanitize_csv_text(x)
s = string(x);
s = replace(s, """", """""");
if contains(s, ",") || contains(s, newline) || contains(s, """")
    s = """" + s + """";
end
end
