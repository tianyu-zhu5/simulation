function diagnose_ac_from_sim_dir_dcnt1_only(simDir)
%DIAGNOSE_AC_FROM_SIM_DIR_DCNT1_ONLY Diagnose dcnt1-only contact area (Ac) extraction for a solved sim output directory.
%
% Writes:
%   out/pyramid_5x5/diag_ac_YYYYMMDD_HHMMSS/Ac_diagnose.csv
%
% Intended usage:
%   matlab -batch "diagnose_ac_from_sim_dir_dcnt1_only('out/pyramid_5x5/sim_YYYYMMDD_HHMMSS/')"

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

enforceOk = false;
enforceNote = "none";
try
    solid = model.physics('solid');
    [enforceOk, enforceNote] = enforce_dcnt1_only(solid, 'dcnt1', 'cnt1', 'pc');
catch ME
    enforceOk = false;
    enforceNote = "exception: " + short_err(ME);
end

sampleNote = "selection=pc.destination.entities; enforce_ok=" + string(enforceOk) + "; enforce_note=" + string(enforceNote);

exprs = {
    struct('expr', 'solid.incontact', 'kind', 'status', 'thr', 0.5, 'op', '>')
    struct('expr', 'solid.Tn', 'kind', 'pressure', 'thr', 1.0, 'op', '>')
    struct('expr', 'solid.gap', 'kind', 'gap', 'thr', 0, 'op', '<')
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
    row.error = "mpheval: " + short_err(ME);
end

try
    row.Ac_candidate_m2 = mphint2(model, sprintf('if(%s%s%g,1,0)', expr, op, thr), 'surface', 'selection', bndSel);
catch ME
    if row.error == "none"
        row.error = "mphint2(Ac): " + short_err(ME);
    else
        row.error = row.error + " | mphint2(Ac): " + short_err(ME);
    end
end
end

function [ok, note] = enforce_dcnt1_only(solid, dcntTag, cntTag, pairTag)
ok = false;
note = "none";

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
        note = "cnt1_removed";
    catch
    end
    if ~removed
        try
            solid.feature(cntTag).active(false);
            note = "cnt1_deactivated";
        catch ME
            note = "cnt1_disable_failed: " + short_err(ME);
        end
    end
end

try
    dcnt = solid.feature(dcntTag);
catch ME
    note = "missing_dcnt1: " + short_err(ME);
    return;
end
try, dcnt.active(true); catch, end

ok = bind_pair_to_feature(dcnt, pairTag);
if ~ok
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
    arr = feat.getStringArray('pairs');
    c = cell(arr);
    c = cellfun(@char, c, 'UniformOutput', false);
    if any(strcmp(c, pairTag))
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

function out = preview_ids(ids, n)
try
    v = double(ids(:));
catch
    out = 'NA';
    return;
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

function s = short_err(ME)
try
    s = string(ME.message);
catch
    s = "unknown error";
end
s = replace(s, newline, " ");
s = replace(s, sprintf('\r'), " ");
end
