function repair_model_dcnt1_only(modelPath)
%REPAIR_MODEL_DCNT1_ONLY Disable obsolete cnt1 and bind dcnt1 SolidContact explicitly to pair pc.
%
% This script is meant to be used on a baseline MPH (e.g. /mnt/data/... in other environments)
% to produce a repaired MPH for manual inspection or as a new runtime template (do not commit MPH).
%
% Usage examples:
%   matlab -batch "repair_model_dcnt1_only('out/pyramid_5x5/Pyramid_5x5_Mech.mph')"
%   matlab -batch "repair_model_dcnt1_only(getenv('SIM_BASELINE_MPH'))"

import com.comsol.model.util.*

if nargin < 1 || strlength(string(modelPath)) == 0
    modelPath = getenv('SIM_BASELINE_MPH');
end
modelPath = char(strtrim(string(modelPath)));
if strlength(string(modelPath)) == 0
    error('modelPath is required (arg or SIM_BASELINE_MPH).');
end
if ~exist(modelPath, 'file')
    error('Missing model file: %s', modelPath);
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>
model = mphload(modelPath);
model.hist.disable();

solid = model.physics('solid');
[ok, note] = enforce_dcnt1_only(solid, 'dcnt1', 'cnt1', 'pc');

outRoot = fullfile(pwd, 'out', 'pyramid_5x5');
outDir = fullfile(outRoot, ['repair_dcnt1_only_' datestr(now, 'yyyymmdd_HHMMSS')]);
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outMph = fullfile(outDir, 'Pyramid_5x5_repaired_dcnt1_only.mph');

try
    mphsave(model, outMph);
catch ME
    ModelUtil.disconnect();
    error('Failed to save repaired mph: %s', ME.message);
end

fprintf('repair_ok: %d\n', ok);
fprintf('repair_note: %s\n', string(note));
fprintf('repaired_mph: %s\n', outMph);

% Print a brief contact feature snapshot.
try
    d = solid.feature('dcnt1');
    fprintf('dcnt1_type: %s\n', char(d.getType()));
    fprintf('dcnt1_pairSelection: %s\n', char(d.getString('pairSelection')));
    try
        c = cell(d.getStringArray('pairs'));
        c = cellfun(@char, c, 'UniformOutput', false);
        fprintf('dcnt1_pairs: %s\n', strjoin(c, ','));
    catch
        fprintf('dcnt1_pairs: (unavailable)\n');
    end
catch
end
try
    solid.feature('cnt1');
    fprintf('cnt1_present: 1\n');
catch
    fprintf('cnt1_present: 0\n');
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
end

function [ok, note] = enforce_dcnt1_only(solid, dcntTag, cntTag, pairTag)
ok = false;
note = "none";

% Remove/deactivate obsolete cnt1.
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
            note = "cnt1_disable_failed: " + string(ME.message);
        end
    end
end

% Bind dcnt1 to pc.
try
    dcnt = solid.feature(dcntTag);
catch ME
    note = "missing_dcnt1: " + string(ME.message);
    return;
end
try, dcnt.set('pairSelection', 'list'); catch, end
try, dcnt.set('pairselection', 'list'); catch, end
try
    arr = javaArray('java.lang.String', 1);
    arr(1) = java.lang.String(pairTag);
    dcnt.set('pairs', arr);
catch ME
    note = "dcnt_pairs_set_failed: " + string(ME.message);
    return;
end

ok = is_pair_bound(dcnt, pairTag);
if ~ok
    note = "dcnt_pair_verify_failed";
end
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

