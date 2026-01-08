function inspect_contact_features(modelPath)
%INSPECT_CONTACT_FEATURES Print Solid Mechanics contact feature metadata for debugging.
%
% Usage:
%   matlab -batch "inspect_contact_features('out/pyramid_5x5/sim_20260107_171121/Pyramid_5x5_checkpoint_last_ok.mph')"

import com.comsol.model.util.*

if nargin < 1 || strlength(string(modelPath)) == 0
    error('modelPath is required');
end

modelPath = char(modelPath);
if ~exist(modelPath, 'file')
    error('Missing model file: %s', modelPath);
end

[~, ~, proc] = comsol_matlab_connect(); %#ok<ASGLU>
model = mphload(modelPath);
model.hist.disable();

solid = model.physics('solid');
tags = {};
try
    tags = cell(solid.feature.tags);
catch
end

fprintf('modelPath: %s\n', modelPath);
fprintf('solid.feature.tags (%d): %s\n', numel(tags), strjoin(cellfun(@char, tags, 'UniformOutput', false), ', '));

inspect_one(solid, 'dcnt1');
inspect_one(solid, 'cnt1');

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

function inspect_one(solid, tag)
fprintf('\n-- %s --\n', tag);
try
    feat = solid.feature(tag);
catch ME
    fprintf('missing: %s\n', ME.message);
    return;
end
try
    fprintf('type: %s\n', char(feat.getType()));
catch
    fprintf('type: (unavailable)\n');
end

keys = {'pairSelection','pairselection','pairs','pair','pairname','ContactMethodCtrl','method','penaltyCtrl'};
for i = 1:numel(keys)
    k = keys{i};
    try
        v = feat.getString(k);
        fprintf('%s: %s\n', k, char(v));
        continue;
    catch
    end
    try
        arr = feat.getStringArray(k);
        out = cell(arr);
        out = cellfun(@char, out, 'UniformOutput', false);
        fprintf('%s[]: %s\n', k, strjoin(out, ','));
        continue;
    catch
    end
end
end

