function [model, port, pid, proc] = comsol_matlab_setup()
%COMSOL_MATLAB_SETUP Connect MATLAB to COMSOL via LiveLink and create a model.
%
% Assumes COMSOL 6.3 is installed at:
%   C:\Program Files\COMSOL\COMSOL63\Multiphysics
[port, pid, proc] = comsol_matlab_connect();

import com.comsol.model.util.*
import com.comsol.model.*

model = ModelUtil.create('Model');
model.modelNode.create('mod1');
model.hist.disable();
end
