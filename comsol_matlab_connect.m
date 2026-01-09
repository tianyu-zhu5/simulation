function [port, pid, proc] = comsol_matlab_connect()
%COMSOL_MATLAB_CONNECT Start a local COMSOL mphserver and connect from MATLAB.
%
% Returns:
%   port: server port
%   pid:  server process id
%   proc: .NET Process handle (Windows)

comsolRoot = 'C:\Program Files\COMSOL\COMSOL63\Multiphysics';
mliPath = fullfile(comsolRoot, 'mli');
startupPath = fullfile(mliPath, 'startup');

if ~exist(mliPath, 'dir')
    error('COMSOL mli path not found: %s', mliPath);
end

addpath(mliPath);
if exist(startupPath, 'dir')
    addpath(startupPath);
end

comsolServerExe = fullfile(comsolRoot, 'bin', 'win64', 'comsolmphserver.exe');
if ~exist(comsolServerExe, 'file')
    error('COMSOL server executable not found: %s', comsolServerExe);
end

portfile = fullfile(tempdir, ['comsol_port_' char(java.util.UUID.randomUUID()) '.txt']);
args = ['-silent -login auto -port 0 -portfile "' portfile '"'];

psi = System.Diagnostics.ProcessStartInfo();
psi.FileName = comsolServerExe;
psi.Arguments = args;
psi.UseShellExecute = false;
psi.CreateNoWindow = true;
proc = System.Diagnostics.Process.Start(psi);
pid = double(proc.Id);

t0 = tic;
port = NaN;
while toc(t0) < 120
    if exist(portfile, 'file') == 2
        txt = strtrim(fileread(portfile));
        if ~isempty(txt)
            port = str2double(txt);
            break;
        end
    end
    pause(0.2);
end
if ~isfinite(port)
    error('Failed to start COMSOL server: portfile not created or empty: %s', portfile);
end

mphstart('localhost', port, comsolRoot);
end

