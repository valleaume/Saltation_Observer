function ws = runScriptIn(runScriptIn_script, runScriptIn_overrides)
% RUNSCRIPTIN - Run a script in an isolated workspace and return its variables
%
%   ws = runScriptIn(scriptPath, overrides)
%
% Each field of the struct overrides is defined as a variable before the
% script runs (the scripts keep a variable that already exists, see the
% "if ~exist(...)" lines). ws is a struct of all variables left by the script.

if nargin < 2
    runScriptIn_overrides = struct();
end
runScriptIn_names = fieldnames(runScriptIn_overrides);
for runScriptIn_k = 1:numel(runScriptIn_names)
    eval([runScriptIn_names{runScriptIn_k} ' = runScriptIn_overrides.(runScriptIn_names{runScriptIn_k});']);
end
clear runScriptIn_names runScriptIn_k runScriptIn_overrides

run(runScriptIn_script);

runScriptIn_vars = setdiff(who, {'runScriptIn_script'});
ws = struct();
for runScriptIn_k = 1:numel(runScriptIn_vars)
    if ~strcmp(runScriptIn_vars{runScriptIn_k}, 'runScriptIn_vars')
        ws.(runScriptIn_vars{runScriptIn_k}) = eval(runScriptIn_vars{runScriptIn_k});
    end
end
end
