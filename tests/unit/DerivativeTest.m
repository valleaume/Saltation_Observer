classdef DerivativeTest < ProjectTestCase
    % Every hand-written derivative of the classes in utils/ is compared
    % with finite differences of the function it differentiates.
    %
    % Classes are discovered automatically: any class in utils/ defining one
    % of the methods of CONVENTION is checked, so a new class or a new
    % derivative method is tested without editing this file.

    properties (Constant)
        % derivative method  ->  method it is the Jacobian of (w.r.t. x)
        CONVENTION = struct( ...
            'jumpJacobian', 'jumpMap', ...
            'flowJacobian', 'flowMap', ...
            'guardGradient', 'guard', ...
            'outputJacobian', 'output');
    end

    properties (TestParameter)
        derivative = discoverDerivatives(DerivativeTest.CONVENTION);
    end

    methods (Test)
        function matchesFiniteDifferences(tc, derivative)
            model = feval(derivative.class);
            u = zeros(model.input_dimension, 1);
            rng(0);
            for trial = 1:3
                x = randn(model.state_dimension, 1);
                exact = callMethod(model, derivative.method, x, u);
                numeric = numericalJacobian(@(z) callMethod(model, derivative.map, z, u), x);
                numeric = restrictToDeclared(model, derivative.method, numeric, exact);
                tc.verifyEqual(exact(:), numeric(:), 'AbsTol', 1e-6, 'RelTol', 1e-6, ...
                    sprintf('%s.%s is not the Jacobian of %s at x = %s.', derivative.class, ...
                    derivative.method, derivative.map, mat2str(x', 3)));
            end
        end
    end
end

function cases = discoverDerivatives(convention)
% One test case per (class, derivative method) found in utils/.
cases = struct();
utils_folder = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'utils');
derivatives = fieldnames(convention);
for file = dir(fullfile(utils_folder, '*.m'))'
    [~, name] = fileparts(file.name);
    try
        mc = meta.class.fromName(name);
    catch
        continue   % file that does not parse (work in progress)
    end
    if isempty(mc) || mc.Abstract
        continue
    end
    defined = {mc.MethodList(~[mc.MethodList.Abstract]).Name};
    for k = find(ismember(derivatives, defined))'
        cases.([name '_' derivatives{k}]) = struct('class', name, ...
            'method', derivatives{k}, 'map', convention.(derivatives{k}));
    end
end
end

function value = callMethod(obj, name, x, u)
% Call obj.name(x, u, t, j), dropping trailing arguments the method does not
% take: the classes use (), (x), (x, u) or (x, u, t, j) signatures.
args = {x, u, 0, 0};
for n = numel(args):-1:0
    try
        value = obj.(name)(args{1:n});
        return
    catch err
        if ~any(strcmp(err.identifier, {'MATLAB:TooManyInputs', 'MATLAB:maxrhs'}))
            rethrow(err)
        end
    end
end
end

function J = numericalJacobian(fun, x)
% Central differences, one column per state component.
h = 1e-6;
J = zeros(numel(fun(x)), numel(x));
for i = 1:numel(x)
    d = zeros(size(x)); d(i) = h;
    J(:, i) = reshape(fun(x + d) - fun(x - d), [], 1)/(2*h);
end
end

function numeric = restrictToDeclared(model, method, numeric, exact)
% The billiard classes return the guard gradient w.r.t. the positions only;
% keep the matching columns, named by their *_index properties.
if numel(exact) == numel(numeric)
    return
end
assert(strcmp(method, 'guardGradient'), 'Unexpected size of %s.', method);
names = intersect({'x_index', 'y_index', 'x1_index', 'x2_index'}, properties(model), 'stable');
positions = cellfun(@(p) model.(p), names);
numeric = numeric(:, positions);
end
