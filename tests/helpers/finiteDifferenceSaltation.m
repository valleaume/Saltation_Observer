function S_fd = finiteDifferenceSaltation(plant, x_jump, guard, T, eps)
% FINITEDIFFERENCESALTATION - Saltation matrix estimated from simulations
%
% Independent of any hardcoded derivative: only plant.flowMap, plant.jumpMap
% and the guard function handle are used. With
%   Phi   = flow(T) o jump o flow(until guard)   (full hybrid map),
%   P_pre = flow(T) from x_pre, P_post = flow(T) from g(x_jump),
% where x_pre is x_jump flowed backward by T, the chain rule gives
%   DPhi = DP_post * S * DP_pre   =>   S = DP_post \ DPhi / DP_pre.

if nargin < 4, T = 0.05; end
if nargin < 5, eps = 1e-5; end

opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);
f = @(t, x) plant.flowMap(x, 0, 0, 0);
flow = @(x0, tf) deval(ode45(f, [0, tf], x0, opts), tf);

x_pre = deval(ode45(@(t, x) -f(t, x), [0, T], x_jump, opts), T);
x_post = plant.jumpMap(x_jump, 0, 0, 0);

hybridMap = @(x0) hybridFlow(f, plant, guard, x0, 2*T, opts);

n = numel(x_jump);
DPhi = zeros(n); DP_pre = zeros(n); DP_post = zeros(n);
for i = 1:n
    d = zeros(n, 1); d(i) = eps;
    DPhi(:, i) = (hybridMap(x_pre + d) - hybridMap(x_pre - d))/(2*eps);
    DP_pre(:, i) = (flow(x_pre + d, T) - flow(x_pre - d, T))/(2*eps);
    DP_post(:, i) = (flow(x_post + d, T) - flow(x_post - d, T))/(2*eps);
end
S_fd = DP_post \ DPhi / DP_pre;
end

function x_end = hybridFlow(f, plant, guard, x0, tf, opts)
% Flow until the guard is crossed downward, jump once, flow until tf.
ev_opts = odeset(opts, 'Events', @(t, x) guardEvent(guard, x));
sol = ode45(f, [0, tf], x0, ev_opts);
assert(~isempty(sol.xe), 'finiteDifferenceSaltation: no impact detected.');
t_jump = sol.xe(end);
x_plus = plant.jumpMap(sol.ye(:, end), 0, 0, 0);
x_end = deval(ode45(f, [t_jump, tf], x_plus, opts), tf);
end

function [value, isterminal, direction] = guardEvent(guard, x)
value = guard(x);
isterminal = 1;
direction = -1;
end
