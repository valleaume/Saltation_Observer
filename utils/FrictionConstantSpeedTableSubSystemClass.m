classdef FrictionConstantSpeedTableSubSystemClass < HybridSubsystem & SaltationModel
    % Mass m on a spring of stiffness k attached to a fixed wall, resting on a
    % table that moves at the constant speed vb, with Coulomb friction whose
    % static and dynamic levels mu_s > mu_d are UNKNOWN. The table speed is
    % unknown as well: vb, mu_s and mu_d are part of the state.
    %
    %   x = [x1; ...; x5; q] = [x; v; vb; mu_s; mu_d; q]
    %   q = 1: slip with s > 0,  q = 2: slip with s < 0,  q = 3: stick
    %   s = v - vb (relative velocity), z = k*x (force needed to stick)
    %
    % Why z: the mass obeys m*v' = -k*x + F (friction force F). Stuck, v' = 0,
    % so the friction must supply F = k*x = z: it holds while |z| <= mu_s.
    % Slipping, F = -/+mu_d and the relative acceleration is m*s' = -(z +/- mu_d).
    % mu_s and mu_d are forces (the normal force is absorbed in them).
    %
    %   flow : x' = v, m*v' = -k*x - mu_d (q = 1), -k*x + mu_d (q = 2),
    %          v' = 0 (q = 3, so s stays 0), vb' = mu_s' = mu_d' = 0
    %   jump : q = 1, 2 -> 3 when s reaches 0 (stick entry), which sets v = vb,
    %          q = 3 -> 2 / 1 when z reaches +mu_s / -mu_s (breakaway),
    %          identity on x1..x5 except at the stick entry (resetMatrix)
    %   output: y = x ('position') or y = v ('velocity')
    %
    % Why v = vb at the stick entry: on its guard s = 0 it changes nothing,
    % but the solver detects the guard with s a rounding error >= 0, which
    % the stick flow keeps until the breakaway back into slip 2 (the orbit
    % never reverses), outside the flow set s <= 0. The saltation matrix keeps
    % its value: the reset carries what the jump timing carried (see
    % FrictionConstantSpeedTableObserver.saltationFactors).
    %
    % Guard: s (q = 1), -s (q = 2), mu_s - |z| (q = 3). vb enters only the
    % stick-entry guard and mu_s only the breakaway guard.
    % For vb > 0 the stick-slip orbit is 3 -> 2 -> 3 (periodicOrbit): the
    % slip direction never changes, so for y = v the shift
    % (x, mu_s, mu_d) -> (x + d, mu_s + k*d, mu_d + k*d) leaves the output
    % unchanged. Only y = x makes the whole state observable.
    % Symmetry: (xi, q) -> (T*xi, 1 <-> 2), T = diag(-1,-1,-1,1,1) (vb -> -vb).

    properties
        m = 1;          % mass
        k = 1;          % spring stiffness
    end

    properties (SetAccess = immutable)
        measurement     % 'position' or 'velocity' (the HyEQ output is fixed at construction)
    end

    properties (Constant)
        T = diag([-1 -1 -1 1 1]);
    end

    methods
        function obj = FrictionConstantSpeedTableSubSystemClass(measurement)
            if nargin < 1, measurement = 'position'; end
            H = FrictionConstantSpeedTableSubSystemClass.outputMatrix(measurement);
            obj = obj@HybridSubsystem(6, 0, 1, @(x) H*x(1:5));
            obj.measurement = measurement;
        end

        function xdot = flowMap(this, x, ~, ~, ~)
            xdot = [this.flowMatrix(this.modeOf(x))*x(1:5); 0];
        end

        function xplus = jumpMap(this, x, ~, ~, ~)
            q_next = this.nextMode(x);
            xplus = [this.resetMatrix(this.modeOf(x), q_next)*x(1:5); q_next];
        end

        function inC = flowSetIndicator(this, x, ~, ~, ~)
            switch this.modeOf(x)
                case 1, inC = this.s(x) >= 0;
                case 2, inC = this.s(x) <= 0;
                case 3, inC = abs(this.z(x)) <= x(4);
            end
        end

        function inD = jumpSetIndicator(this, x, ~, ~, ~)
            switch this.modeOf(x)
                case 1, inD = this.s(x) <= 0 && this.z(x) >= -x(5);   % s' <= 0 there
                case 2, inD = this.s(x) >= 0 && this.z(x) <= x(5);
                case 3, inD = abs(this.z(x)) >= x(4);
            end
        end
    end

    methods
        function A = flowMatrix(this, q)
            % xi' = A_q*xi in mode q (xi = x1..x5).
            A = FrictionConstantSpeedTableSubSystemClass.modeMatrix(q, this.m, this.k);
        end

        function H = H(this)
            H = FrictionConstantSpeedTableSubSystemClass.outputMatrix(this.measurement);
        end

        function qn = nextMode(this, x)
            % Stick entry, direct reversal if |z| > mu_s, or breakaway.
            z = this.z(x);
            switch this.modeOf(x)
                case 1, qn = 3 - (z > x(4));
                case 2, qn = 3 - 2*(z < -x(4));
                case 3, qn = 1 + (z >= 0);
            end
        end

        function s = s(~, x)
            s = x(2) - x(3);
        end

        function z = z(this, x)
            z = this.k*x(1);
        end

        % Guard and hand-written derivatives (see SaltationModel). Every
        % derivative with respect to q is zero.
        function w = guard(this, x)
            switch this.modeOf(x)
                case 1, w = this.s(x);
                case 2, w = -this.s(x);
                case 3, w = x(4) - abs(this.z(x));
            end
        end
        function A = flowJacobian(this, x)
            A = blkdiag(this.flowMatrix(this.modeOf(x)), 0);
        end
        function Dg = jumpJacobian(this, x)
            Dg = blkdiag(this.resetMatrix(this.modeOf(x), this.nextMode(x)), 0);
        end
        function gradGuard = guardGradient(this, x)
            switch this.modeOf(x)
                case 1, gradGuard = [0 1 -1 0 0 0];
                case 2, gradGuard = [0 -1 1 0 0 0];
                case 3, gradGuard = [-sign(this.z(x))*this.k, 0, 0, 1, 0, 0];
            end
        end
        function H = outputJacobian(this)
            H = [this.H(), 0];
        end

        function orb = periodicOrbit(this, mu_s, mu_d, vb)
            % Stick-slip orbit 3 -> 2 -> 3 for the friction levels
            % mu_s > mu_d >= 0 and the table speed vb > 0, in closed form.
            % Around its equilibrium x = mu_d/k, the slip 2 is the harmonic
            % oscillator of pulsation w = sqrt(k/m), from the breakaway point
            % (x - mu_d/k = (mu_s - mu_d)/k, v = vb) to its mirror image
            % (-(mu_s - mu_d)/k, vb), where it sticks; the table then carries
            % the mass at the speed vb up to the breakaway point:
            %   x_b = mu_s/k,  x_e = (2*mu_d - mu_s)/k,  tau_st = (x_b - x_e)/vb,
            %   tau_sl = (pi + 2*alpha)/w,  tan(alpha) = vb*sqrt(k*m)/(mu_s - mu_d)
            assert(vb > 0 && mu_s > mu_d && mu_d >= 0, 'FrictionConstantSpeedTableSubSystemClass:orbit', ...
                'The stick-slip orbit needs vb > 0 and mu_s > mu_d >= 0.');
            point = @(x) [x; vb; vb; mu_s; mu_d];     % v = vb at both switches
            xi_e = point((2*mu_d - mu_s)/this.k);
            xi_b = point(mu_s/this.k);
            tau_st = 2*(mu_s - mu_d)/(this.k*vb);
            tau_sl = (pi + 2*atan(vb*sqrt(this.k*this.m)/(mu_s - mu_d)))/sqrt(this.k/this.m);
            residual = norm(expm(this.flowMatrix(3)*tau_st)*xi_e - xi_b) ...
                + norm(expm(this.flowMatrix(2)*tau_sl)*xi_b - xi_e);
            assert(residual < 1e-10, 'FrictionConstantSpeedTableSubSystemClass:orbit', 'The orbit does not close.');
            orb = struct('xi_e', xi_e, 'xi_b', xi_b, 'tau_st', tau_st, 'tau_sl', tau_sl, ...
                'z_e', this.k*xi_e(1), 'mu_s', mu_s, 'mu_d', mu_d, 'vb', vb, 'residual', residual);
        end

        function x = jumpPoint(this, edge)
            % Jump point of the edge [q, q_next], for the tests: on the orbit
            % of periodicOrbit(0.6, 0.4, 0.2) for the stick entries and the
            % breakaways (mirrored by T for mode 1), at s = 0 with
            % |z| = 0.7 > mu_s for the direct reversals.
            orb = this.periodicOrbit(0.6, 0.4, 0.2);
            switch sprintf('%d%d', edge(1), edge(2))
                case '23', x = [orb.xi_e; 2];
                case '32', x = [orb.xi_b; 3];
                case '13', x = [this.T*orb.xi_e; 1];
                case '31', x = [this.T*orb.xi_b; 3];
                case '21', x = [-0.7/this.k; 0.2; 0.2; 0.6; 0.4; 2];
                case '12', x = [0.7/this.k; -0.2; -0.2; 0.6; 0.4; 1];
                otherwise, error('FrictionConstantSpeedTableSubSystemClass:edge', 'No jump %d -> %d.', edge(1), edge(2));
            end
        end

        function [X, names] = jumpPoints(this)
            % The jump points of every edge, one per column, and their edges.
            edges = [2 3; 3 2; 1 3; 3 1; 2 1; 1 2];
            X = zeros(6, size(edges, 1));
            names = cell(1, size(edges, 1));
            for i = 1:size(edges, 1)
                X(:, i) = this.jumpPoint(edges(i, :));
                names{i} = sprintf('%d->%d', edges(i, 1), edges(i, 2));
            end
        end
    end

    methods (Static)
        function A = modeMatrix(q, m, k)
            A = zeros(5);
            A(1, 2) = 1;
            switch q
                case 1, A(2, [1 5]) = [-k, -1]/m;
                case 2, A(2, [1 5]) = [-k, 1]/m;
            end
        end
        function R = resetMatrix(q, q_next)
            % xi+ = R*xi at the jump q -> q_next: the stick entry sets v = vb,
            % R = I - e2*c with c = [0 1 -1 0 0]; every other jump is the identity.
            R = eye(5);
            if q < 3 && q_next == 3
                R(2, :) = [0 0 1 0 0];
            end
        end
        function H = outputMatrix(measurement)
            switch measurement
                case 'position', H = [1 0 0 0 0];
                case 'velocity', H = [0 1 0 0 0];
                otherwise, error('FrictionConstantSpeedTableSubSystemClass:measurement', ...
                        'measurement must be ''position'' or ''velocity''.');
            end
        end
        function q = modeOf(x)
            % q is piecewise constant: rounding makes every derivative with
            % respect to q zero (DerivativeTest perturbs it by 1e-6).
            q = round(x(end));
        end
        function x = randomState()
            % Random state with a valid mode, for DerivativeTest.
            x = [randn(5, 1); randi(3)];
        end
    end
end
