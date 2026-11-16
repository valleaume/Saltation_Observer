classdef FrictionConstantSpeedTableObserver < HybridSubsystem
    % Constant-gain observer for FrictionConstantSpeedTableSubSystemClass:
    % estimates the mass states, the table speed AND both friction levels.
    %
    %   xhat = [xh; vh; vbh; mush; mudh; qh],  input y,  r = y - H*xhat(1:5)
    %
    %   flow : xhat' = A_qh*xhat + Lc(qh)*r
    %   jump : xhat+ = R*xhat + Ld(qh, qh+)*r,  R the plant reset (vh = vbh at a stick entry)
    %          slip 1, 2: qh+ = 3 (stick entry), or 2 / 1 if |zh| > mush (direct reversal)
    %          stick 3:   qh+ = 2 / 1 (breakaway), as in the plant
    %
    % Gains of the orbit 3 -> 2 -> 3 (vb > 0); slip 1 (vb < 0) uses the
    % mirrored gains -T*L (both outputs are odd under T):
    %   L_c2 (slip 2), L_c1 = -T*L_c2, L_c3 (stick, must satisfy L_c3 = -T*L_c3)
    %   L_d23 (stick entry from 2), L_d13 = -T*L_d23, L_d32 (breakaway into 2), L_d31 = -T*L_d32
    %   L_d21 (direct reversal 2 -> 1), L_d12 = -T*L_d21
    %
    % Lemma 2 sets, omega_hat(xhat, y) = omega(xhat), O_D threshold c = mudh:
    %   qh = 1: Dhat = {sh <= 0, zh >= -c}      qh = 3: Dhat = {|zh| >= mush}
    %   qh = 2: Dhat = {sh >= 0, zh <=  c}      Chat = cl(Dhat^c)
    % The conditions on zh are the O_D sets: the stick entry of the orbit,
    % z_e = 2*mu_d - mu_s, is inside; right after a breakaway zh = +/-mush is
    % outside, so the observer does not jump back to stick (no Zeno). Lemma 2
    % allows any c between z_e and mu_s; c = mudh is the plant's own set
    % (sh' >= 0) and the middle of that interval (docs/guard_and_O_D.md).
    % After a direct reversal 2 -> 1, zh < -mush < -c, outside Dhat of mode 1 as well.

    properties
        m = 1;          % mass and stiffness, known (same as the plant)
        k = 1;
        L_c2  = zeros(5, 1);
        L_c3  = zeros(5, 1);
        L_d23 = zeros(5, 1);
        L_d32 = zeros(5, 1);
        % Direct reversal: kept at 0. On the reference orbit the mass always
        % sticks (|z| = |2*mu_d - mu_s| = 0.2 < mu_s = 0.6 at s = 0), so this
        % jump never occurs near it: it has no saltation matrix in the
        % certificate and no gain to design. With 0 the observer only switches
        % mode, without correction.
        L_d21 = zeros(5, 1);
    end

    properties (SetAccess = immutable)
        measurement     % 'position' or 'velocity'
    end

    methods
        function obj = FrictionConstantSpeedTableObserver(measurement)
            if nargin < 1, measurement = 'position'; end
            FrictionConstantSpeedTableSubSystemClass.outputMatrix(measurement);   % validates
            obj = obj@HybridSubsystem(6, 1, 6);
            obj.measurement = measurement;
        end

        function xdot = flowMap(this, xhat, y, ~, ~)
            q = FrictionConstantSpeedTableSubSystemClass.modeOf(xhat);
            r = y - this.H()*xhat(1:5);
            xdot = [this.flowMatrix(q)*xhat(1:5) + this.flowGain(q)*r; 0];
        end

        function xplus = jumpMap(this, xhat, y, ~, ~)
            q = FrictionConstantSpeedTableSubSystemClass.modeOf(xhat);
            q_next = this.nextMode(xhat);
            r = y - this.H()*xhat(1:5);
            R = FrictionConstantSpeedTableSubSystemClass.resetMatrix(q, q_next);
            xplus = [R*xhat(1:5) + this.jumpGain(q, q_next)*r; q_next];
        end

        function inC = flowSetIndicator(this, xhat, ~, ~, ~)
            [s, z, c] = this.setVariables(xhat);
            switch FrictionConstantSpeedTableSubSystemClass.modeOf(xhat)
                case 1, inC = s >= 0 || z <= -c;
                case 2, inC = s <= 0 || z >= c;
                case 3, inC = abs(z) <= xhat(4);
            end
        end

        function inD = jumpSetIndicator(this, xhat, ~, ~, ~)
            [s, z, c] = this.setVariables(xhat);
            switch FrictionConstantSpeedTableSubSystemClass.modeOf(xhat)
                case 1, inD = s <= 0 && z >= -c;
                case 2, inD = s >= 0 && z <= c;
                case 3, inD = abs(z) >= xhat(4);
            end
        end
    end

    methods
        function A = flowMatrix(this, q)
            A = FrictionConstantSpeedTableSubSystemClass.modeMatrix(q, this.m, this.k);
        end

        function H = H(this)
            H = FrictionConstantSpeedTableSubSystemClass.outputMatrix(this.measurement);
        end

        function L = flowGain(this, q)
            switch q
                case 1, L = -FrictionConstantSpeedTableSubSystemClass.T*this.L_c2;
                case 2, L = this.L_c2;
                case 3, L = this.L_c3;
            end
        end

        function L = jumpGain(this, q, q_next)
            T = FrictionConstantSpeedTableSubSystemClass.T;
            switch sprintf('%d%d', q, q_next)
                case '23', L = this.L_d23;
                case '13', L = -T*this.L_d23;
                case '32', L = this.L_d32;
                case '31', L = -T*this.L_d32;
                case '21', L = this.L_d21;      % 0, never used near the orbit
                case '12', L = -T*this.L_d21;
                otherwise, error('FrictionConstantSpeedTableObserver:edge', 'No observer jump %d -> %d.', q, q_next);
            end
        end

        function q_next = nextMode(this, xhat)
            % Same rule as the plant, with the estimated mu_s.
            [~, z] = this.setVariables(xhat);
            mu_s = xhat(4);
            switch FrictionConstantSpeedTableSubSystemClass.modeOf(xhat)
                case 1, q_next = 3 - (z > mu_s);         % stick, or reversal to 2
                case 2, q_next = 3 - 2*(z < -mu_s);      % stick, or reversal to 1
                case 3, q_next = 1 + (z > 0);            % breakaway
            end
        end

        function [s, z, c] = setVariables(this, xhat)
            % c: threshold of the O_D sets, the estimated mu_d.
            s = xhat(2) - xhat(3);
            z = this.k*xhat(1);
            c = xhat(5);
        end

        function A = flowJacobian(this, xhat)
            A = blkdiag(this.errorFlowMatrix(FrictionConstantSpeedTableSubSystemClass.modeOf(xhat)), 0);
        end

        function A_F = errorFlowMatrix(this, q)
            % The error flows with A_q - Lc(q)*H when both are in mode q.
            A_F = this.flowMatrix(q) - this.flowGain(q)*this.H();
        end

        function [Mb, Ma] = saltationMatrices(this, x_jump)
            % Mb (observer jumps before), Ma (after) at a plant jump point
            % x_jump = [xi; q] (on the guard), x1..x5 block:
            %   Mb = Xi - Ld*H,   Ma = Xi - Ld*Htil
            [Xi, H, Htil] = this.saltationFactors(x_jump);
            Ld = this.jumpGain(FrictionConstantSpeedTableSubSystemClass.modeOf(x_jump), this.nextMode(x_jump));
            Mb = Xi - Ld*H;
            Ma = Xi - Ld*Htil;
        end

        function [Xi, H, Htil] = saltationFactors(this, x_jump)
            % Saltation factors (12) at a plant jump point x_jump = [xi; q],
            % for the jump q -> q_next of nextMode (the plant rule). With the
            % reset xi+ = Dg*xi (resetMatrix), f- = A_q*xi, f+ = A_q_next*Dg*xi,
            % w the guard gradient and den = w*f- (transversality: den ~= 0):
            %   Xi = Dg + (f+ - Dg*f-)*w/den,   Htil = H + H*(f+ - f-)*w/den
            % Only v and vdot change at a jump, and in every case
            % Xi = I - kappa*e2*w,  Htil = H - kappa*(H*e2)*w  (Htil = H for y = x):
            %   2 -> 3 stick entry: w = [0 -1 1 0 0], kappa = -1, Xi = I - e2*c
            %                       (the v error becomes the vb error)
            %   3 -> 2 breakaway:   w = [-k 0 0 1 0], kappa = -(mu_s - mu_d)/(m*k*vb) on the orbit
            %                       (the mu_s error enters the velocity)
            %   2 -> 1 reversal:    w = [0 -1 1 0 0], kappa = 2 mu_d/(z - mu_d)
            % and their mirror images 1 -> 3, 3 -> 1, 1 -> 2.
            % At the stick entry Dg = I - e2*c, and f+ = Dg*f- on the guard: Xi = Dg,
            % the same matrix as with the identity reset, where the timing
            % term (f+ - f-)*w/den = -e2*c gave it. Htil does not involve Dg.
            xi = x_jump(1:5);
            q = FrictionConstantSpeedTableSubSystemClass.modeOf(x_jump);
            q_next = this.nextMode(x_jump);
            [~, z] = this.setVariables(x_jump);
            switch q
                case 1, w = [0 1 -1 0 0];
                case 2, w = [0 -1 1 0 0];
                case 3, w = [-sign(z)*this.k, 0, 0, 1, 0];
            end
            Dg = FrictionConstantSpeedTableSubSystemClass.resetMatrix(q, q_next);
            f_minus = this.flowMatrix(q)*xi;
            f_plus = this.flowMatrix(q_next)*Dg*xi;
            den = w*f_minus;
            if abs(den) < 1e-12
                error('FrictionConstantSpeedTableObserver:notTransverse', 'The flow is tangent to the guard at this jump point.');
            end
            H = this.H();
            Xi = Dg + (f_plus - Dg*f_minus)*w/den;
            Htil = H + H*(f_plus - f_minus)*w/den;
        end
    end

    methods (Static)
        function x = randomState()
            % Random state with a valid mode, for DerivativeTest.
            x = FrictionConstantSpeedTableSubSystemClass.randomState();
        end
    end
end
