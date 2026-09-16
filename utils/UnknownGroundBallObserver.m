classdef UnknownGroundBallObserver < HybridSubsystem
    % Constant-gain observer for UnknownGroundBallSubSystemClass.
    %
    %   xhat = [xh1; xh2; xh3],  input y = x1,  r = y - xh1
    %
    %   flow : xhatdot = [xh2; -gamma - sign(xh2)*f_air*xh2^2; 0] + L_c * r
    %   jump : xhat+   = [xh3; -lambda*xh2 + mu; xh3]             + L_d * r
    %
    % Observer guard (kappa = 0 recovers the Lemma-2 design with O_D = { xh2 <= zeno_margin}, omega_hat
    % independent of y, hence Lambda_omega_hat = 0):
    %
    %   omega_hat(xhat,y) = (xh1 - xh3) + kappa * r
    %   Dhat = {omega_hat <= 0, xh2 <= 0},   Chat = cl(Dhat^c)
    %
    % The jump map above is the ONLY structure consistent with the
    % differential identity  dG/dxhat = dg/dx - L_d*H.  Do NOT add terms
    % proportional to (xh1 - xh3): they vanish at the observer's own jump
    % (where omega_hat = 0) and therefore change nothing in the implemented
    % observer, while corrupting the saltation matrices. See README.

    properties
        lambda = 0.5;
        mu     = 2.0;
        gamma  = 9.8;
        f_air  = 0.0;
        zeno_margin = 1e-6;  % margin for omega_hat <= 0 to avoid Zeno

        L_c = [3.5; 49.0; 0.0];   % flow gain   (poles -1.75 +/- 6.78i)
        L_d = [1.5; -3.2; 1.0];   % jump gain
        kappa = 0.0;              % guard gain on r; 0 = Lemma 2 design
    end

    methods
        function obj = UnknownGroundBallObserver()
            state_dim  = 3;
            input_dim  = 1;    % y
            output_dim = 3;
            obj = obj@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function xdot = flowMap(this, xhat, y, ~, ~)
            r = y - xhat(1);
            xdot = [ xhat(2);
                    -this.gamma - sign(xhat(2))*this.f_air*xhat(2)^2;
                     0 ] + this.L_c * r;
        end

        function xplus = jumpMap(this, xhat, y, ~, ~)
            r = y - xhat(1);
            xplus = [ xhat(3);
                     -this.lambda*xhat(2) + this.mu;
                      xhat(3) ] + this.L_d * r;
        end

        function inC = flowSetIndicator(this, xhat, y, ~, ~)
            om = this.omegaHat(xhat, y);
            inC = (om >= 0) || (xhat(2) >= -this.zeno_margin);
        end

        function inD = jumpSetIndicator(this, xhat, y, ~, ~)
            om = this.omegaHat(xhat, y);
            inD = (om <= 0) && (xhat(2) <= -this.zeno_margin);
        end

        function y = output(~, xhat, ~, ~, ~)
            y = xhat;
        end
    end

    methods
        function om = omegaHat(this, xhat, y)
            r  = y - xhat(1);
            om = (xhat(1) - xhat(3)) + this.kappa * r;
        end

        function A_F = flowJacobian(this)
            % dF/dxhat at (x, h(x)), frictionless case
            A_F = [0 1 0; 0 0 0; 0 0 0] - this.L_c*[1 0 0];
        end

        function [Mb, Ma] = saltationMatrices(this, v)
            % Saltation matrices at impact speed v > 0 (so x2 = -v),
            % for kappa = 0.  Frictionless case only.
            %
            %   M_before = Xi - L_d*H
            %   M_after  = Xi - L_d*Htilde,     Htilde = H + (delta/d)*w
            [Xi, H, Htil] = this.saltationFactors(v);
            Mb = Xi - this.L_d*H;
            Ma = Xi - this.L_d*Htil;
        end

        function [Xi, H, Htil] = saltationFactors(this, v)
            e = this.lambda;  g = this.gamma;  m = this.mu;
            x2 = -v;
            dgdx  = [0 0 1; 0 -e 0; 0 0 1];
            N     = [e*x2 - m; g*(1+e); 0];      % dg/dx*f - f(g(x))
            d     = x2;                          % domega/dx * f
            delta = -(1+e)*x2 + m;               % L_f h(g(x)) - L_f h(x)
            w     = [1 0 -1];                    % domega_hat/dxhat (kappa = 0)
            H     = [1 0 0];
            Xi    = dgdx - (N/d)*w;
            Htil  = H + (delta/d)*w;
        end
    end
end