classdef UnknownGroundBallSubSystemClass < HybridSubsystem
    % Bouncing ball above an UNKNOWN, constant ground height.
    %
    %   x = [x1; x2; x3]   x1 = absolute height, x2 = velocity,
    %                      x3 = ground height (constant, unknown)
    %
    %   flow :  xdot = [x2; -gamma - sign(x2)*f_air*x2^2; 0]     on C = {x1 >= x3}
    %   jump :  x+   = [x3; -lambda*x2 + mu; x3]                 on D = {x1 <= x3, x2 <= 0}
    %   output: y    = x1              (absolute position is measured)
    %
    % Guard: omega(x) = x1 - x3  (height above ground).
    % Note h(g(x)) = h(x) on D, so the jump is invisible in y: this is the
    % whole point of the unknown-jump-time setting.
    %
    % NOTE ON THE JUMP MAP. On D we have x1 = x3, so x1+ = x3 and x1+ = x1
    % define the SAME physical map but different Jacobians dg/dx. The choice
    % below (x1+ = x3, x3+ = x3) is the one used in the accompanying analysis.
    % See README: this non-uniqueness is a genuine modelling choice, but it is
    % constrained -- the observer jump map G must satisfy the DIFFERENTIAL
    % consistency identity  dG/dxhat + (dG/dy)(dh/dx) = dg/dx  on D, not merely
    % G(x,h(x)) = g(x).

    properties
        lambda = 0.5;   % restitution coefficient (called e in the notes)
        mu     = 2.0;   % velocity added at each impact (mu > 0 removes Zeno)
        g      = 9.8;   % gravity
        f_air  = 0.0;   % quadratic air friction
    end

    methods
        function obj = UnknownGroundBallSubSystemClass()
            state_dim  = 3;
            input_dim  = 0;
            output_dim = 1;
            obj = obj@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function xdot = flowMap(this, x, ~, ~, ~)
            xdot = [ x(2);
                    -this.g - sign(x(2))*this.f_air*x(2)^2;
                     0 ];
        end

        function xplus = jumpMap(this, x, ~, ~, ~)
            xplus = [ x(3);
                     -this.lambda*x(2) + this.mu;
                      x(3) ];
        end

        function inC = flowSetIndicator(~, x, ~, ~, ~)
            inC = (x(1) - x(3) >= 0);
        end

        function inD = jumpSetIndicator(~, x, ~, ~, ~)
            inD = (x(1) - x(3) <= 0) && (x(2) <= 0);
        end

        function y = output(~, x, ~, ~, ~)
            y = x(1);
        end
    end

    methods
        function vstar = vStar(this)
            % Asymptotic impact speed: v_{j+1} = lambda*v_j + mu is a
            % contraction with fixed point mu/(1-lambda). Frictionless case only.
            vstar = this.mu/(1 - this.lambda);
        end
        function taustar = tauStar(this)
            % Asymptotic flight time between impacts (frictionless case only).
            taustar = 2*this.vStar()/this.g;
        end
    end
end