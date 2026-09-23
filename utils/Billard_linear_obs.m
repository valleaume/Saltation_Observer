classdef Billard_linear_obs < HybridSubsystem
    % Observer for the 3-state billiard system with a linear correction
    % added both during flow and at jumps.

    properties
        v_0 = 1;
        l = 30;
        L_c = [9.99998908e-03; -5.54571215e-19; -1.70625787e-17];
        L_d_square = [
            -9.99999084e-01,  9.99999085e-01, -9.99999084e-01,  9.99999085e-01;
             2.42011474e-16, -1.22470723e-16,  1.88664638e-16,  1.22463186e-16;
            -1.32930841e-16, -1.99999817e+00, -1.15206729e-16,  1.99999817e+00
        ];
    end

    properties(SetAccess = immutable)
        x_index = 1;
        y_index = 2;
        theta_index = 3;
        C = [1, 0, 0];
    end

    methods
        function this = Billard_linear_obs()
            state_dim = 3;
            input_dim = 2;
            output_dim = 1;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function X_0 = init_cond(this, x)
            X_0 = x;
        end

        function xdot = flowMap(this, x, u, t, j)
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);

            % Plant copy
            f = [this.v_0*cos(theta); this.v_0*sin(theta); 0];

            if isempty(u)
                y = x_1;
            else
                y = u(1);
            end
            e = y - this.C*x;

            % Linear correction during flow
            xdot = f + this.l*this.L_c*e;
        end

        function w = guard(this, x)
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            w = 0.5 - 0.5*(x_1^2 + x_2^2);
        end

        function dw = guardGradient(this, x)
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            dw = [-x_1; -x_2];
        end

        function theta_p = principal_value(this, theta)
            theta_p = mod(theta + pi, 2*pi) - pi;
        end

        function xplus = jumpMap(this, x, u, t, j)
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);

            phi = this.normal_angle(x);

            vx = this.v_0*cos(theta);
            vy = this.v_0*sin(theta);

            v_plus = this.v_0*([cos(theta); sin(theta)] - 2*cos(theta-phi)*[cos(phi); sin(phi)]);
            theta_plus = atan2(v_plus(2), v_plus(1));

            xplus_base = [x_1; x_2; theta_plus];

            if isempty(u)
                y = x_1;
            else
                y = u(1);
            end
            e = y - this.C*x;

            % choose quadrant-based correction column index
            if x_1 <= -1/2
                idx = 1; % quadrant I
            elseif x_1 > 1/2 
                idx = 3; % quadrant II
            elseif  x_2 < -1/2
                idx = 4; % quadrant III
            else
                idx = 2; % quadrant IV
            end

            Ld_wall = this.L_d_square(:, idx);
            % Linear correction at the jump
            xplus = xplus_base + Ld_wall*e;
        end

        function rotation_mat = R(this, theta)
            rotation_mat = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end

        function phi = normal_angle(this, x)
            dw = this.guardGradient(x);
            phi = atan2(dw(2), dw(1));
        end

        function inC = flowSetIndicator(this, x, u, t, j)
            inC = (-1 < this.guard(x));
        end

        function inD = jumpSetIndicator(this, x, u, t, j)
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);
            phi = this.normal_angle(x);
            inD = (0 >= this.guard(x)) && (cos(phi-theta) <= 0);
        end
    end
end