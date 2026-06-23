classdef Billard_sys_4d < HybridSubsystem
    % Bouncing ball with state ordered as [x1; x1dot; x2; x2dot].

    properties
        % (no fixed-speed parameter; velocities are part of the state)
    end

    properties(SetAccess = immutable)
        x1_index = 1;
        x1dot_index = 2;
        x2_index = 3;
        x2dot_index = 4;
    end

    methods
        function this = Billard_sys_4d()
            state_dim = 4;
            input_dim = 1;
            output_dim = 2;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function xdot = flowMap(this, x, u, t, j)
            % x = [x1; x1dot; x2; x2dot]
            x1dot = x(this.x1dot_index);
            x2dot = x(this.x2dot_index);

            % continuous dynamics: position derivatives equal velocities;
            % assume no accelerations (constant velocity) between jumps
            xdot = [x1dot; 0; x2dot; 0];
        end

        function w = guard(this, x)
            x1 = x(this.x1_index);
            x2 = x(this.x2_index);
            w = 0.5 - 0.5*(x1^2 + x2^2);
        end

        function dw = guardGradient(this, x)
            x1 = x(this.x1_index);
            x2 = x(this.x2_index);
            dw = [-x1; -x2];
        end

        function phi = normal_angle(this, x)
            dw = this.guardGradient(x);
            phi = atan2(dw(2), dw(1));
        end

        function xplus = jumpMap(this, x, u, t, j)
            x1 = x(this.x1_index);
            x2 = x(this.x2_index);
            v = [x(this.x1dot_index); x(this.x2dot_index)];

            % unit normal to guard surface
            phi = this.normal_angle(x);
            n = [cos(phi); sin(phi)];

            % reflect velocity across normal: v_plus = v - 2*(n'*v)*n
            v_plus = v - 2*(dot(n, v))*n;

            xplus = [x1; v_plus(1); x2; v_plus(2)];
        end

        function inC = flowSetIndicator(this, x, u, t, j)
            % inside flow set when guard > -1 (same logic as original)
            inC = (-1 < this.guard(x));
        end

        function inD = jumpSetIndicator(this, x, u, t, j)
            % in jump set when on or inside guard and velocity points into
            % the surface (dot(n, v) <= 0)
            v = [x(this.x1dot_index); x(this.x2dot_index)];
            phi = this.normal_angle(x);
            n = [cos(phi); sin(phi)];

            inD = (0 >= this.guard(x)) && (dot(n, v) <= 0);
        end
    end
end
