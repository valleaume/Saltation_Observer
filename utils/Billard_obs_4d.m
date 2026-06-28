classdef Billard_obs_4d < HybridSubsystem
    % Observer for the bouncing ball with state [x1; x1dot; x2; x2dot]

    properties
        l = 1; % observer tuning
        % observer gain for the single measurement y = x1 (4x1)
        %L_c = [1.4142; 1; 0; 0];
        L_c = [ 0.68440069; 1.48911843; -1.25136612 ;-0.76382869];
        % jump-time linear correction columns for quadrants 1..4 (4x4)
        % columns assigned as requested: col3, col2, col1, col4
        L_d_all = [11.07831135, -4.48277155, -3.0389842, 2.02085714;
        -0.39661948,  2.20383996, 0.09395072, -1.19096221;
        -1.96218402, 0.88675902, 0.37897709, -0.33025135;
         7.2239355, -1.14385092, -2.29205928, 0.42398462]';

        % %L_d_all = [
        %     11.07831135,  7.2239355,  -1.96218402, -0.39661948;
        %     -4.48277155, -1.14385092,  0.88675902,  2.20383996;
        %     -3.0389842,  -2.29205928,  0.37897709,  0.09395072;
        %     2.02085714,   0.42398462, -0.33025135, -1.19096221
        % ];
        %Ld_all = [Ld_1, this.Ld_2, this.Ld_3, this.Ld_4];
        L_d = zeros(4,4);
    end

    properties(SetAccess = immutable)
        x1_index = 1;
        x1dot_index = 2;
        x2_index = 3;
        x2dot_index = 4;
        % System matrices defined as properties
        A = [0, 1, 0, 0;
             0, 0, 0, 0;
             0, 0, 0, 1;
             0, 0, 0, 0];
        % Measurement matrix: y = x1
        C = [1, 0, 0, 0];
    end

    methods
        function this = Billard_obs_4d()
            state_dim = 4;
            input_dim = 1;  % two measurements (x1 and x2)
            output_dim = 2;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function X_0 = init_cond(this, x)
            % initialize observer state from plant state if available
            % expect x = [x1; x1dot; x2; x2dot] or at least positions
            if numel(x) >= 4
                X_0 = x(1:4);
            else
                X_0 = zeros(4,1);
            end
        end

        function xdot = flowMap(this, x, u, t, j)
            % x = [x1; x1dot; x2; x2dot]
            A_x = [0, 1, 0, 0;
                   0, 0, 0, 0;
                   0, 0, 0, 1;
                   0, 0, 0, 0];
            % use A and C properties
            A_x = this.A;
            C = this.C;
            L_c = this.L_c;

            % observer correction
            l_gain = this.l;
            K = diag([l_gain, l_gain^2, l_gain, l_gain^2]);

            if isempty(u)
                y = C*x; % use measurement from state if u not provided
            else
                y = u;   % scalar measurement y = x1
            end

            xdot = A_x*x + K*this.L_c*(y - this.C*x);
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

            phi = this.normal_angle(x);
            n = [cos(phi); sin(phi)];

            v_plus = v - 2*(dot(n, v))*n;

            % base post-jump state (positions unchanged, velocities reflected)
            xplus_base = [x1; v_plus(1); x2; v_plus(2)];

            % measurement (scalar) and innovation
            if isempty(u)
                y = this.C*x;
            else
                y = u;
            end
            e = y - this.C*x;

            % choose quadrant-based correction column index
            if x1 <= -1/2
                idx = 1; % quadrant I
            elseif x1 > 1/2 
                idx = 3; % quadrant II
            elseif  x2 < -1/2
                idx = 4; % quadrant III
            else
                idx = 2; % quadrant IV
            end
            
            % apply linear jump-time correction
            xplus = xplus_base + this.L_d_all(:, idx) * e;
        end

        function inC = flowSetIndicator(this, x, u, t, j)
            inC = (-1 < this.guard(x));
        end

        function inD = jumpSetIndicator(this, x, u, t, j)
            v = [x(this.x1dot_index); x(this.x2dot_index)];
            phi = this.normal_angle(x);
            n = [cos(phi); sin(phi)];

            inD = (0 >= this.guard(x)) && (dot(n, v) <= 0);
        end
    end
end
