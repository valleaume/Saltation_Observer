classdef billard_obs_2d < HybridSubsystem
    % Observer for the 2D bouncing-ball system with unknown ground height.
    %
    % The observer state is the same as the plant state:
    %   x = [x; y; vx; vy; h]
    % and it uses a linear correction term during flow and at jumps.

    properties
        g = 9.8;              % gravitational acceleration
        restitution = 0.8;    % coefficient of restitution
        l = 1;                % observer gain scaling

        % Linear correction gain used during continuous flow.
        L_c = [1, 0;
               0, 1;
               0.2, 0;
               0.2, 0;
               0, 0];

        % Linear correction gain used at jumps.
        L_d = [1, 0;
               0, 1;
               0.2, 0;
               0.2, 0;
               0, 0];

        % Output matrix for position measurements [x; y].
        C = [1, 0, 0, 0, 0;
             0, 1, 0, 0, 0];
    end

    properties(SetAccess = immutable)
        x_index = 1;
        y_index = 2;
        vx_index = 3;
        vy_index = 4;
        ground_height_index = 5;
    end

    methods
        function this = billard_obs_2d()
            state_dim = 5;
            input_dim = 2;
            output_dim = 2;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function X_0 = init_cond(this, ~)
            X_0 = zeros(5, 1);
        end

        function xdot = flowMap(this, x, u, ~, ~)
            % Plant-copy flow plus linear output-error correction.
            xdot_base = [x(this.vx_index);
                         x(this.vy_index);
                         0;
                         -this.g;
                         0];

            if isempty(u)
                y = zeros(2, 1);
            else
                y = u(:);
            end
            e = y - this.C*x;

            xdot = xdot_base + this.l*this.L_c*e;
        end

        function w = guard(this, x, u)
            % Impact occurs when the ball reaches the ground.
            y = x(this.y_index);
            y_measured = u(2);
            h = x(this.ground_height_index);
            w = y - h;% - (y-y_measured)^2/00.02;
        end

        function dw = guardGradient(this, ~)
            dw = [0;
                  1;
                  0;
                  0;
                 -1];
        end

        function xplus = jumpMap(this, x, u, ~, ~)
            % Jump map with an additional linear correction term.
            vy = x(this.vy_index);
            h = x(this.ground_height_index);

            xplus_base = [x(this.x_index);
                          x(this.y_index);
                          x(this.vx_index);
                          -this.restitution*vy;
                          h];

            if isempty(u)
                y = zeros(2, 1);
            else
                y = u(:);
            end
            e = y - this.C*x;

            xplus = xplus_base + this.l*this.L_d*e;
            %xplus(this.ground_height_index) = h;
        end

        function phi = normal_angle(this, x)
            dw = this.guardGradient(x);
            phi = atan2(dw(2), dw(1));
        end

        function inC = flowSetIndicator(this, x, u, ~, ~)
            inC = (this.guard(x, u) >= 0) || (x(this.vy_index) >= 0);
        end

        function inD = jumpSetIndicator(this, x, u, ~, ~)
            inD = (this.guard(x, u) <= 0) && (x(this.vy_index) <= 0); %  || (x(this.y_index) - u(2) < -0.1)hard code
        end
    end
end
