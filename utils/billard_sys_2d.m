classdef billard_sys_2d < HybridSubsystem
    % A 2D bouncing-ball hybrid system with an unknown ground height.
    %
    % The state is
    %   x = [x; y; vx; vy; h]
    % where
    %   - (x, y) is the ball position,
    %   - (vx, vy) is the velocity,
    %   - h is the ground height, treated as an unknown constant state
    %     component that is preserved by the flow and reset at impact.

    properties
        g = 9.8;              % gravitational acceleration
        restitution = 0.8;    % coefficient of restitution
    end

    properties(SetAccess = immutable)
        x_index = 1;
        y_index = 2;
        vx_index = 3;
        vy_index = 4;
        ground_height_index = 5;
    end

    methods
        function this = billard_sys_2d()
            state_dim = 5;
            input_dim = 0;
            output_dim = 2;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function xdot = flowMap(this, x, ~, ~, ~)
            % Continuous dynamics: free fall in the plane.
            xdot = [x(this.vx_index);
                    x(this.vy_index);
                    0;
                    -this.g;
                    0];
        end

        function w = guard(this, x)
            % Guard condition: impact occurs when the ball reaches the
            % ground surface y = h.
            y = x(this.y_index);
            h = x(this.ground_height_index);
            w = y - h;
        end

        function dw = guardGradient(this, ~)
            % Gradient of the guard with respect to the state.
            dw = [0;
                  1;
                  0;
                  0;
                 -1];
        end

        function xplus = jumpMap(this, x, ~, ~, ~)
            % Reset map at impact: reflect the vertical velocity and
            % reset the position to the ground level.
            vy = x(this.vy_index);
            h = x(this.ground_height_index);

            xplus = [x(this.x_index);
                     h;
                     x(this.vx_index);
                     -this.restitution * vy;
                     h];
        end

        function inC = flowSetIndicator(this, x, ~, ~, ~)
            % The ball is in the flow set when it is above the ground or
            % moving upward.
            inC = (this.guard(x) >= 0) || (x(this.vy_index) >= 0);
        end

        function inD = jumpSetIndicator(this, x, ~, ~, ~)
            % The ball is in the jump set when it is on or below the
            % ground and moving downward.
            inD = (this.guard(x) <= 0) && (x(this.vy_index) <= 0);
        end
    end
end
