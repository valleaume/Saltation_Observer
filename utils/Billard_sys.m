classdef Billard_sys < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        v_0 = 1;        % Initial velocity of the ball.
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        x_index = 1;
        
        % The index of 'velocity' component 
        % within the state vector 'x'. 
        y_index = 2;
        theta_index = 3;
    end
    methods 
        function this = Billard_sys()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 3;  %dim x
            input_dim = 1;  %dim y
            output_dim = 2; %dim hat x
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);

            % Define the value of the flow map f(x). 
            xdot = [this.v_0*cos(theta); this.v_0*sin(theta); 0];
        
        end

        function w = guard(this, x)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);

            % Define the value of the guard function g(x). 
            w = 0.5-0.5*(x_1^2+ x_2^2);
        end

        function dw = guardGradient(this, x)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);

            % Define the value of the guard function g(x). 
            dw = [-x_1; -x_2];
        end

        function theta_p = principal_value(this, theta)
            % Returns the principal value of an angle in radians (-pi to pi)
            theta_p = mod(theta + pi, 2*pi) - pi;
        end

        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);

            phi = this.normal_angle(x);

            vx = this.v_0*cos(theta);
            vy = this.v_0*sin(theta);

            v_plus = this.R(-phi)*diag([-1; 1])*this.R(phi)*[vx; vy];
            %disp(v_plus)
            v_plus = this.v_0*([cos(theta); sin(theta)]-2*cos(theta-phi)*[cos(phi); sin(phi)]);
            %disp(v_plus);
            theta_plus = atan2(v_plus(2), v_plus(1)); %angle of velocity after reflection
            % Define the value of the jump map g(x). 

            xplus = [x_1; x_2; theta_plus ];
        end

        function rotation_mat = R(this, theta)
            rotation_mat = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end

        function phi = normal_angle(this, x)
            dw = this.guardGradient(x);

            phi = atan2(dw(2), dw(1)); %angle of normal vector to the guard surface dw
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);
            % Set 'inC' to 1 if 'hat{x}, y' is in the extended flow set $hat{C}$ and to 0 otherwise.
            phi = this.normal_angle(x);
            
            vx = this.v_0*cos(theta);
            vy = this.v_0*sin(theta);

            v_plus = this.R(phi)*[vx; vy];
            %vn = this.v_0*sin(theta-phi);
            inC =  (-1 < this.guard(x));
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);

            phi = this.normal_angle(x);
            
            %vn = this.v_0*sin(theta-phi);
            inD = (0 >= this.guard(x)) && (cos(phi-theta) <= 0);
        end

    end
end