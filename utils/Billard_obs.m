classdef Billard_obs < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        v_0 = 1;        % Initial velocity of the ball.
        l = 3;
        L = [1.4142, 0; 1, 0; 0, 0*1.4142; 0, 0*1; 0, 0 ];          % observer gain
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        x_index = 1;
        
        % The index of 'velocity' component 
        % within the state vector 'x'. 
        y_index = 3;
        x_dot_index = 2;
        y_dot_index = 4;
        theta_index = 5;
    end
    methods 
        function this = Billard_obs()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 5;  %dim x
            input_dim = 2;  %dim y
            output_dim = 1; %dim hat x
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        function X_0 = init_cond(this, x)
            x_1 = x(1);
            x_2 = x(2);
            theta = x(3);
            X_0 = [x_1; this.v_0*cos(theta); x_2; this.v_0*sin(theta); theta];
        end

        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            theta = x(this.theta_index);
            v_1 = x(this.x_dot_index);


            A_x = [0, 1, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0*1, 0;  0, 0, 0, 0, 0; 0, 0, 0, 0, 0];
            L_x = this.L;
            C = [1, 0, 0, 0, 0; 0, 0, 1, 0, 0];
            % Define the value of the flow map f(x). 
            l_gain = this.l;
            xdot = A_x*x + diag([l_gain, l_gain^2, l_gain, 0*l_gain^2*(-v_1/(real(sqrt(this.v_0)^2-v_1^2))), l_gain^1])*L_x*(u-C*x)+[0; 0; 1*sign(sin(theta))*this.v_0*real(sqrt(1-(v_1/this.v_0)^2)); 0; 0]; %0*this.v_0*sin(theta); 0];     
        
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

        function theta_p = principal_value( theta)
            % Returns the principal value of an angle in radians (-pi to pi)
            theta_p = mod(theta + pi, 2*pi) - pi;
        end

        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            x_1 = x(this.x_index);
            x_2 = x(this.y_index);
            v_x = x(this.x_dot_index);
            theta = x(this.theta_index);

            phi = this.normal_angle(x);

            theta_hat = real(acos(v_x/this.v_0)*sign(sin(theta)));
            %disp(v_x/this.v_0)
            %disp(theta_hat)
            theta = real((theta_hat + theta_hat)/2);


            v_plus = this.v_0*([cos(theta); sin(theta)]-2*cos(theta-phi)*[cos(phi); sin(phi)]);
            %disp(v_plus);
            theta_plus = atan2(v_plus(2), v_plus(1)); %angle of velocity after reflection
            % Define the value of the jump map g(x). 

            xplus = [x_1; this.v_0*cos(theta_plus); x_2; this.v_0*sin(theta_plus); theta_plus ];
        end

        function rotation_mat = R_mat(theta)
            rotation_mat = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end

        function phi = normal_angle(this, x)
            dw = this.guardGradient(x);
            %disp('dw')
            %disp(dw)
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