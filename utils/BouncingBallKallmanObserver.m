classdef BouncingBallKallmanObserver < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        gamma = 9.8;        % Acceleration due to gravity.
        lambda = 0.8;       % Coefficient of restitution.
        mu = 2;             % Coefficient of additive velocity.
        f_air = 0.0;        % Coefficient of air friction.
        K  = [0, 0];        % Linear correction terms applied to detect jumps -> not implemented
        lambda_kallman = 0.9;
        gamma_kallman = 0.78;
        gain = 0.2;
        L_c = [0.3; 0.1];   % Flow gain of the observer
        L_d = [3; 10];      % Jump/discrete gain of the observer
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        height_index = 1;
        
        % The index of 'velocity' component 
        % within the state vector 'x'. 
        velocity_index = 2;
        P_index_start = 3;
        P_index_end = 6;

    end
    methods 
        function this = BouncingBallKallmanObserver()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 6;  %dim x
            input_dim = 1;  %dim y
            output_dim = 2; %dim hat x
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            % import
            v = x(this.velocity_index);
            h = x(this.height_index);
            % define P
            P = reshape(x(this.P_index_start:this.P_index_end), [2,2]);
            f = [v; -this.gamma - sign(v) * this.f_air*v^2];

            % define matrices
            F = [0, 1; 0, -2*sign(v)*v*this.f_air];
            H = [1, 0];
            R_c_inv = eye(1)/this.gain;



            % Define the value of the flow map f(x). 
            xdot_partial = f + P*H'*R_c_inv*(u - h);
            P_dot = this.lambda_kallman*P + F*P + P*F'- P*H'*R_c_inv*H*P;

            xdot = [xdot_partial; reshape(P_dot, [4,1])];
        end
        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);

            P = reshape(x(this.P_index_start:this.P_index_end), [2,2]);
            f = [v; -this.gamma - sign(v) * this.f_air*v^2];

            % define matrices
            %F = [0, 1; 0, -2*sign(v)*v*this.f_air]
            H = [1, 0];
            R_d = this.gain*eye(1);

            v_plus = -this.lambda*v;
            f_plus = [v_plus; -this.gamma - sign(v_plus) * this.f_air*v_plus^2];
            w = [1; 0];
            J = [1, 0; 0, -this.lambda];
            % saltation matrix
            salt_before = J - (J*f - f_plus )/v*w';
            %salt_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/v*w';  saltation of initial system, after same as before
            % Define the value of the jump map g(x). 
            K_d = P*H'/(H*P*H'+ R_d);

            P_plus = 1/this.gamma_kallman*salt_before*P*salt_before'; % 1/this.gamma_kallman*salt_before*(eye(2)-K_d*H)*P*salt_before';

            x_plus_partial = [h; -this.lambda*v + this.mu] ;%+ J*K_d*(u - h);
            xplus = [x_plus_partial; reshape(P_plus, [4,1])];
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            x_c = [h; v] + this.K*(u-h);
            h_c = x_c(1);
            v_c = x_c(2);
            % Set 'inC' to 1 if 'hat{x}, y' is in the extended flow set $hat{C}$ and to 0 otherwise.
            inC = (h_c >= 0) || (v_c >= 0);
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            x_c = [h; v] + this.K*(u-h);
            h_c = x_c(1);
            v_c = x_c(2);
            % Set 'inD' to 1 if 'hat{x} + K(y-h(x))' is in the jump set and to 0 otherwise.
            inD = (h_c <= 0) && (v_c <= 0); % We choose h <= 0 insead of h == 0 in order to better detect jumps. 
        end
    end
end