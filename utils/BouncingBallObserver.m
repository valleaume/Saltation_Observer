classdef BouncingBallObserver < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        gamma = 9.8;        % Acceleration due to gravity.
        lambda = 0.8;       % Coefficient of restitution.
        mu = 2;             % Coefficient of additive velocity.
        f_air = 0.01;       % Coefficient of air friction.
        K  = [0; 0];        % Linear correction terms applied to detect jumps
        L_c = [0.3; 0.1];   %Flow gain of the observer
        L_d = [3; 10];      %Jump/discrete gain of the observer
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        height_index = 1;
        
        % The index of 'velocity' component 
        % within the state vector 'x'. 
        velocity_index = 2;
    end
    methods 
        function this = BouncingBallObserver()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 2;  %dim x
            input_dim = 1;  %dim y
            output_dim = 2; %dim hat x
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end

        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            v = x(this.velocity_index);
            h = x(this.height_index);
            % Define the value of the flow map f(x). 
            xdot = [v; -this.gamma - sign(v) * this.f_air*v^2] + (u - h)*this.L_c;
        end
        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            % Define the value of the jump map g(x). 
            xplus = [h; -this.lambda*v + this.mu]+ (u - h)*this.L_d;
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