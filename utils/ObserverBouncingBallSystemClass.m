classdef ObserverBouncingBallSystemClass < HybridSystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        gamma = 9.8;  % Acceleration due to gravity.
        lambda = 0.8; % Coefficient of restitution.
        mu = 2;       % Coefficient of additive velocity.
        f_air = 0.01; % Coefficient of air friction.
        Lc_1 = 3;
        Lc_2 = 5;
        Ld_1 = 1;
        Ld_2 = 1;
        k1 = 3;
         
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        height_index = 1;
        
        % The index of 'velocity' component 
        % within the state vector 'x'. 
        velocity_index = 2;

        % The index of 'height obs' component 
        % within the state vector 'x'. 
        height_index_obs = 3;
        
        % The index of 'velocity obs' component 
        % within the state vector 'x'. 
        velocity_index_obs = 4;
    end
    methods 
        function this = ObserverBouncingBallSystemClass()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 4;
            this = this@HybridSystem(state_dim);
        end
        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            h_obs = x(this.height_index_obs);
            v_obs = x(this.velocity_index_obs);
            % Define the value of the flow map f(x). 
            xdot = [v; -this.gamma - sign(v) * this.f_air*v^2; v_obs + this.Lc_1*(h - h_obs); -this.gamma - sign(v_obs) * this.f_air*v_obs^2 + this.Lc_2*(h - h_obs)];
        end
        function xplus = jumpMap(this, x, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            h_obs = x(this.height_index_obs);
            v_obs = x(this.velocity_index_obs);
            % Define the value of the jump map g(x). 
            if (h <= 0) && (h_obs + this.k1*(h-h_obs) <= 0) 
                 xplus = [0; -this.lambda*v + this.mu; h_obs + this.Ld_1*(h - h_obs); -this.lambda*v_obs + this.mu + this.Ld_2*(h - h_obs)];
            else
                if h_obs + this.k1*(h-h_obs) <= 0 
                    xplus = [h; v; h_obs + this.Ld_1*(h - h_obs); -this.lambda*v_obs + this.mu + this.Ld_2*(h - h_obs)];
                else
                    if h <= 0
                        xplus = [0; -this.lambda*v + this.mu; h_obs; v_obs];
                    end
                end
            end
        end
        
        function inC = flowSetIndicator(this, x, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            h_obs = x(this.height_index_obs);
            v_obs = x(this.velocity_index_obs);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = ((h >= 0) || (v >= 0)) && ((h_obs + this.k1*(h-h_obs) >= 0) || (v_obs >= 0));
        end
        function inD = jumpSetIndicator(this, x, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            h_obs = x(this.height_index_obs);
            v_obs = x(this.velocity_index_obs);

            % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
            inD = ((h <= 0) && (v <= 0)) || ((h_obs + this.k1*(h-h_obs) <= 0) && (v_obs <= 0)); % We choose h <= 0 insead of h == 0 in order to better detect jumps. 
        end
    end
end