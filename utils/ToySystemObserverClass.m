classdef ToySystemObserverClass < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        speed = 1;    % Speed
        half_D = 1;   % Geometric parameter
        half_C = 3;   % Geometric parameter
        L_c = 1;
        L_d = 1;
        K_jump = 0.05;
         
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        state_index = 1;
        obs_index = 2;

    end
    methods 
        function this = ToySystemObserverClass()
            % Constructor for instances of the ToySystemClass class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 1;  %dim x
            input_dim = 1;  %dim y
            output_dim = 1;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim);
        end


        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);

            % Define the value of the flow map f(x). 
            xdot = this.speed + (u - x^2)*this.L_c;
        end
        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);
    
            % Define the value of the jump map g(x). 
            xplus = -x + (u - x^2)*this.L_d;
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);
            x_c = x + this.K_jump*(u-x^2);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = ( abs(x_c)-0.9 >= 0) && ( x_c-this.half_C <= 0);
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);
            x_c = x + this.K_jump*(u-x^2);

            % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
            inD = (( -x_c-1 < 0)&&(x_c < 0)) || ( x_c-this.half_C > 0); 
        end
    end
end