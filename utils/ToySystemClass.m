classdef ToySystemClass < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        speed = 1;    % Speed
        half_D = 1;   % Geometric parameter
        half_C = 3;   % Geometric parameter
         
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        state_index = 1;
        
    end
    methods 
        function this = ToySystemClass()
            % Constructor for instances of the ToySystemClass class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 1;
            input_dim = 0;
            output_dim = 1;
            output_fnc = @(x, u) x(1)^2;
            this = this@HybridSubsystem(state_dim, input_dim, output_dim, output_fnc);
        end


        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)

            % Define the value of the flow map f(x). 
            xdot = this.speed;
        end
        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);
    
            % Define the value of the jump map g(x). 
            xplus = -x;
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = ( abs(x)-0.9 >= 0)&&( x-this.half_C <= 0);
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x = x(this.state_index);

            % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
            inD = (( -x-1 <= 0)&&(x <= 0)) || ( x-this.half_C >= 0); 
        end
    end
end