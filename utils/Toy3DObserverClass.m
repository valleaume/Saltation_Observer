classdef Toy3DObserverClass < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        A = [0, 1, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0];
        B = [0; 1; 0; 0];
        speed = 1;    % Speed 
        Lc = [5; 6; 0; 0]; % observer gain
        Ld = [0; 3; 2; 0]; % observer gain
        alpha = 0.5;
        C = [1, 0, 0, 0];
    end
    
    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component 
        % within the state vector 'x'. 
        position_index = 1;
        velocity_index = 2;
        wall_index = 3;  %wall is enforced on velocity, not position
        mode_index = 4;
        
    end
    methods 
        function this = Toy3DObserverClass()
            % Constructor for instances of the ToySystemClass class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 4;
            input_dim = 1;
            output_dim = 1;
            output_fnc = @(x, u) x(1);
            this = this@HybridSubsystem(state_dim, input_dim, output_dim, output_fnc);
        end


        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, y, t, j)

            % Define the value of the flow map f(x). 
            xdot = this.A*x + this.B*this.speed*x(this.mode_index) + this.Lc*(y-this.C*x);
        end
        function xplus = jumpMap(this, x, y, t, j)
            % Extract the state components.            
            q = x(this.mode_index);
            q_new = -q;
            q = x(this.mode_index);
            xplus = x;
            x2 = x(this.velocity_index);
            x3 = x(this.wall_index);

            if q == 1     
                xplus(this.wall_index) = this.alpha*x2 + (1-this.alpha)*x3;
                xplus =  x + this.Ld*(y-this.C*x);
            else
                xplus(this.wall_index) = -this.alpha*x2 + (1-this.alpha)*x3;
                xplus =  x + this.Ld*(y-this.C*x);
            end
    
            xplus(this.mode_index) = q_new;
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x2 = x(this.velocity_index);
            x3 = x(this.wall_index);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = true ;
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            x2 = x(this.velocity_index);
            x3 = x(this.wall_index);
            q = x(this.mode_index);
            if q == 1
                % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
                inD = (x2 >= x3);
            else
                inD = (x2 <= -x3);
            end
        end
    end
end