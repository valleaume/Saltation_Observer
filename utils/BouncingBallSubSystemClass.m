classdef BouncingBallSubSystemClass < HybridSubsystem
    % A bouncing ball modeled as a HybridSystem subclass.

    % Define variable properties that can be modified.
    properties
        g = 9.8;      % Acceleration due to gravity.
        lambda = 0.8; % Coefficient of restitution.
        mu = 2;       % Coefficient of additive velocity.
        f_air = 0.01; % Coefficient of air friction.
         
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
        function this = BouncingBallSubSystemClass()
            % Constructor for instances of the BouncingBall class.
            % Call the constructor for the HybridSystem superclass and
            % pass it the state dimension. This is not strictly necessary, 
            % but it enables more error checking.
            state_dim = 2;
            input_dim = 0;
            output_dim = 1;
            output_fnc = @(x, u) x(1);
            this = this@HybridSubsystem(state_dim, input_dim, output_dim, output_fnc);
        end


        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(this, x, u, t, j)
            % Extract the state components.
            v = x(this.velocity_index);
            % Define the value of the flow map f(x). 
            xdot = [v; -this.g - sign(v) * this.f_air*v^2];
        end
        function xplus = jumpMap(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            % Define the value of the jump map g(x). 
            xplus = [0; -this.lambda*v + this.mu];
        end
        
        function inC = flowSetIndicator(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = (h >= 0) || (v >= 0);
        end
        function inD = jumpSetIndicator(this, x, u, t, j)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);

            % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
            inD = (h <= 0) && (v <= 0); % We choose h <= 0 insead of h == 0 in order to better detect jumps.
        end

        % Derivatives used by the saltation analysis (hardcoded).
        function Dg = jumpJacobian(this, x)
            % Jacobian of jumpMap: g(x) = [0; -lambda*v + mu].
            Dg = [0, 0; 0, -this.lambda];
        end
        function gradGuard = guardGradient(this, x)
            % Gradient of the guard h(x) = height.
            gradGuard = [1, 0];
        end
        function H = outputJacobian(this)
            % Jacobian of the output y = height.
            H = [1, 0];
        end

        function S = saltationMatrix(this, x)
            % Saltation matrix of the plant at a jump point x (on the guard).
            f_minus = this.flowMap(x, 0, 0, 0);
            f_plus = this.flowMap(this.jumpMap(x, 0, 0, 0), 0, 0, 0);
            S = saltationMatrix(this.jumpJacobian(x), this.guardGradient(x), f_minus, f_plus);
        end

        function [Xi, H, Htil] = saltationFactors(this, x)
            % Parts of the observer error saltation matrices that do not
            % depend on the jump gain L_d (observer with K = 0):
            %   M_before = Xi - L_d*H,   M_after = Xi - L_d*Htil
            f_minus = this.flowMap(x, 0, 0, 0);
            f_plus = this.flowMap(this.jumpMap(x, 0, 0, 0), 0, 0, 0);
            gradGuard = this.guardGradient(x);
            H = this.outputJacobian();
            Xi = saltationMatrix(this.jumpJacobian(x), gradGuard, f_minus, f_plus);
            Htil = H + H*(f_plus - f_minus)*gradGuard/(gradGuard*f_minus);
        end

        function [M_before, M_after] = errorSaltationMatrices(this, x, L_d)
            % Saltation matrices of the observer error at the plant jump
            % point x, for an observer with jump gain L_d and K = 0.
            % M_before: observer jumps before the plant, M_after: after.
            [Xi, H, Htil] = this.saltationFactors(x);
            M_before = Xi - L_d*H;
            M_after = Xi - L_d*Htil;
        end
    end
end