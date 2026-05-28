classdef ASLIP_Hybrid < HybridSubsystem
    % ASLIP_HyEQ: Simulates the Asymmetric Spring Loaded Inverted Pendulum (ASLIP)
    %   using the Hybrid Equations Toolbox (HyEQ).
    %   This class implements the hybrid dynamics of the ASLIP, including flight and stance phases.

    properties

        m_b = 1;        % Body mass
        J_b = 1;      % Body moment of inertia
        l_b = 0.5;        % Distance from hip to COM
        a_g = 9.8;      % Gravitational acceleration
        k = 1000;       % Leg spring constant
        k_h = 400;      % Hip spring constant
        l_l0 = 1;       % Resting leg length
        theta_0 = -pi/8; % Resting hip angle
    end

    methods
        function obj = ASLIP_Hybrid()
            % Initialize the ASLIP system with default parameters.
            % State: [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode]
            % mode = 0 for flight, mode = 1 for stance.
            state_dim = 9; % 6 continuous states + 1 discrete mode
            input_dim = 0;  % No inputs
            output_dim = 3; % Body position (for visualization)
            output_fnc = @(x) [x(1); x(2); x(3)]; % Output the body position for visualization

            % Call the superclass constructor for HybridSystem
            obj = obj@HybridSubsystem(state_dim, input_dim, output_dim, output_fnc);
        end

        function G = compute_potential_energy_gradient(obj, ql)
            % Compute the gradient of the potential energy V with respect to q = [theta_l, theta_h, l_l]
            %
            % Args:
            %   obj: Instance of ASLIP_Hybrid
            %   ql: Generalized coordinates [theta_l, theta_h, l_l] (array)
            %
            % Returns:
            %   G: Gradient of V (array, shape=(3,))

            theta_l = ql(1);
            theta_h = ql(2);
            l_l = ql(3);

            % Compute partial derivatives
            dV_dtheta_l = obj.m_b * obj.a_g * (l_l * cos(theta_l) + obj.l_b * cos(theta_l + theta_h));
            dV_dtheta_h = obj.m_b * obj.a_g * obj.l_b * cos(theta_l + theta_h) + obj.k_h * (theta_h - obj.theta_0);
            dV_dll = obj.m_b * obj.a_g * sin(theta_l) + obj.k * (l_l - obj.l_l0);

            G = [dV_dtheta_l, dV_dtheta_h, dV_dll];
        end

        function qt = T_bt(obj, qb)
            % Transformation from body configuration to toe position
            %
            % Args:
            %   obj: Instance of ASLIP_Hybrid
            %   qb: Body configuration [x_b, y_b, theta_b] (array)
            %
            % Returns:
            %   qt: Toe position [x_t, y_t] (array)

            x_b = qb(1);
            y_b = qb(2);
            theta_b = qb(3);

            x_t = x_b - obj.l_b * cos(theta_b) - obj.l_l0 * cos(theta_b - obj.theta_0);
            y_t = y_b - obj.l_b * sin(theta_b) - obj.l_l0 * sin(theta_b - obj.theta_0);

            qt = [x_t; y_t];
        end

        function d_qt = T_bt_dqb(obj, qb)
            % Transformation from body configuration to toe position
            %
            % Args:
            %   obj: Instance of ASLIP_Hybrid
            %   qb: Body configuration [x_b, y_b, theta_b] (array)
            %
            % Returns:
            %   d_qt: Jacobian of toe position with respect to body configuration (2x3 array)

            x_b = qb(1);
            y_b = qb(2);
            theta_b = qb(3);

            x_t = x_b - obj.l_b * cos(theta_b) - obj.l_l0 * cos(theta_b - obj.theta_0);
            y_t = y_b - obj.l_b * sin(theta_b) - obj.l_l0 * sin(theta_b - obj.theta_0);

            d_qt = [1, 0, obj.l_b * sin(theta_b) + obj.l_l0 * sin(theta_b - obj.theta_0);
                     0, 1, -obj.l_b * cos(theta_b) - obj.l_l0 * cos(theta_b - obj.theta_0)];
        end

        function dqt_dql = T_bt_dql(obj, ql)
            % Transformation from leg configuration to toe position
            %
            % Args:
            %   obj: Instance of ASLIP_Hybrid
            %   ql: Leg configuration [theta_l, theta_h, l_l] (array)
            %
            % Returns:
            %   dqt_dql: Jacobian of toe position with respect to leg configuration (2x3 array)

            theta_l = ql(1);
            theta_h = ql(2);
            theta_b = theta_l + theta_h;
            l_l = ql(3);

            dqt_dql = [0, -l_l * sin(theta_b - theta_h),  cos(theta_b - obj.theta_0);
                       0,  l_l * cos(theta_b - theta_h),  sin(theta_b - obj.theta_0)];
        end

        function ql = T_bl(obj, qb, qt)
            % Transformation from body configuration and toe position to leg configuration
            %
            % Args:
            %   obj: Instance of ASLIP_Hybrid
            %   qb: Body configuration [x_b, y_b, theta_b]
            %   qt: Toe position [x_t, y_t]
            %   lb: Distance from hip to COM
            %
            % Returns:
            %   ql: Leg configuration [theta_t, theta_b_leg, l_l]

            x_b = qb(1);
            y_b = qb(2);
            theta_b = qb(3);
            lb = obj.l_b;

            x_t = qt(1);
            y_t = qt(2);

            % Compute A, B, r
            A = y_b - lb * sin(theta_b) - y_t;
            B = x_b - lb * cos(theta_b) - x_t;
            r = sqrt(A^2 + B^2);

            % Leg configuration
            theta_t = atan2(A, B);
            theta_b_leg = theta_b - atan2(A, B);
            l_l = r;

            ql = [theta_t; theta_b_leg; l_l];
        end
        % --- Flow Map (Continuous Dynamics) ---
        function xdot = flowMap(obj, x, u, t, j)
            % x = [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode]
            % mode = 0: flight, mode = 1: stance

            mode = x(9);
            xdot = zeros(9, 1);
            xdot(1:3) = x(4:6); % Position derivatives

            if mode == 0 % Flight phase
                % Ballistic motion of the body
                
                xdot(4) = 0;         % x_b velocity is constant in flight
                xdot(5) = -obj.a_g;     % y_b acceleration due to gravity
                xdot(6) = 0;         % theta_b velocity is constant in flight
                xdot(7) = x(4) + x(6) * ( obj.l_b * sin(x(3)) + obj.l_l0* sin(x(3)- obj.theta_0)); % theta_l_dot = x_b_dot + l_b * theta_b_dot * sin(theta_b)
                xdot(8) = x(5) - x(6) * ( obj.l_b * cos(x(3)) + obj.l_l0* cos(x(3)- obj.theta_0)); % theta_h_dot = y_b_dot - l_b * theta_b_dot * cos(theta_b)
            
            else % Stance phase
                % Lagrangian dynamics of the leg-body system
                M = diag([obj.m_b, obj.m_b, obj.J_b]); % Mass matrix for body
                qt_dot = [0; 0]; % [theta_l; theta_h]
                qb_dot = x(4:6); % [x_b; y_b; theta_b]

                qb = x(1:3); % Body configuration
                qt = x(7:8); % Toe position

                Jac = ASLIP_Compute_Jacobian(qb, qt, obj.l_b); % Jacobian of toe position w.r.t. body configuration
                Jac = Jac(1:3, 1:3); % Extract the relevant part of the Jacobian for leg configuration mapping
                
                % Coriolis matrice is null in the correct frame
                C = zeros(3, 3); % Coriolis matrix (assumed zero for simplicity)


                ql = obj.T_bl(qb, qt); % Leg configuration
                G_l = obj.compute_potential_energy_gradient(ql); % Gradient of potential energy
                %disp(G_l);
                %disp(Jac);
                G = (G_l*Jac)'; % Map to body coordinates
                %disp(C*x(4:6));
                %disp(M);
                % Acceleration terms: D*ddot_q = -C*dot_q - G

                ddot_q = M\(-C*x(4:6) - G);

                % State derivatives
    
                xdot(1:3) = x(4:6); % Position derivatives
                xdot(7:8) = 0;
                xdot(4:6) = ddot_q;
            end
            xdot(9) = 0; % Mode does not change during flow
        end

        % --- Jump Map (Discrete Reset) ---
        function xplus = jumpMap(obj, x, u, t, j)
            % x = [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode]
            % Apply the reset map depending on the mode
            x(9) = 1-x(9); 
            xplus = x; % For simplicity, we are not changing the continuous states here
            % mode = x(9);
            % if mode == 0 % Flight-to-stance transition
            %     % Reset to stance phase
            %     x_plus = flightToStanceReset(obj, x);
            % else % Stance-to-flight transition
            %     % Reset to flight phase
            %     x_plus = stanceToFlightReset(obj, x);
            % end
        end

        % --- Flow Set (Guard for Flow) ---
        function inC = flowSetIndicator(obj, x, u, t, j)
            % Define when the system is allowed to flow (i.e., not in a jump condition)
            % For ASLIP, flow is allowed except during jumps
            inC = true; % Always allow flow (jumps are handled by jumpSetIndicator)
        end

        % --- Jump Set (Guard for Jumps) ---
        function inD = jumpSetIndicator(obj, x, u, t, j)
            % Define when the system must jump (transition between phases)
            mode = x(9);
            if mode == 0 % Flight phase: jump to stance when toe touches ground
                % Toe position: (x_toe, y_toe) = (l * cos(phi+theta_b) + L*sin(theta_b), l * sin(phi+theta_b) - L*cos(theta_b))
                % Ground contact when y_toe <= 0
                y_toe = x(8);
                y_toe_dot = x(5) - x(6) * ( obj.l_b * cos(x(3)) + obj.l_l0* cos(x(3)- obj.theta_0));
                inD = (y_toe <= 0) && (y_toe_dot < 0); % Toe touches ground and is moving downward
            else % Stance phase: jump to flight when leg length exceeds l0
                l_l = sqrt((x(1) - obj.l_b*cos(x(3)) - x(7))^2 + (x(2) - obj.l_b*sin(x(3)) - x(8))^2); % Leg length
                Jac = ASLIP_Compute_Jacobian(x(1:3), x(7:8), obj.l_b);
                q_dot = [x(4:6); 0; 0]; % Body velocities and zero leg velocities
                l_l_dot = Jac(3, :) * q_dot; % Leg length rate of change
                inD = (l_l >= obj.l_l0) && (l_l_dot >= 0); % Leg length exceeds l0 and is extending
            end
        end

        function L = lagrangian(obj, x)
            % Compute the Lagrangian of the system for a given state x
            qb = x(1:3); % Body configuration
            qt = x(7:8); % Toe position
            ql = obj.T_bl(qb, qt); % Leg configuration

            % Kinetic energy (T) and potential energy (V)
            T = 0.5 * obj.m_b * (x(4)^2 + x(5)^2) + 0.5 * obj.J_b * x(6)^2; % Kinetic energy of the body
            V = obj.m_b * obj.a_g * qb(2) + ...
                + 0.5 * obj.k * (ql(3) - obj.l_l0)^2 + 0.5 * obj.k_h * (ql(2) - obj.theta_0)^2; % Potential energy

            L = T + V; % Lagrangian
        end

        % --- Helper Functions ---
        function x_plus = flightToStanceReset(obj, x)
            % Reset map for flight-to-stance transition.
            % x = [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode=0]
            % x_plus = [l; phi; theta_b; l_dot; phi_dot; theta_b_dot; mode=1]

            x_b = x(1); y_b = x(2); theta_b = x(3);
            x_b_dot = x(4); y_b_dot = x(5); theta_b_dot = x(6);

            % Leg length and angle at touchdown
            l = sqrt((x_b - obj.L*cos(theta_b))^2 + (y_b - obj.L*sin(theta_b))^2);
            phi = atan2(y_b - obj.L*sin(theta_b), x_b - obj.L*cos(theta_b)) - theta_b;

            % Stance state at touchdown: [l; phi; theta_b; l_dot; phi_dot; theta_b_dot; mode=1]
            x_plus = [l; phi; theta_b; x_b_dot; y_b_dot - obj.L*cos(theta_b)*theta_b_dot; theta_b_dot; 1];
        end

        function x_plus = stanceToFlightReset(obj, x)
            % Reset map for stance-to-flight transition.
            % x = [l; phi; theta_b; l_dot; phi_dot; theta_b_dot; mode=1]
            % x_plus = [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode=0]

            theta_b = x(3); theta_b_dot = x(6);
            l_dot = x(4);

            % Flight state at liftoff: [x_b; y_b; theta_b; x_b_dot; y_b_dot; theta_b_dot; mode=0]
            x_plus = x;
        end
    end
end
