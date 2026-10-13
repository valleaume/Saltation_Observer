classdef (Abstract) SaltationModel < handle
    % SALTATIONMODEL - Saltation analysis shared by the plant classes
    %
    % Mixin for a HybridSubsystem with a guard h(x) = 0. The subclass
    % provides its flowMap/jumpMap and the hand-written derivatives below;
    % this class builds the saltation matrices from them.
    %
    %   classdef MyPlant < HybridSubsystem & SaltationModel
    %
    % Naming convention (checked numerically by tests/unit/DerivativeTest.m):
    %   jumpJacobian(x)  = d jumpMap / dx
    %   guardGradient(x) = d guard / dx
    %   outputJacobian() = d output / dx
    %   flowJacobian(x)  = d flowMap / dx      (optional)

    methods (Abstract)
        w = guard(this, x)
        Dg = jumpJacobian(this, x)
        gradGuard = guardGradient(this, x)
        H = outputJacobian(this)
    end

    methods
        function S = saltationMatrix(this, x)
            % Saltation matrix of the plant at a jump point x (on the guard).
            S = this.saltationFactors(x);
        end

        function [Xi, H, Htil] = saltationFactors(this, x)
            % Parts of the observer error saltation matrices that do not
            % depend on the jump gain L_d (observer with K = 0):
            %   M_before = Xi - L_d*H,   M_after = Xi - L_d*Htil
            % Xi is the saltation matrix of the plant.
            f_minus = this.flowMap(x, 0, 0, 0);
            f_plus = this.flowMap(this.jumpMap(x, 0, 0, 0), 0, 0, 0);
            gradGuard = this.guardGradient(x);
            H = this.outputJacobian();
            Xi = saltationMatrixFromDerivatives(this.jumpJacobian(x), gradGuard, f_minus, f_plus);
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
