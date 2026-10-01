function S = saltationMatrix(Dg, gradGuard, f_minus, f_plus)
% SALTATIONMATRIX - Saltation matrix of a hybrid system at a jump point
%
%   S = saltationMatrix(Dg, gradGuard, f_minus, f_plus)
%
% For a jump triggered when the guard h(x) = 0 is reached, with jump map g
% and flow map f, the saltation matrix maps a perturbation just before the
% jump to the perturbation just after it:
%
%   S = Dg + (f_plus - Dg*f_minus) * gradGuard / (gradGuard*f_minus)
%
% Inputs:
%   Dg        - Jacobian of the jump map at x (n x n)
%   gradGuard - gradient of the guard function at x (1 x n)
%   f_minus   - flow map evaluated at x, before the jump (n x 1)
%   f_plus    - flow map evaluated at g(x), after the jump (n x 1)
%
% The flow must be transverse to the guard (gradGuard*f_minus ~= 0).

transversality = gradGuard*f_minus;
if abs(transversality) < 1e-12
    error('saltationMatrix:notTransverse', ...
        'Flow is tangent to the guard (gradGuard*f = %g): saltation matrix undefined.', transversality);
end

S = Dg + (f_plus - Dg*f_minus)*gradGuard/transversality;
end
