function [sign_jump, synchronized] = whoJumpsFirst(j_plant, j_observer)
% WHOJUMPSFIRST - Tell whether the observer jumped before or after the plant.
%
%   [sign_jump, synchronized] = whoJumpsFirst(j_plant, j_observer)
%
% j_plant and j_observer are the jump counters of the plant and of the
% observer on the same time grid, e.g. sol('Plant').j and sol('Observer').j
% for a CompositeHybridSystem solution. At each sample, for the last jump
% mismatch so far:
%
%   sign_jump = -1   the observer jumped before the plant
%               +1   the observer jumped after the plant
%                0   no mismatch yet
%
% The sign is set while the counters differ and kept once they agree again,
% until the next mismatch. synchronized is true where the counters agree.
% Both outputs are column vectors, like the t and j of the solution.

if numel(j_plant) ~= numel(j_observer)
    error('whoJumpsFirst:size', 'j_plant and j_observer must have the same length.');
end

plant_ahead = sign(j_plant(:) - j_observer(:));   % +1 plant ahead, -1 observer ahead
sign_jump = zeros(size(plant_ahead));
last = 0;
for i = 1:numel(plant_ahead)
    if plant_ahead(i) ~= 0
        last = plant_ahead(i);
    end
    sign_jump(i) = last;
end
synchronized = (plant_ahead == 0);
end
