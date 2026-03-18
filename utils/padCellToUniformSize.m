function paddedCell = padCellToUniformSize(inputCell, padValue)
    % Pads all arrays in a cell to the maximum dimensions found in the cell.
    % Usage: paddedCell = padCellToUniformSize(inputCell, padValue)
    %   inputCell: Cell array of numeric arrays.
    %   padValue: Value to use for padding (e.g., NaN, 0).

    if nargin < 2
        padValue = NaN; % Default padding value
    end

    % Find max rows and columns
    maxRows = max(cellfun(@(x) size(x,1), inputCell));
    maxCols = max(cellfun(@(x) size(x,2), inputCell));

    % Pad each array in the cell
    paddedCell = cellfun( ...
        @(x) [x, padValue * ones(size(x,1), maxCols - size(x,2)); ...
               padValue * ones(maxRows - size(x,1), maxCols)], ...
        inputCell, 'UniformOutput', false);
end

%AI generated