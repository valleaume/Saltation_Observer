function filename = myPrintPDF(fig_h, filename, size_inch)
% MYPRINTPDF - Export a figure as a vector PDF formatted for the papers
%
%   myPrintPDF(fig, filename)              % figure of 15 x 12 inches
%   myPrintPDF(fig, filename, size_inch)   % [width, height] in inches
%   file = myPrintPDF(...)                 % path of the written PDF
%
% The paper style (formatFigureForPaper) is applied to a hidden copy, so the
% displayed figure is unchanged. The '.pdf' extension is added if missing.
% exportgraphics crops the PDF to its content (plus 0.1 inch from R2025a),
% so the page is about size_inch or smaller.

arguments
    fig_h (1,1) matlab.ui.Figure
    filename {mustBeTextScalar}
    size_inch (1,2) double {mustBePositive} = [15, 12]
end

[folder, base_name] = fileparts(char(filename));
filename = fullfile(folder, [base_name, '.pdf']);

export_fig = copyobj(fig_h, groot);
close_export_fig = onCleanup(@() close(export_fig));
set(export_fig, 'Visible', 'off', 'Units', 'inches');
export_fig.Position(3:4) = size_inch;

formatFigureForPaper(export_fig);

options = {'ContentType', 'vector'};
if ~isMATLABReleaseOlderThan('R2025a')
    % Small margin so labels are not flush with the page border
    options = [options, {'Padding', 0.1, 'Units', 'inches'}];
end
exportgraphics(export_fig, filename, options{:});
end
