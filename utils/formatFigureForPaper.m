function formatFigureForPaper(fig_h, options)
% FORMATFIGUREFORPAPER - Apply the paper style to a figure, in place
%
%   formatFigureForPaper(fig)
%   formatFigureForPaper(fig, 'Style', style)   % 'small', 'medium' (default) or 'big'
%
% Hides the titles (axes, tiledlayout and sgtitle: they go in the paper
% captions), sets the plotted objects to line width 3, the axis labels and
% legends to the LaTeX interpreter, and all fonts (ticks, labels, legends)
% to the size of the style: 14, 20 or 37 pt.
% myPrintPDF applies it to a copy before exporting; call it directly to
% preview the result on screen.

arguments
    fig_h (1,1) matlab.ui.Figure
    options.Style {mustBeMember(options.Style, {'small', 'medium', 'big'})} = 'medium'
end

font_sizes = struct('small', 14, 'medium', 20, 'big', 37);

axes_handles = findall(fig_h, 'Type', 'axes');
layouts = findall(fig_h, 'Type', 'tiledlayout');

set([property(axes_handles, 'Title'); property(layouts, 'Title'); ...
    findall(fig_h, 'Type', 'subplottext')], 'Visible', 'off');

% Plotted objects (lines, scatter, xline, ...) but not the axes themselves
plot_handles = findall(axes_handles, '-property', 'LineWidth');
plot_handles(ismember(plot_handles, axes_handles)) = [];
set(plot_handles, 'LineWidth', 3);

set([property(axes_handles, 'XLabel'); property(axes_handles, 'YLabel'); ...
    findall(fig_h, 'Type', 'legend')], 'Interpreter', 'latex');
fontsize(fig_h, font_sizes.(char(options.Style)), 'points');
end

function values = property(handles, name)
% Column of the graphics objects handles(k).(name).
values = gobjects(numel(handles), 1);
for k = 1:numel(handles)
    values(k) = handles(k).(name);
end
end
