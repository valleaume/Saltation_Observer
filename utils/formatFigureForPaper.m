function formatFigureForPaper(fig_h)
% FORMATFIGUREFORPAPER - Apply the paper style to a figure, in place
%
%   formatFigureForPaper(fig)
%
% Hides the titles (axes, tiledlayout and sgtitle: they go in the paper
% captions), sets the plotted objects to line width 3, the axis labels and
% legends to the LaTeX interpreter, and all fonts to 20 pt.
% myPrintPDF applies it to a copy before exporting; call it directly to
% preview the result on screen.

arguments
    fig_h (1,1) matlab.ui.Figure
end

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
fontsize(fig_h, 20, 'points');
end

function values = property(handles, name)
% Column of the graphics objects handles(k).(name).
values = gobjects(numel(handles), 1);
for k = 1:numel(handles)
    values(k) = handles(k).(name);
end
end
