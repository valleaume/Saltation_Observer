function []=MyPrintPDF(fig_h, filename, size_inch)
    % Prints a PDF of a figure, with the correct Fonts/LineWidth etc.
    % h = handle of the figure
    % hERE
    
     
    hAxes=get(fig_h,'CurrentAxes');
    %{ 
    set(get(hAxes, 'xlabel'), 'Interpreter', 'Latex', 'Fontsize', 54)
    set(get(hAxes, 'ylabel'), 'Interpreter', 'Latex', 'Fontsize', 54)
    set(get(hAxes, 'title'), 'Interpreter', 'Latex', 'Fontsize', 24)
    %}
    leg_h = get(hAxes, 'Legend');
    %if ~isempty(fig_h.legend)
    %    leg_h = fig_h.legend;
    if ~isempty(leg_h)
        set(leg_h,'Interpreter','Latex','Fontsize',25)
    end

    if nargin < 3 || isempty(size_inch)
        size_inch = [15, 12];
    end
    %xlim_ = get(hAxes, Xlim);
    %set(gco,'Interpreter','Latex','Fontsize',14)
    %{
    set(hAxes,...
    'Units','normalized',...
    'FontSize',25,...
    'Position',[.1 .1 .85 .85],'xLimMode','manual', 'yLimMode',...
    'auto','zLimMode','auto','XGrid','on','YGrid','on','ZGrid','on')
    %}
    set(hAxes,...
    'FontSize',25)

    plots=get(hAxes,'Children');
    set(plots,'LineWidth', 3)
    pos_size = [0.2 0.2 size_inch(1) size_inch(2)];
    set(fig_h,'Units','inches',...
        'Position',pos_size,...
        'PaperPositionMode','auto','PaperUnits','inches','PaperSize',size_inch)
    grid on
    set(get(hAxes, 'xlabel'), 'Interpreter', 'Latex', 'Fontsize', 34)
    set(get(hAxes, 'ylabel'), 'Interpreter', 'Latex', 'Fontsize', 34)
    set(get(hAxes, 'title'), 'Interpreter', 'Latex', 'Fontsize', 24)

    print(fig_h, filename, '-dpdf')