function []=MyPrintPDF(fig_h, filename)
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
    set(fig_h,'Units','inches',...
        'Position',[0.5 0.5 7 6],...
        'PaperPositionMode','auto','PaperUnits','inches','PaperSize',[7 6])
    grid on
    set(get(hAxes, 'xlabel'), 'Interpreter', 'Latex', 'Fontsize', 34)
    set(get(hAxes, 'ylabel'), 'Interpreter', 'Latex', 'Fontsize', 34)
    set(get(hAxes, 'title'), 'Interpreter', 'Latex', 'Fontsize', 24)

    print(fig_h, filename, '-dpdf')