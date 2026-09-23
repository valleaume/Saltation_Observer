function MyPrintPDF(fig_h, filename, size_inch)
    % Export a figure as a tightly sized, vector PDF.
    % All axes in the figure are included, including subplot/tiled layouts.

    if nargin < 2
        error('MyPrintPDF:MissingInput', ...
            'A figure handle and an output filename are required.');
    end
    if ~ishghandle(fig_h, 'figure')
        error('MyPrintPDF:InvalidFigure', ...
            'The first input must be a valid figure handle.');
    end
    if nargin < 3 || isempty(size_inch)
        size_inch = [15, 12];
    end
    if ~isnumeric(size_inch) || numel(size_inch) ~= 2 || any(size_inch <= 0)
        error('MyPrintPDF:InvalidSize', ...
            'size_inch must contain two positive dimensions in inches.');
    end
    size_inch = reshape(size_inch, 1, 2);

    [folder, base_name, extension] = fileparts(filename);
    if isempty(extension) || ~strcmpi(extension, '.pdf')
        filename = fullfile(folder, [base_name, '.pdf']);
    end

    axes_handles = findall(fig_h, 'Type', 'axes');
    legend_handles = findall(fig_h, 'Type', 'Legend');

    % Hide titles only for the export; the source figure is restored on exit.
    title_handles = gobjects(0, 1);
    title_visibility = cell(0, 1);
    for axis_index = 1:numel(axes_handles)
        title_handle = get(axes_handles(axis_index), 'Title');
        if ishghandle(title_handle)
            title_handles(end + 1, 1) = title_handle;
            title_visibility{end + 1, 1} = get(title_handle, 'Visible');
            set(title_handle, 'Visible', 'off');
        end

        set(axes_handles(axis_index), 'FontSize', 25);
        set(get(axes_handles(axis_index), 'XLabel'), ...
            'Interpreter', 'latex', 'FontSize', 34);
        set(get(axes_handles(axis_index), 'YLabel'), ...
            'Interpreter', 'latex', 'FontSize', 34);

        plot_handles = findall(axes_handles(axis_index), '-property', 'LineWidth');
        plot_handles(plot_handles == axes_handles(axis_index)) = [];
        if ~isempty(plot_handles)
            set(plot_handles, 'LineWidth', 3);
        end
    end

    for legend_index = 1:numel(legend_handles)
        set(legend_handles(legend_index), 'Interpreter', 'latex', 'FontSize', 25);
    end

    figure_text = findall(fig_h, 'Type', 'text');
    for text_index = 1:numel(figure_text)
        text_tag = get(figure_text(text_index), 'Tag');
        if strcmpi(text_tag, 'sgtitle') || strcmpi(text_tag, 'suptitle')
            title_handles(end + 1, 1) = figure_text(text_index);
            title_visibility{end + 1, 1} = get(figure_text(text_index), 'Visible');
            set(figure_text(text_index), 'Visible', 'off');
        end
    end

    restore_titles = onCleanup(@() restoreTitleVisibility( ...
        title_handles, title_visibility));

    set(fig_h, 'Units', 'inches', ...
        'Position', [0.2, 0.2, size_inch], ...
        'PaperUnits', 'inches', ...
        'PaperPosition', [0, 0, size_inch], ...
        'PaperSize', size_inch, ...
        'PaperPositionMode', 'manual');

    print(fig_h, filename, '-dpdf', '-painters');
end

function restoreTitleVisibility(title_handles, title_visibility)
    for title_index = 1:numel(title_handles)
        if ishghandle(title_handles(title_index))
            set(title_handles(title_index), 'Visible', title_visibility{title_index});
        end
    end
end