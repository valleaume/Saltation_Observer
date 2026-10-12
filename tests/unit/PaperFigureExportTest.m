classdef PaperFigureExportTest < ProjectTestCase
    % formatFigureForPaper (paper style) and myPrintPDF (PDF export).

    properties
        Folder
    end

    methods (TestMethodSetup)
        function createTemporaryFolder(tc)
            tc.Folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
        end
    end

    methods (Test)
        % ---- formatFigureForPaper ----
        function titlesAreHidden(tc)
            [fig, ax, layout] = sampleFigure();
            formatFigureForPaper(fig);
            tc.verifyEqual(char(ax(1).Title.Visible), 'off');
            tc.verifyEqual(char(ax(2).Title.Visible), 'off');
            tc.verifyEqual(char(layout.Title.Visible), 'off');
        end

        function sgtitleIsHidden(tc)
            fig = figure();
            subplot(1, 2, 1); plot(1:3);
            subplot(1, 2, 2); plot(1:3);
            s = sgtitle(fig, 'global title');
            formatFigureForPaper(fig);
            tc.verifyEqual(char(s.Visible), 'off');
        end

        function plottedObjectsAreThick(tc)
            [fig, ax] = sampleFigure();
            formatFigureForPaper(fig);
            for obj = [findobj(ax(1), 'Type', 'line'); findobj(ax(1), 'Type', 'scatter'); ...
                       findobj(ax(1), 'Type', 'constantline')]'
                tc.verifyEqual(obj.LineWidth, 3, sprintf('%s not thickened.', obj.Type));
            end
            tc.verifyNotEqual(ax(1).LineWidth, 3, 'Axes box must keep its line width.');
        end

        function labelsAndLegendUseLatex(tc)
            [fig, ax] = sampleFigure();
            formatFigureForPaper(fig);
            tc.verifyEqual(char(ax(1).XLabel.Interpreter), 'latex');
            tc.verifyEqual(char(ax(1).YLabel.Interpreter), 'latex');
            tc.verifyEqual(char(ax(1).Legend.Interpreter), 'latex');
        end

        function fontsAre20Points(tc)
            [fig, ax] = sampleFigure();
            formatFigureForPaper(fig);
            tc.verifyEqual(ax(1).FontSize, 20);
            tc.verifyEqual(ax(1).XLabel.FontSize, 20);
            tc.verifyEqual(ax(1).Legend.FontSize, 20);
        end

        function formattingHandlesFigureWithoutAxes(tc)
            fig = figure();
            tc.verifyWarningFree(@() formatFigureForPaper(fig));
        end

        % ---- myPrintPDF ----
        function writesPdfFile(tc)
            fig = sampleFigure();
            file = myPrintPDF(fig, fullfile(tc.Folder, 'figure'));
            tc.verifyEqual(file, fullfile(tc.Folder, 'figure.pdf'));
            tc.assertTrue(isfile(file));
            tc.verifyTrue(startsWith(fileread(file), '%PDF'));
        end

        function extensionIsReplacedByPdf(tc)
            fig = sampleFigure();
            file = myPrintPDF(fig, fullfile(tc.Folder, 'figure.png'));
            tc.verifyEqual(file, fullfile(tc.Folder, 'figure.pdf'));
            tc.verifyTrue(isfile(file));
        end

        function acceptsStringAndNamesWithSpaces(tc)
            % e.g. 'Position error' in draw_figures_CDC_PDF.m
            fig = sampleFigure();
            file = myPrintPDF(fig, string(fullfile(tc.Folder, 'Position error')));
            tc.verifyTrue(isfile(file));
        end

        function pageSizeFollowsSizeInch(tc)
            fig = sampleFigure();
            for size_inch = {[4, 3], [15, 12]}
                page = pdfPageSizeInches(myPrintPDF(fig, fullfile(tc.Folder, 'sized'), size_inch{1}));
                % Cropped to the content: a bit smaller, never much larger
                tc.verifyLessThanOrEqual(page, size_inch{1} + 0.25);
                tc.verifyGreaterThan(page, 0.6*size_inch{1});
            end
        end

        function sourceFigureIsUnchanged(tc)
            [fig, ax, layout] = sampleFigure();
            line = findobj(ax(1), 'Type', 'line');
            n_figures = numel(findall(groot, 'Type', 'figure'));

            myPrintPDF(fig, fullfile(tc.Folder, 'figure'));

            tc.verifyEqual(char(ax(1).Title.Visible), 'on');
            tc.verifyEqual(char(layout.Title.Visible), 'on');
            tc.verifyEqual(line.LineWidth, 0.5);
            tc.verifyEqual(char(ax(1).XLabel.Interpreter), 'tex');
            tc.verifyNotEqual(ax(1).FontSize, 20);
            tc.verifyEqual(numel(findall(groot, 'Type', 'figure')), n_figures, ...
                'The export copy was not closed.');
        end

        function invalidInputsAreRejected(tc)
            fig = sampleFigure();
            file = fullfile(tc.Folder, 'x');
            tc.verifyError(@() myPrintPDF(42, file), 'MATLAB:validation:UnableToConvert');
            tc.verifyError(@() myPrintPDF(fig, file, [1, -1]), 'MATLAB:validators:mustBePositive');
            tc.verifyError(@() myPrintPDF(fig, file, [1, 2, 3]), 'MATLAB:validation:IncompatibleSize');
            tc.verifyError(@() myPrintPDF(fig), 'MATLAB:minrhs');
        end
    end
end

function [fig, ax, layout] = sampleFigure()
% Two tiles with title, labels, legend, line, scatter and xline.
fig = figure();
layout = tiledlayout(fig, 1, 2);
title(layout, 'layout title');
ax = gobjects(2, 1);
ax(1) = nexttile(layout);
plot(ax(1), 1:10, (1:10).^2);
hold(ax(1), 'on');
scatter(ax(1), 1:10, 1:10);
xline(ax(1), 3);
title(ax(1), 'left');
xlabel(ax(1), '$x$');
ylabel(ax(1), '$y$');
legend(ax(1), 'a', 'b');
ax(2) = nexttile(layout);
plot(ax(2), 1:3);
title(ax(2), 'right');
end

function page = pdfPageSizeInches(file)
% Width and height of the first page, from its /MediaBox (in points).
box = regexp(fileread(file), '/MediaBox\s*\[([^\]]*)\]', 'tokens', 'once');
box = sscanf(box{1}, '%f')';
page = (box(3:4) - box(1:2))/72;
end
