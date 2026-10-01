classdef UtilsTest < ProjectTestCase
    % Small helpers of utils/.

    methods (Test)
        function padCellPadsToLargestSize(tc)
            padded = padCellToUniformSize({[1; 2; 3], [4; 5]}, NaN);
            tc.verifyEqual(cell2mat(padded), [1 4; 2 5; 3 NaN]);
        end

        function plotEllipseDrawsCovarianceContour(tc)
            C = [4 0; 0 1];
            plot_ellipse(C, [1; 2], 1);
            line = findobj(figure(1), 'Type', 'line');
            tc.verifyNumElements(line, 1);
            % 2-sigma ellipse: half-widths 2*sqrt(4) = 4 and 2*sqrt(1) = 2
            tc.verifyEqual([min(line.XData), max(line.XData)], [1 - 4, 1 + 4], 'AbsTol', 1e-2);
            tc.verifyEqual([min(line.YData), max(line.YData)], [2 - 2, 2 + 2], 'AbsTol', 1e-2);
        end

        function printPdfWritesFile(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            fig = figure();
            plot(1:3);
            file = fullfile(folder, 'test_figure');
            myPrintPDF(fig, file);
            tc.verifyTrue(isfile([file '.pdf']));
        end
    end
end
