function results = runTests(scope)
% RUNTESTS - Run the project test suite
%
%   runTests()                 % unit + integration tests (a few minutes)
%   runTests('unit')           % fast unit tests only
%   runTests('integration')    % integration tests only (run the scripts)
%   runTests('examples')       % smoke tests of the Examples/ scripts (slow)
%   results = runTests(...)    % also return the TestResult array
%
% From a terminal:  matlab -batch "runTests"

import matlab.unittest.TestSuite
import matlab.unittest.selectors.HasTag

if nargin < 1
    scope = 'all';
end

project_folder = setupPaths();
tests_folder = fullfile(project_folder, 'tests');
addpath(fullfile(tests_folder, 'helpers'));

switch lower(scope)
    case 'unit'
        suite = TestSuite.fromFolder(fullfile(tests_folder, 'unit'));
    case 'integration'
        suite = TestSuite.fromFolder(fullfile(tests_folder, 'integration'));
        suite = suite.selectIf(~HasTag('Examples'));
    case 'examples'
        suite = TestSuite.fromFolder(fullfile(tests_folder, 'integration'));
        suite = suite.selectIf(HasTag('Examples'));
    case 'all'
        suite = [TestSuite.fromFolder(fullfile(tests_folder, 'unit')), ...
                 TestSuite.fromFolder(fullfile(tests_folder, 'integration'))];
        suite = suite.selectIf(~HasTag('Examples'));
    otherwise
        error('runTests:unknownScope', 'Unknown scope "%s".', scope);
end

res = run(suite);
disp(res);

if nargout > 0
    results = res;
else
    % Non-zero exit code for matlab -batch when a test fails
    assertSuccess(res);
end
end
