% https://github.com/petercorke/spatialmath-matlab/blob/master/plot_ellipse.m

function plot_ellipse(C, mean, num_fig, varargin)
    
    npts=50;
    % plot the gaussian fits
    tt=linspace(0,2*pi,npts)';
    x = cos(tt); y=sin(tt);
    ap = [x(:) y(:)]';
    [v,d] = eig(C); 
    d = 2 * sqrt(d); % convert variance to sdwidth*sd
    %disp(v*d*ap)
    %disp(repmat(mean, 1, size(ap,2)))
    bp = (v*d*ap) + repmat(mean, 1, size(ap,2)); 
    figure(num_fig);
    hold on;
    plot(bp(1,:), bp(2,:), varargin{:});
end 