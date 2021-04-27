%% plot an ellipse with orientation and return the plot handle
function h = plot_ellipse_3D(ax, pos, ell, orient, varargin)
    % plot an ellipse with orientation and return the plot handle
    % 
    % inputs:
    %   - ax: figure handle
    %   - pos: ellipse center, [2x1]
    %   - ell: ellipse size, [2x1]
    %   - orient: ellipse orientation, 1
    %   - varargin: patch properties
    % 
    % outputs: 
    %   - h: plot handle
    % 
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    % 
    
    % generate plot data

    xc = pos(1);
    yc = pos(2);
    zc = pos(3);
    
    xr = ell(1);
    yr = ell(2);
    zr = ell(3);
    
    [x, y, z] = ellipsoid(xc, yc, zc, xr, yr, zr, 20);
    h = surf(ax, x, y, z, varargin{:});
    axis equal
    
end
