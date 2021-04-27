%% plot an ellipse with orientation and return the plot handle
function h = plot_square_2D(ax, pos, size, varargin)
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
    position = [pos(1) - size(1)/2, pos(2) - size(2)/2, size(1), size(2)];

    % plot the ellipse
    h = rectangle(ax, 'Position', position, varargin{:});
    
end
