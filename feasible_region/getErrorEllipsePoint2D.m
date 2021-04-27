%% get error ellipse points for plotting
function [xData, yData] = getErrorEllipsePoint2D(covP, mu, r, r_coll)
    % Input:  mean vector (2x1), covariance matrix (2x2) and Mahalanobis distance
    % Output: two-dimensional surface data


    n=100; % Number of points around ellipse
    p=0:pi/n:2*pi; % angles around a circle

    % 1-sigma ellipse
    [eigvec,eigval] = eig(covP);                        % Compute eigen-stuff
    xy = [cos(p'),sin(p')] * sqrt(eigval) * eigvec';    % Transformation

    % enlarge with Mahalanobis distance
    xy(:,1) = r*xy(:,1);
    xy(:,2) = r*xy(:,2);

    % adding extra robot radius
    for i = 1 : length(xy(:,1))
        d = norm(xy(i,:));
        xy(i,:) = xy(i,:)*((d+r_coll)/d);
    end

    % moving with mean vector
    xData = xy(:,1) + mu(1);
    yData = xy(:,2) + mu(2);

end