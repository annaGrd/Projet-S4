%v = 1;
%w = .5;
%path = [0, 1, 1; 0, 0, 2; 0, 0, 0; 0, 0, 0];
%xa = [0;0;0;0];
%traj = pathToTraj_(xa, path, v, w);

function traj = pathToTraj(xa, path, v, w)
    nbPoints = length(path(1, :));
    
    points = path;
    points(:, 1) = xa;
    
    timeValues = zeros(1, nbPoints);
    t = 0;
    for i = 2:nbPoints
        travelTime = sqrt((points(1, i) - points(1, i-1))^2 + (points(2, i) - points(2, i-1))^2 + (points(3, i) - points(3, i-1))^2)/v;
        rotationTime = abs(points(4, i) - points(4, i-1))/w;
        t = t + max(travelTime, rotationTime);
        timeValues(i) = t;
    end
    
    if nbPoints == 1
        points = [xa, xa];
        timeValues = [0, 1];
    end

    %Interpolation avec des cubic splines
    tt = 0:.1:timeValues(length(timeValues));
    xx = spline(timeValues, [0 points(1, :) 0], tt);
    yy = spline(timeValues, [0 points(2, :) 0], tt);
    zz = spline(timeValues, [0 points(3, :) 0], tt);
    psi = spline(timeValues, [0 points(4, :) 0], tt);

    timeValues = tt;
    points = [xx;yy;zz;psi];

    traj = timeseries(points, timeValues);
    
end