%v = 1;
%w = .5;
%path = [0, 1, 1; 0, 0, 2; 0, 0, 0; 0, 0, 0];
%xa = [0;0;0;0];
%traj = pathToTraj_(xa, path, v, w);

function [traj, tStart] = pathToTraj(xa, path, v, w, traveledPath, speed)
    nbPoints = length(path(1, :));
    nbPointsTraveled = length(traveledPath(1, :));
    
    points = path;
    points(:, 1) = xa;

    pointsTraveled = traveledPath;
    
    timeValuesTraveled = zeros(1, nbPointsTraveled);
    t = 0;

    for i = 2:nbPointsTraveled
        travelTime = sqrt((pointsTraveled(1, i) - pointsTraveled(1, i-1))^2 + (pointsTraveled(2, i) - pointsTraveled(2, i-1))^2 + (pointsTraveled(3, i) - pointsTraveled(3, i-1))^2)/v;
        rotationTime = abs(pointsTraveled(4, i) - pointsTraveled(4, i-1))/w;
        t = t + max(travelTime, rotationTime);
        timeValuesTraveled(i) = t;
    end

    tStart = t;
    timeValues = zeros(1, nbPointsTraveled);

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
    tt = timeValues(1):.1:timeValues(length(timeValues));
    xx = spline(timeValues, [speed(1) points(1, :) 0], tt);
    yy = spline(timeValues, [speed(2) points(2, :) 0], tt);
    zz = spline(timeValues, [speed(3) points(3, :) 0], tt);
    psi = spline(timeValues, [0 points(4, :) 0], tt);

    if length(pointsTraveled(1, :)) > 1
        ttTraveled = 0:.1:timeValuesTraveled(length(timeValuesTraveled));
        xxTraveled = spline(timeValuesTraveled, [0 pointsTraveled(1, :) speed(1)], ttTraveled);
        yyTraveled = spline(timeValuesTraveled, [0 pointsTraveled(2, :) speed(2)], ttTraveled);
        zzTraveled = spline(timeValuesTraveled, [0 pointsTraveled(3, :) speed(3)], ttTraveled);
        psiTraveled = spline(timeValuesTraveled, [0 pointsTraveled(4, :) 0], ttTraveled);

        tt = cat(2, ttTraveled, tt);
        xx = cat(2, xxTraveled, xx);
        yy = cat(2, yyTraveled, yy);
        zz = cat(2, zzTraveled, zz);
        psi = cat(2, psiTraveled, psi);
    end

    

    timeValues = tt;
    points = [xx;yy;zz;psi];

    traj = timeseries(points, timeValues);
    
end