%v = 1;
%w = .5;
%path = [0, 1, 1; 0, 0, 2; 0, 0, 0; 0, 0, 0];
%xa = [0;0;0;0];
%traj = pathToTraj_(xa, path, v, w);

function [traj, xorigin, tStart] = pathToTraj(xa, path, v, w, speed)
        
    acceleration_duration = 5;
    points = path;
    points(:, 1) = xa;
    if length(points(1, :)) == 1
        points = [[0;0;0;0], [0;0;0;0]];
        timeValues = [0, 10];
        traj = timeseries(points, timeValues);
        xorigin = points(:, 1);
        tStart = 0;
    else

        timeValues = zeros(1, length(points(1, :)));
        tStart = 0;
    
        index_start = 1;
        if ~isequal(speed, [0;0;0;0])
            beginPoint = points(:, 1) - [acceleration_duration*speed(1:3)/distance_column_vectors(speed(1:3), [0;0;0]); 0];
            points = [beginPoint, points];
            timeValues = [0, timeValues];
            timeValues(2) = acceleration_duration;
            tStart = acceleration_duration;
            index_start = 2;
        end

        points

        xorigin = points(:, 1);
        points = points - xorigin;
    
        for j = index_start+1:length(timeValues)
            translation_time = distance_column_vectors(points(1:3, j), points(1:3, j-1))/v;
            rotation_time = abs(points(4, j) - points(4, j-1))/w;
            timeValues(j) = timeValues(j-1) + max(translation_time, rotation_time);
        end

        timeValues
        
        % Il arrive que la position actuelle et le point suivant soient
        % trop proches et engendre un changement de direction trop soudain
        % On retire donc un de ces points
        if length(timeValues) >= index_start + 1 && distance_column_vectors(points(:, index_start + 1), points(:, index_start)) < 1
            points = [points(:, 1:index_start), points(:, index_start + 2:length(points(1, :)))];
            timeValues = [timeValues(1:index_start), timeValues(index_start+2: length(timeValues))];
        end

        [xx, newtimeValues] = interpolate(timeValues, points(1, :), tStart, speed(1));
        [yy, ~] = interpolate(timeValues, points(2, :), tStart, speed(2));
        [zz, ~] = interpolate(timeValues, points(3, :), tStart, speed(3));
        [psi, ~] = interpolate(timeValues, points(4, :), tStart, speed(4));

        points = [xx;yy;zz;psi];
    
        traj = timeseries(points, newtimeValues);
    end
end

function value = distance_column_vectors(A, B)
    value = sqrt((A(1)-B(1))^2 + (A(2)-B(2))^2 + (A(3)-B(3))^2);
end