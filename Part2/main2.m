warning("off", "all")

xa = [0;0;0;0];
speed = [0;0;0;0];
pathBefore = [0;0;0;0];
path = [0;0;0;0];
pathTraveled = [0;0;0;0];
v = 1;
w = pi/180;
delta = .1;

tGlobal = time();

tSpeed = timeseries([0;0;0], 0);
tDistance = timeseries(0, 0);
tpos = timeseries([0;0;0;0], 0);
tAngles = timeseries([0;0;0], 0);

[traj, xorigin, tStart] = pathToTraj(xa, path, v, w, speed);
tsOut = sim("PIDF_avec_xy_pour_algo2.slx", "StopTime", "10").tsOut;
tLastSim = time();
t = time();

while time() - tGlobal < 300
    path = readmatrix("../Passerelle1-2/py_to_m.csv");
    
    while isempty(path)
        path = readmatrix("../Passerelle1-2/py_to_m.csv");
    end

    distanceToEndPath = sqrt((path(1, length(path(1, :))) - xa(1))^2 + (path(2, length(path(1, :))) - xa(2))^2 + (path(3, length(path(1, :))) - xa(3))^2);
    
    %2 cas possibles pour lesquels on voudrait refaire une simu:
    %- Si le chemin a changé (sauf si c'est parce qu'on avance dessus)
    %- Si on arrive à la fin de la trajectoire prédite (ça arrive rarement)
    %De plus si on atteint l'objectif, on ne refait jamais la simu
    if ((~isIn(path, pathBefore)) | (time() - tLastSim > tsOut.Time(length(tsOut.Time)))) & ( distanceToEndPath > .5)
        disp("Simulation begining")
        %if ~isequal(pathTraveled(:, length(pathTraveled(1, :))), xa)
        %    pathTraveled = cat(2, pathTraveled, xa + xorigin);
        %end

        % Le régulateur conserve des résultats cohérents autour de son
        % point d'équilibre, donc pour avoir des résultats corrects
        % partout, il faut décaler l'origine du repère en la position
        % actuelle du drone
           
        grad = gradient(squeeze(traj.Data));
        tsSpeed = timeseries(grad, traj.Time);
        
        actualTime = time() - tLastSim;
        t = time();
        speedList = getsampleusingtime(tsSpeed, actualTime - delta, actualTime + delta);
        if isequal(speedList.Data, [])
            speed = [0;0;0;0];
        elseif length(speedList.Data(1, 1, :)) == 1
            speed = speedList.Data(:, 1, 1);
        elseif actualTime < tsOut.Time(length(tsOut.Time))
            speed = speedList.Data(:, :, fix(length(speedList.Data(1, 1, :))/2));
        else
            speed = speedList.Data(:, :, length(speedList.Data(1, 1, :)));
        end     
         
        [traj, xorigin, tStart] = pathToTraj(xa, path, v, w, [realSpeed; speed(4)]);
        
        simTime = min(10, traj.Time(length(traj.Time)));

        tsOut = sim("PIDF_avec_xy_pour_algo2.slx", "StopTime", num2str(simTime)).tsOut;
        tLastSim = t - tStart;
        
        pathBefore = path;
        disp("Simulation done !")
        disp("")
    end

    actualTime = time() - tLastSim;
    
    if actualTime < tsOut.Time(length(tsOut.Time))
        posList = getsampleusingtime(tsOut, actualTime - delta, actualTime + delta);
        pos = posList.Data(:, :, fix(length(posList.Data(1, 1, :))/2));
    else
        pos = posList.Data(:, :, length(posList.Data(1, 1, :)));
    end

    % Vu qu'on a décalé l'origine, il faut réajuster avant d'envoyer
    % dans le csv
    xa = [pos(1:3);pos(6)] + xorigin;
    realSpeed = pos(7:9);
    
    %if ~isequal(pathTraveled(:, length(pathTraveled(1, :))), path(:, 2)) && sqrt((xa(1) - path(1, 2))^2 + (xa(2) - path(2, 2))^2 + (xa(3) - path(3, 2))^2) < .5
    %    pathTraveled = cat(2, pathTraveled, path(:, 2));
    %end

    writematrix(pos(1:6) + [xorigin(1:3);0;0;xorigin(4)], "../Passerelle1-2/m_to_py.csv");

    at = time() - tGlobal;
    tpos = addsample(tpos, "Data", xa, "Time", at);
    tDistance = addsample(tDistance, "Data", distanceToEndPath, "Time", at);
    tSpeed = addsample(tSpeed, "Data", realSpeed, "Time", at);
    tAngles = addsample(tAngles, "Data", pos(4:6), "Time", at);
end


function t = time()
    t = posixtime(datetime);
end