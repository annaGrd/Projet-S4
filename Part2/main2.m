warning("off", "all")

xa = [0;0;0;0];
speed = [0;0;0];
pathBefore = [0;0;0;0];
path = [0;0;0;0];
pathTraveled = [0;0;0;0];
xorigin = xa;
v = 1;
w = pi/180;
delta = .5;

tGlobal = time();

[traj, tStart] = pathToTraj(xa, path, v, w, pathTraveled, speed);
tsOut = sim("PIDF_avec_xy_pour_algo2.slx").tsOut;
tLastSim = time();

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
        t = time();
        %if ~isequal(pathTraveled(:, length(pathTraveled(1, :))), xa)
        %    pathTraveled = cat(2, pathTraveled, xa + xorigin);
        %end

        % Le régulateur conserve des résultats cohérents autour de son
        % point d'équilibre, donc pour avoir des résultats corrects
        % partout, il faut décaler l'origine du repère en la position
        % actuelle du drone
        
        if isequal(speed, [0;0;0])
            pathTraveled = xa;
        else
            pathTraveled = [xa - [5*speed/sqrt(speed(1)^2 + speed(2)^2 + speed(3)^2);0],xa];
        end
        
        xorigin = pathTraveled(:, 1);
        pathDisplacement = zeros(4, length(path(1, :)));
        pathTraveledDisplacement = zeros(4, length(pathTraveled(1, :)));

        for i = 1:length(path(1, :))
            pathDisplacement(:, i) = xorigin;
        end

        for i = 1:length(pathTraveled(1, :))
            pathTraveledDisplacement(:, i) = xorigin;
        end

        [traj, tStart] = pathToTraj(xa - xorigin, path - pathDisplacement, v, w, pathTraveled - pathTraveledDisplacement, speed);
        

        tsOut = sim("PIDF_avec_xy_pour_algo2.slx", "StopTime", num2str(traj.Time(length(traj.Time)))).tsOut;
        tLastSim = t - tStart;
        
        pathBefore = path;
        disp("Simulation done !")
        disp("")
    end

    actualTime = time() - tLastSim;
    
    if distanceToEndPath > .5
        posList = getsampleusingtime(tsOut, actualTime - delta, actualTime + delta);
        pos = posList.Data(:, :, fix(length(posList.Data(1, 1, :))/2));
        % Vu qu'on a décalé l'origine, il faut réajuster avant d'envoyer
        % dans le csv
        xa = [pos(1:3);pos(6)] + xorigin;
        speed = pos(4:6);
        
        %if ~isequal(pathTraveled(:, length(pathTraveled(1, :))), path(:, 2)) && sqrt((xa(1) - path(1, 2))^2 + (xa(2) - path(2, 2))^2 + (xa(3) - path(3, 2))^2) < .5
        %    pathTraveled = cat(2, pathTraveled, path(:, 2));
        %end

        writematrix(pos(1:6) + [xorigin(1:3);0;0;xorigin(4)], "../Passerelle1-2/m_to_py.csv");
    else
        speed = [0;0;0];
    end

end


function t = time()
    t = posixtime(datetime);
end