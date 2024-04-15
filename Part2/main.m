warning("off", "all")

xa = [15;15;15;0];
speed = [0;0;0;0];
pathBefore = [0;0;0;0];
path = [0;0;0;0];
v = 1;
w = 2*pi/180;
delta = .1;

tGlobal = time();

% Pour du test
tSpeed = timeseries(speed, 0);
tDistance = timeseries(0, 0);
tpos = timeseries(xa, 0);
tAngles = timeseries([0;0;0], 0);

% Pour la modélisation
ts = timeseries(xa(1:3), 0);
tGoal = timeseries([0;0;0], 0);

obstacles = readmatrix("../Passerelle1-2/dynamic_obstacles.csv");
while isempty(obstacles)
    obstacles = readmatrix("../Passerelle1-2/dynamic_obstacles.csv");
end
tObstacles = timeseries(transpose(obstacles), 0);

[traj, xorigin, tStart] = pathToTraj(xa, path, v, w, speed);
tsOut = sim("PIDF_avec_xy_pour_algo.slx", "StopTime", "10").tsOut;
tLastSim = time();
t = time();

while time() - tGlobal < 300
    % On récupère la suite de points par lesquel passer
    path = readmatrix("../Passerelle1-2/py_to_m.csv");
    while isempty(path)
        path = readmatrix("../Passerelle1-2/py_to_m.csv");
    end
    
    % On enregistre l'objectif pour la modélisation
    goal = readmatrix("../Passerelle1-2/goal.csv");
    while isempty(goal)
        goal = readmatrix("../Passerelle1-2/goal.csv");
    end
    tGoal = addsample(tGoal, "Data", transpose(goal), "Time", time() - tGlobal);
    
    % On enregistre la position des obstacles dynamiques pour la
    % modélisation
    obstacles = readmatrix("../Passerelle1-2/dynamic_obstacles.csv");
    while isempty(obstacles)
        obstacles = readmatrix("../Passerelle1-2/dynamic_obstacles.csv");
    end 
    tObstacles = addsample(tObstacles, "Data", transpose(obstacles), "Time", time() - tGlobal);

    % Distance entre la position du drone et la fin du trajet actuel
    distanceToEndPath = sqrt((path(1, length(path(1, :))) - xa(1))^2 + (path(2, length(path(1, :))) - xa(2))^2 + (path(3, length(path(1, :))) - xa(3))^2);
    
    %2 cas possibles pour lesquels on voudrait refaire une simu:
    %- Si le chemin a changé
    %- Si on arrive à la fin de la trajectoire prédite
    %De plus si on atteint l'objectif, on ne refait jamais la simu
    if ((~isequal(path, pathBefore)) | (time() - tLastSim > tsOut.Time(length(tsOut.Time)))) & ( distanceToEndPath > .01)
        disp("Simulation begining")

        % Le régulateur conserve des résultats cohérents autour de son
        % point d'équilibre, donc pour avoir des résultats corrects
        % partout, il faut décaler l'origine du repère en la position
        % actuelle du drone   
        [traj, xorigin, tStart] = pathToTraj(xa, path, v, w, speed);
        
        % On simule sur un petit temps pour que ce soit rapide
        simTime = 5;
        
        tsOut = sim("PIDF_avec_xy_pour_algo.slx", "StopTime", num2str(simTime)).tsOut;
        tLastSim = t - tStart;
        
        pathBefore = path;
        disp("Simulation done !")
        disp("")
    end

    actualTime = time() - tLastSim;
    
    % On récupère la position actuelle du drone
    if actualTime < tsOut.Time(length(tsOut.Time))
        posList = getsampleusingtime(tsOut, actualTime - delta, actualTime + delta);
        pos = posList.Data(:, :, fix(length(posList.Data(1, 1, :))/2));
    else
        pos = posList.Data(:, :, length(posList.Data(1, 1, :)));
    end

    t = time();
    
    % Vu qu'on a décalé l'origine, il faut réajuster avant d'envoyer
    % dans le csv
    xa = [pos(1:3);pos(6)] + xorigin;
    speed = pos(7:10);

    writematrix(pos(1:6) + [xorigin(1:3);0;0;xorigin(4)], "../Passerelle1-2/m_to_py.csv");

    at = time() - tGlobal;
    tpos = addsample(tpos, "Data", xa, "Time", at);
    tDistance = addsample(tDistance, "Data", distanceToEndPath, "Time", at);
    tSpeed = addsample(tSpeed, "Data", speed, "Time", at);
    tAngles = addsample(tAngles, "Data", pos(4:6), "Time", at);
    ts = addsample(ts, "Data", xa(1:3), "Time", at);
end


function t = time()
    t = posixtime(datetime);
end