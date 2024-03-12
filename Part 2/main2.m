warning("off", "all")

xa = [0;0;0;0];
pathBefore = [0;0;0;0];
path = [0;0;0;0];
xorigin = xa;
v = 1;
w = pi/180;
delta = .5;

tGlobal = time();

traj = pathToTraj(xa, path, v, w);
tsOut = sim("PIDF_avec_xy_pour_algo2.slx").tsOut;
tLastSim = time();

while time() - tGlobal < 300
    path = readmatrix("py_to_m.csv");
    
    while isempty(path)
        path = readmatrix("py_to_m.csv");
    end
    distanceToEndPath = sqrt((path(1, length(path(1, :))) - xa(1))^2 + (path(2, length(path(1, :))) - xa(2))^2 + (path(3, length(path(1, :))) - xa(3))^2);
    
    %2 cas possibles pour lesquels on voudrait refaire une simu:
    %- Si le chemin a changé (sauf si c'est parce qu'on avance dessus)
    %- Si on arrive à la fin de la trajectoire prédite (ça arrive rarement)
    %De plus si on attend l'objectif, on ne refait jamais la simu
    if ((~isIn(path, pathBefore)) | (time() - tLastSim > tsOut.Time(length(tsOut.Time)))) & ( distanceToEndPath > .5)
        disp("Simulation begining")
        path
        
        % Le régulateur conserve des résultats cohérents autour de son
        % point d'équilibre, donc pour avoir des résultats corrects
        % partout, il faut décaler l'origine du repère en la position
        % actuelle du drone
        xorigin = xa;
        pathDisplacement = zeros(4, length(path(1, :)));

        for i = 1:length(path(1, :))
            pathDisplacement(:, i) = xorigin;
        end

        traj = pathToTraj([0;0;0;0], path - pathDisplacement, v, w);
        
        tsOut = sim("PIDF_avec_xy_pour_algo2.slx").tsOut;
        tLastSim = time();
        
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
        writematrix(pos + [xorigin(1:3);0;0;xorigin(4)], "m_to_py.csv");
    end
end


function t = time()
    t = posixtime(datetime);
end