warning("off", "all")

xa = [0;0;0;0];
xgoalBefore = [0;0;0;0];
xgoal = [0;0;0;0];
xorigin = xa;
v = 1;
w = pi/180;
delta = .5;

tGlobal = time();

[p_traj, t_traj, coef_traj] = goalToTraj(xa, xgoal, v, w);
tsOut = sim("PIDF_avec_xy_pour_algo.slx").tsOut;
tLastSim = time();

while time() - tGlobal < 120
    xgoal = readmatrix("py_to_m.csv");
    
    while isempty(xgoal)
        xgoal = readmatrix("py_to_m.csv");
    end

    if ((~isequal(xgoal, xgoalBefore)) | (time() - tLastSim > 20)) & (sqrt((xgoal(1) - xa(1))^2 + (xgoal(2) - xa(2))^2 + (xgoal(3) - xa(3))^2) > .5)
        disp("Simulation begining")
        xgoal
        xorigin = xa;
        [p_traj, t_traj, coef_traj] = goalToTraj([0;0;0;0], xgoal - xorigin, v, w);
        
        
        tsOut = sim("PIDF_avec_xy_pour_algo.slx").tsOut;
        tLastSim = time();
        
        xgoalBefore = xgoal;
        disp("Simulation done !")
        disp("")
    end
    actualTime = time() - tLastSim;
    
    if actualTime < 10
        posList = getsampleusingtime(tsOut, actualTime - delta, actualTime + delta);
        pos = posList.Data(:, :, fix(length(posList.Data(1, 1, :))/2));
        xa = [pos(1);pos(2);pos(3);pos(6)] + xorigin;
        writematrix(pos + [xorigin(1); xorigin(2); xorigin(3); 0; 0; xorigin(4)], "m_to_py.csv");
    end
end


function t = time()
    t = posixtime(datetime);
end