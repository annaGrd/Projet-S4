xa = [0;0;0;0];
xgoalBefore = [0;0;0;0];
xgoal = [0;0;0;0];
v = 1;
w = pi/180;
delta = .5;

tGlobal = time();

[p_traj, t_traj, coef_traj] = goalToTraj(xa, xgoal, v, w);
tsOut = sim("PIDF_avec_xy_pour_algo.slx").tsOut;
tLastSim = time();

while time() - tGlobal < 300
    xgoal = readmatrix("py_to_m.csv");
    
    while isempty(xgoal)
        xgoal = readmatrix("py_to_m.csv");
    end

    if ((~isequal(xgoal, xgoalBefore)) | (time() - tLastSim > 20)) & (~isequal(xgoal, xa))
        [p_traj, t_traj, coef_traj] = goalToTraj(xa, xgoal, v, w);
        tsOut = sim("PIDF_avec_xy_pour_algo.slx").tsOut;
        tLastSim = time();

        if ~isequal(xa, [0;0;0;0])
            plot(tsOut)
        end

        xgoalBefore = xgoal;
    end
    actualTime = time() - tLastSim;
    posList = getsampleusingtime(tsOut, actualTime - delta, actualTime + delta);
    pos = posList.Data(:, :, fix(length(posList.Data(1, 1, :))/2));
    xa = pos;
    xa
    writematrix(pos, "m_to_py.csv");
end


function t = time()
    t = posixtime(datetime);
end