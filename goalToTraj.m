function [p_traj, t_traj, coef_traj] = goalToTraj(xa, xgoal, v, w)
    
    tEnd = sqrt((xa(1, 1)-xgoal(1, 1))^2 + (xa(2, 1)-xgoal(2, 1))^2 + (xa(3, 1)-xgoal(3, 1))^2)/v;
    
    tEndangle = abs(xa(4, 1) - xgoal(4, 1))/w;
    
    p_traj = [xa(1, 1), xgoal(1, 1), xa(2, 1), xgoal(2, 1), xa(3, 1), xgoal(3, 1), xa(4, 1), xgoal(4, 1)];
    t_traj = [0, tEnd, 0, tEnd, 0, tEnd, tEnd, tEnd + tEndangle];
    coef_traj = zeros(length(t_traj)/2, 6);
   
    for i = 1:length(t_traj)/2
        if (i < 4 && tEnd ~= 0) || (i == 4 && tEndangle ~= 0)
            coef_traj(i, :) = interp_poly5(p_traj(2*i-1), p_traj(2*i), t_traj(2*i-1), t_traj(2*i));
        end
    end
end