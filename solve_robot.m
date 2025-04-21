%% Example
% goal = utoi([0.5 0.3 -45]); current = [0.3 0.5 0]; trelw = utoi([0 0 -45]); srelb = utoi([0 0 0]); L = [0.5 0.3]; thetalim = [-170 -170 -170; 170 170 170];
% [near,far,sol]=solve_robot(goal,current,trelw,srelb,L,thetalim)
%% Example validity test
% trels = itou(where_robot(near*180/pi,trelw,srelb,L))


function [near, far, sol] = solve_robot(goal, current, trelw, srelb, L, thetalim)

    temp1 = tinvert(trelw);  

    temp2 = tmult(goal, temp1);  

    temp3 = tmult(srelb, temp2);

    [near, far, sol] = invkin(temp3, current, L, thetalim);

end

