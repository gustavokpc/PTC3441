%% SOLVE_ROBOT
% Calcula a matriz de transformação homogênea do punho em relação a base
% dados o o sistema da ferramenta com relação ao sistema do punho e o
% sistema da estação com relação à base do robô
% 
%% Calling Syntax
% [near, far, sol] = solve_robot(goal, current, trelw, srelb, L, thetalim)
%
%% I/O Variables
% |IN Double Matrix| *trelw*, *srelb*: : Transformation Matrix 4x4
% |IN Double Array| *goal*, *current*: User form  [x y theta] [metros metros graus]
% |IN Double Matrix| *thetalim*: Matrix de valores limites de thetas 2xn
% (n = número de juntas)
% |IN Double Array| *L*: Array de valores de L (em metros, [1x2])
% |OU Double Array| *near*, *far*: : User form  [x y theta] [metros metros graus]
% |OUT Flag| *sol*: 1 if there are valid solutions, 0 otherwise
% 
%% Example
% goal = utoi([0.5 0.3 -45]);
% current = [0.3 0.5 0];
% trelw = utoi([0 0 -45]);
% srelb = utoi([0 0 0]);
% L = [0.5 0.3];
% thetalim = [-170 -170 -170; 170 170 170];
%
% [near, far, sol] = solve_robot(goal, current, trelw, srelb, L, thetalim)
%
%% Hypothesis
% RRR planar robot.
%
%% Limitations
%
%% Version Control
%
% 1.0; Grupo 3; 2025/04/19 ; First issue.
%
%% Function
function [near, far, sol] = solve_robot(goal, current, trelw, srelb, L, thetalim)

%% Validity


%% Main Calculations
    temp1 = tinvert(trelw);

    temp2 = tmult(goal, temp1);

    temp3 = tmult(srelb, temp2);

%% Output Data
    [near, far, sol] = invkin(temp3, current, L, thetalim);

end