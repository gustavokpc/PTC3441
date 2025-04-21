%% INVKIN
% Calcula a cinemática inversa para um manipulador RRR planar.
% 
%% Calling Syntax
% [near, far, sol] = invkin(wrelb, current, L, thetalim)
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% |IN Double Array| *wrelb*, *current*: User form  [x y theta] [metros metros graus]
% |IN Double Matrix| *thetalim*: Matrix de valores limites de thetas 2xn
% (n = número de juntas)
% |OU Double Array| *near*, *far*: : User form  [x y theta] [metros metros graus]
% |OUT Flag| *sol*: 1 if there are valid solutions, 0 otherwise
% 
%% Example
% wrelb = utoi([0.5 0.3 45]);
% current = [0.3 0.5 0];
% L = [0.5 0.3];
% thetalim = [-170 -170 -170; 170 170 170];
%
% [near, far, sol] = invkin(wrelb, current, L, thetalim)
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
function [near, far, sol] = invkin(wrelb, current, L, thetalim)

%% Validity


%% Main Calculations 
    x = wrelb(1,4);
    y = wrelb(2,4);
    R = wrelb(1:2,1:2); 
    phi = atan2(R(2,1), R(1,1)); 

    c2 = (x^2 + y^2 - L(1)^2 - L(2)^2) / (2 * L(1) *L(2));

    if abs(c2) > 1
        near = [];
        far =  [] ;
        sol =  [] ;
        return;
    end

    s2_possibilidades = [+sqrt(1 - c2^2), -sqrt(1 - c2^2)];
     valid_solutions = [];

    for s2 = s2_possibilidades
        theta2 = atan2(s2, c2);
        k1 = L(1) + L(2) * c2;
        k2 = L(2) * s2;
        gamma = atan2(k2, k1);
        theta1 = atan2(y, x) - gamma;
        theta3 = phi - theta1 - theta2;

        theta = [theta1; theta2; theta3];

        dentro_limites = all(theta >= thetalim(1,:)) & all(theta <= thetalim(2,:));
        if dentro_limites
            valid_solutions = [valid_solutions, theta]; 
        end
    end

    if isempty(valid_solutions)
        near = [];
        far = [];
        sol = 0;
    elseif size(valid_solutions,2) == 1
        near = valid_solutions(:,1);
        far = [];
        sol = 1;
    else
        d1 = norm(valid_solutions(:,1) - current);
        d2 = norm(valid_solutions(:,2) - current);
        if d1 <= d2
            near = valid_solutions(:,1);
            far = valid_solutions(:,2);
        else
            near = valid_solutions(:,2);
            far = valid_solutions(:,1);
        end
        sol = 1;
    end
end

