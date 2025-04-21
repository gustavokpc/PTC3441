%% INVKIN
% Calcula a cinemática inversa para um manipulador RRR planar.
% 
%% Calling Syntax
% [near, far, sol] = invkin(wrelb, current, L, thetalim)
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% 
% |OU Double Matrix| *wrelb*: : Transformation Matrix 4x4
% 
%% Example
%  theta = [90 0 -90];
%  L = [0.3 0.5];
%  wrelb = kin(theta, L)
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
%% Function%   exemplo:
% wrelb = utoi([0.5 0.3 45]); current = [0.3 0.5 0]; L = [0.5 0.3]; thetalim = [-170 -170 -170; 170 170 170];
%   
% wrelb_t = itou(kin(near*180/pi, L))
% wrelb_t = itou(kin(far*180/pi, L))

function [near, far, sol] = invkin(wrelb, current, L, thetalim)
    
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
    sol = [];

    for s2 = s2_possibilidades
        theta2 = atan2(s2, c2);
        k1 = L(1) + L(2) * c2;
        k2 = L(2) * s2;
        gamma = atan2(k2, k1);
        theta1 = atan2(y, x) - gamma;
        theta3 = phi - theta1 - theta2;

        theta = [theta1; theta2; theta3];

        dentro_limites = all(theta >= thetalim(1,:)) & all(theta <= thetalim(2,:));
        if  dentro_limites
            sol = [sol, theta]; 
        end
    end

    if size(sol,2) == 1
        near = sol(:,1);
        far = [];
    elseif size(sol,2) == 2
        d1 = norm(sol(:,1) - current);
        d2 = norm(sol(:,2) - current);
        if d1 <= d2
            near = sol(:,1);
            far = sol(:,2);
        else
            near = sol(:,2);
            far = sol(:,1);
        end
    else
        near = [];
        far = [];
    end
end

