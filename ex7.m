%% Ex7:
% Calcule a posição e a orientação da ferramenta com relação ao sistema
% da estação para três diferentes configurações
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% |IN Double Array| *brela*, *crelb*: Transformation Matrix in user form [x y theta]
% 
% |OU Double Matrix| *trels*: Transformation Matrix 4x4
%
%% Example
% L = [0.5 0.3];
%
% trelw = utoi([0.1 0.2 30.0])
% srelb = utoi([-0.1 0.3 0.0])
%
% theta_1 = [0.0 90.0 -90.0];
% theta_2 = [-23.6 -30.3 48.0];
% theta_3 = [130.0 40.0 12.0];
%
% trels_1 = where_robot(theta_1, trelw, srelb, L)
% trels_2 = where_robot(theta_2, trelw, srelb, L)
% trels_3 = where_robot(theta_3, trelw, srelb, L)
%
%% Version Control
%
% 1.0; Grupo 3; 2025/04/19 ; First issue.
%  
%% Code

L = [0.5 0.3];

trelw = utoi([0.1 0.2 30.0])
srelb = utoi([-0.1 0.3 0.0])

theta_1 = [0.0 90.0 -90.0];
theta_2 = [-23.6 -30.3 48.0];
theta_3 = [130.0 40.0 12.0];

trels_1 = where_robot(theta_1, trelw, srelb, L)
trels_2 = where_robot(theta_2, trelw, srelb, L)
trels_3 = where_robot(theta_3, trelw, srelb, L)
