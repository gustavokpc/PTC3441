%% WHERE_ROBOT
% Escreva uma rotina que calcule onde a ferramenta está com
% relação ao sistema da estação para um manipulador planar RRR.
% 
%% Calling Syntax
% trels = where_robot(theta, trelw, srelb, L)
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% |IN Double Matrix| *trelw*, *srelb*: Transformation Matrix 4x4
% 
% |OU Double Matrix| *trels*: : Transformation Matrix 4x4
% 
%% Example
%  theta = [90 0 -90];
%  L = [0.3 0.5];
%  trelw = [0.866 -0.5 0 0.1; 0.5 0.866 0 0.2; 0 0 1 0; 0 0 0 1];
%  srelb = [1 0 0 -0.1; 0 1 0 0.3; 0 0 1 0; 0 0 0 1];
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
%% Function
function [ trels ] = where_robot( theta, trelw, srelb, L )

%% Validity


%% Main Calculations
brels = tinvert(srelb);
wrelb = kin(theta, L);

trels_t = tmult(brels, wrelb);
M = tmult(trels_t, trelw);

%% Output Data
trels = M;

end
