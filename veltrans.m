%% VELTRANS
% Escreva uma rotina que dados aTb e aVa
% Manipulador planar RRR.
% 
%% Calling Syntax
% nirelb = veltrans(nirela,brela)
%
%% I/O Variables
% |IN Double Array| *nirela*: Array de velocidade linear e angular [6x1]
% |IN Double Matrix| *brela*: Transformation Matrix [4x4]
% |OU Double Array| *nirelb*: Array de velocidade linear e angular [6x1]
% 
%% Example
%  nirela = [0.5 0 0 2 0 0];
%  brela = [0.866 -0.5 0 0.1; 0.5 0.866 0 0.2; 0 0 1 0; 0 0 0 1];
%
%% Hypothesis
% RRR planar robot.
%
%% Limitations
%
%% Version Control
%
% 1.0; Grupo 3; 2025/05/26 ; First issue.
%     
%% Function
function [nirelb] = veltrans(nirela,brela)

%% Validity


%% Main Calculations
R = brela(1:3, 1:3);      
Pos = brela(1:3, 4);  

S = [ 0,   -Pos(3),    Pos(2);
      Pos(3),    0,   -Pos(1);
     -Pos(2),  Pos(1),    0 ];

T = [R,        S*R;
    zeros(3),    R ];


%% Output Data

 nirelb = T * nirela;

end




