%% ITOU   
% Converte o formato de entrada de dados do formato interno (matriz de
% transformação homogênea)
% para o formato do usuário
% $(x,y,\theta)$
% para um manipulador RRR planar.
% 
%% Calling Syntax
% uform = itou(iform)
%
%% I/O Variables
% |IN Double Matrix| *iform*: _Internal form_  Homogeneous Transformation Matrix 4x4
% 
% |OU Double Array| *uform*: _User form_  [x y theta] [meters meters degrees]
% 
%% Example
%  iform = [0 -1 0 1; 1 0 0 0; 0 0 1 0; 0 0 0 1];
%  uform = itou(iform)
%
%% Hypothesis
% RRR planar robot.
%
%% Limitations
% A "Forma do usuário" é específica para o exercício de simulação e não tem
% validade para qualquer configuração de robô.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/04/19 ; First issue.
%     
%% Function
function uform = itou(iform)

%% Validity


%% Main Calculations

    T = [iform(1,4), iform(2,4), acosd(iform(1,1))];

%% Output Data

    uform = T;
    
end
