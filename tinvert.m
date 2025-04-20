%% TINVERT   
% Calcula a matriz inversa da matriz de transformação homogênea
% 
%% Calling Syntax
% arelb = tinvert(brela)
%   
%% I/O Variables
% |IN Double Matrix| *brela*: Transformation Matrix 4x4
% 
% |OU Double Matrix| *arelb*: Transformation Matrix 4x4
%
%% Example
%  brela = [1 0 0 0.3;
%           0 1 0 0.5;
%           0 0 1 0  ;
%           0 0 0 1 ];
%
%  arelb = tinvert(brela);
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
function [ arelb ] = tinvert( brela )

%% Validity


%% Main Calculations
R_brela = brela(1:3, 1:3);
O_brela = brela(1:3, 4);
zeros = [0 0 0];

M(1:3, 1:3) = transpose(R_brela);
M(1:3,4) = tmult(-transpose(R_brela), O_brela);
M(4,1:3) = zeros;
M(4,4) = 1;

%% Output Data
arelb = M;

end
