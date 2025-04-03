%% UTOI   
% Converte o formato de entrada de dados do usuário
% $(x,y,\theta)$
% para o formato interno (matriz de transformação homogênea)
% para um manipulador RRR planar.
% 
%% Calling Syntax
% iform = utoi(uform)
%
%% I/O Variables
% |IN Double Array| *uform*: _User form_  [x y theta] [meters meters degrees]
% 
% |OU Double Matrix| *iform*: _Internal form_  Homogeneous Transformation Matrix 4x4
%
%% Example
%  uform = [1 0 90]
%  iform = utoi(uform)
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
% 1.0; Grupo Gabriel, Gustavo, José; 2025/04/03 ; First issue.
%     
%% Function
function [ crela ] = tmult( brela,crelb )

%% Validity


%% Main Calculations
mul = brela*crelb;

%% Output Data

crela = mul;
end

