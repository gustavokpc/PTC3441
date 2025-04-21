%% Ex4:
% Realiza a multiplicação de matrizes de transformação homogênea de:
% A em relação a U;
% A em relação a B;
% U em relação a C.
% Para encontrar a matriz de transformação homogênea de C em relação a B.
%
%% I/O Variables
% |IN Double Matrix| *arelu*, *arelb*, *urelc*: Transformation Matrix 4x4
% 
% |OU Double Matrix| *crelb*: Transformation Matrix 4x4
%
%% Example
%  arelu = [11 -1 30];
%  arelb = [0 7 45];
%  urelc = [-3 -3 -30];
%  
%  urela = tinvert(utoi(arelu));
%  crelu = tinvert(utoi(urelc));

%  crela = tmult(urela,crelu);

%  crelb = tmult(utoi(arelb),crela);

%
%% Version Control
%
% 1.0; Grupo 3; 2025/04/19 ; First issue.
%  
%% Code

urela = tinvert(utoi(arelu));
crelu = tinvert(utoi(urelc));

crela = tmult(urela,crelu);

crelb = tmult(utoi(arelb),crela);
