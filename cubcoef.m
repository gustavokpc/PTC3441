%% CUBCOEF
% Calcula os coeficientes de um polinômio cúbico para interpolação de trajetória
% angular entre dois pontos, considerando velocidades iniciais e finais.
%
%% Calling Syntax
% cc = cubcoef(th0, thdot0, thf, thdotf, T)
%
%% I/O Variables
% |IN Double| *th0*: Posição inicial [graus]  
% |IN Double| *thdot0*: Velocidade inicial [graus/segundo]  
% |IN Double| *thf*: Posição final [graus]  
% |IN Double| *thdotf*: Velocidade final [graus/segundo]  
% |IN Double| *T*: Tempo total da trajetória [segundos]  
%
% |OU Double Array| *cc*: Vetor coluna com os 4 coeficientes do polinômio cúbico [4x1]
%
%% Example
%  th0 = 30;
%  thdot0 = 0;
%  thf = 90;
%  thdotf = 0;
%  T = 2;
%  cc = cubcoef(th0, thdot0, thf, thdotf, T)
%
%% Hypothesis
% A interpolação ocorre de forma suave entre posições angulares com velocidades definidas nas extremidades.
%
%% Limitations
% A interpolação considera apenas um intervalo de tempo T e não trata aceleração inicial ou final específicas.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%     
%% Function
function [cc] = cubcoef(th0, thdot0, thf, thdotf, T)

%% Validity


%% Main Calculations
a0 = th0;
a1 = thdot0;
a2 = (3/(T^2))*(thf - th0) - (2/T)*thdot0 - (1/T)*thdotf;
a3 = (-2/(T^3))*(thf - th0) + (1/(T^2))*(thdotf + thdot0);

%% Output Data
cc = [a0; a1; a2; a3];

end