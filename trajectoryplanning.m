%% TRAJECTORYPLANNING
% Planeja a trajetória de uma junta com base em pontos desejados e 
% tempo fixo para cada segmento, utilizando interpolação cúbica.
%
%% Calling Syntax
% cctot = trajectoryplanning(traj_points, T)
%
%% I/O Variables
% |IN Double Array| *traj_points*: Vetor com os pontos desejados de trajetória [1xn]
% |IN Double| *T*: Tempo de duração de cada segmento [segundos]
%
% |OU Double Matrix| *cctot*: Matriz com os coeficientes dos polinômios cúbicos [4xn-1]
%
%% Example
%  traj_points = [30 45 90 60];
%  T = 2;
%  cctot = trajectoryplanning(traj_points, T)
%
%% Hypothesis
% Movimento de uma única junta, com velocidades nulas nos extremos e 
% heurística para pontos intermediários baseada na continuidade de trajetória.
%
%% Limitations
% Apenas velocidades nulas nas extremidades e abordagem heurística simples para os via-points.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%
%% Function
function [cctot] = trajectoryplanning(traj_points, T)

%% Validity


%% Main Calculations
n = length(traj_points);        
cctot = [];                     
thdots = zeros(1, n);            

for k = 2:n-1
    delta1 = traj_points(k) - traj_points(k-1);
    delta2 = traj_points(k+1) - traj_points(k);
    
    if delta1 * delta2 < 0
        thdots(k) = 0;
    else
        thdots(k) = (delta1 + delta2) / (2 * T);
    end
end

thdots(1) = 0;
thdots(end) = 0;

for k = 1:n-1
    th0 = traj_points(k);
    thf = traj_points(k+1);
    thdot0 = thdots(k);
    thdotf = thdots(k+1);

    cc = cubcoef(th0, thdot0, thf, thdotf, T);
    cctot = [cctot cc];
end

%% Output Data
% Matriz com os coeficientes por coluna: [a0; a1; a2; a3] para cada segmento
end