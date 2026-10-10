%% TRAJECTORYGENERATION
% Gera uma trajetória multissegmentada de posição, velocidade e aceleração
% para uma junta, com base nos coeficientes cúbicos de interpolação.
%
%% Calling Syntax
% thpath = trajectorygeneration(traj_points, T, Ts)
%
%% I/O Variables
% |IN Double Array| *traj_points*: Vetor com os pontos desejados de trajetória [1xn]
% |IN Double| *T*: Tempo de duração de cada segmento [segundos]
% |IN Double| *Ts*: Taxa de amostragem da trajetória [segundos]
%
% |OU Double Matrix| *thpath*: Matriz com [posição, velocidade, aceleração] por linha [mx3]
%
%% Example
%  traj_points = [30 45 90 60];
%  T = 2;
%  Ts = 0.025;
%  thpath = trajectorygeneration(traj_points, T, Ts)
%
%% Hypothesis
% Cada segmento é representado por um polinômio cúbico com duração fixa.
%
%% Limitations
% Apenas uma junta tratada por vez; não inclui sincronização entre múltiplas juntas.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%
%% Function
function [thpath] = trajectorygeneration(traj_points, T, Ts)

%% Validity


%% Main Calculations
cctot = trajectoryplanning(traj_points, T);
n_seg = length(traj_points) - 1;              
N = round(T / Ts);                            
thpath = [];                                  

for k = 1:n_seg
    coeffs = cctot(:, k);                   
    a0 = coeffs(1); a1 = coeffs(2);
    a2 = coeffs(3); a3 = coeffs(4);

    t_vec = 0:Ts:T-Ts;                        

    theta = a0 + a1*t_vec + a2*t_vec.^2 + a3*t_vec.^3;
    dtheta = a1 + 2*a2*t_vec + 3*a3*t_vec.^2;
    ddtheta = 2*a2 + 6*a3*t_vec;

    thpath = [thpath; [theta' dtheta' ddtheta']];
end

final_theta = traj_points(end);
thpath = [thpath; [final_theta 0 0]];

%% Output Data
% Matriz com linhas contendo posição, velocidade e aceleração para cada Ts
end