%% TRAJECTORYGENERATIONRRR
% Gera trajetórias completas (posição, velocidade e aceleração) no espaço de juntas
% para um manipulador RRR planar com base em uma sequência de posições no espaço cartesiano.
%
%% Calling Syntax
% thpath = trajectorygenerationrrr(uform_vec, T, Ts)
%
%% I/O Variables
% |IN Double Matrix| *uform_vec*: Matriz com os pontos desejados no formato do usuário [nx3] (x, y, theta) [m, m, graus]  
% |IN Double| *T*: Tempo de duração de cada segmento [segundos]  
% |IN Double| *Ts*: Intervalo de amostragem da trajetória [segundos]  
%
% |OU Double Matrix| *thpath*: Matriz com as posições de junta no tempo [mx3]  
%
%% Example
%  uform_vec = [0.758 0.173 0; 0.6 -0.3 45; -0.4 0.3 120; 0.758 0.173 0];
%  T = 3;
%  Ts = 1/40;
%  thpath = trajectorygenerationrrr(uform_vec, T, Ts)
%
%% Hypothesis
% Manipulador planar com 3 graus de liberdade do tipo RRR.  
% A cinemática inversa e direta são válidas e bem definidas para os pontos dados.  
% Os limites articulares estão definidos e respeitados.
%
%% Limitations
% Não trata obstáculos, colisões ou validações adicionais além da cinemática.  
% Utiliza a solução mais próxima (near) da cinemática inversa; pode haver descontinuidades.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%
%% Function
function [thpath] = trajectorygenerationrrr(uform_vec, T, Ts)

%% Validity


%% Main Calculations

n = size(uform_vec, 1);     
thpath = [];                 

L = [0.5; 0.3];                 
thetalim = [-pi -pi -pi; pi pi pi]; 
trelw = utoi([0.1 0.2 30]);         
srelb = utoi([0.0 0.0 0.0]);        

joint_points = zeros(n, 3);         
current = [0; 0; 0];           

% Cálculo da trajetória no espaço de juntas
for i = 1:n
    goal = utoi(uform_vec(i,:));  
    [near, ~, sol] = solve_robot(goal, current, trelw, srelb, L, thetalim);
    
    if sol == 0
        error("Sem solução de cinemática inversa para o ponto %d", i);
    end
    
    joint_points(i,:) = near';
    current = near; 
end

thpath = zeros(0,3); 

% Geração de trajetórias para cada junta
for j = 1:3
    cctot = trajectoryplanning(joint_points(:,j)', T);
    thj = trajectorygeneration(joint_points(:,j)', T, Ts);

    if isempty(thpath)
        thpath = thj;
    else
        thpath(:,j) = thj(:,1); 
    end
end

%% Output Data
% Matriz com n linhas e 3 colunas, contendo as posições articulares ao longo do tempo

end