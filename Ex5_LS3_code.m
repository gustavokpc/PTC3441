%% Exercicio 5 
% Integração do controle PD com Feedforward ao controle de velocidade (Resolved-Rate Control).
%
%% Calling Syntax
% Executar o script diretamente:
% >> Ex5_LS3_code
%
%% I/O Variables
% Nenhuma entrada ou saída direta.
%
%% Example
% >> Ex5_LS3_code
%
%% Hypothesis
% Manipulador planar RRR com controle PD + feedforward em malha fechada de velocidade.
%
%% Limitations
% 
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%
%% Script


r = 1/100;
Beff = [50 50 50]; 
Jeff = [14 14 14]; 
K = 2750; 
N = 3;
freq = 100; % Hz
Ts = 1/freq;
wn = 2*pi*freq; 
Kp = [(Jeff(1)*wn*wn)/K (Jeff(2)*wn*wn)/K (Jeff(3)*wn*wn)/K];
Kd = [(2*wn*Jeff(1) - Beff(1))/K (2*wn*Jeff(2) - Beff(2))/K (2*wn*Jeff(3) - Beff(3))/K];
F = 5;
T = 3;
L = [0.5; 0.3];
m = [4.6; 2.3; 1.0];

sim(['Ex5_LS3.slx']);

%% Plots

figure;
subplot(2,2,1);
plot(Jac_det.time, Jac_det.data, 'LineWidth', 1.2);
title('Determinante Jacobiano');
xlabel('Tempo (s)');
ylabel('|J|');
grid on;

subplot(2,2,2);
plot(ThetadotA.time, ThetadotA.data, 'LineWidth', 1.2);
title('Velocidade Articular');
xlabel('Tempo (s)');
ylabel('\thetȧ (rad/s)');
legend('\thetȧ_1', '\thetȧ_2', '\thetȧ_3');
grid on;

subplot(2,2,3);
plot(ThetaA.time, ThetaA.data, 'LineWidth', 1.2);
title('Posição Articular');
xlabel('Tempo (s)');
ylabel('\theta (rad)');
legend('\theta_1', '\theta_2', '\theta_3');
grid on;

subplot(2,2,4);
plot(Tau.time, Tau.data, 'LineWidth', 1.2);
title('Torque de Controle');
xlabel('Tempo (s)');
ylabel('\tau (Nm)');
legend('\tau_1', '\tau_2', '\tau_3');
grid on;

