%% Resolved-Rate Control   
% Aplicação de uma simulação via Simulink para
% produzir os gráficos de: 
% ThetadotA vs Tempo
% ThetaA vs Tempo
% XA vs Tempo
% detJ vs Tempo 
% Tau vs Tempo
%
%% Calling Syntax
% ResolvedRateControl
%
%% I/O Variables
% L [meters]
% ThetaA
% ThetaA0
% XdotC0
% XdotC0
% W_0

%% Hypothesis
% RRR planar robot.
%
%% Limitations
% Essa simulação é baseada em simplificações
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/01 ; First issue.
%
%     
%% Function

L = [0.5 0.5 0.2];
ThetaA_0 = [-10 20 0]';
XdotC0 = [-0.10 0 0]';
W_0 = [1 2 3];

figure(1);
plot(Jac_det.time, Jac_det.data, 'LineWidth', 1.5);
title('Determinante do Jacobiano');
xlabel('Tempo (s)');
ylabel('|J|');

figure(2);

subplot(2,2,1);
plot(ThetadotA.time, ThetadotA.data, 'LineWidth', 1.2);
title('Velocidade articular');
xlabel('Tempo (s)');
ylabel('\theta (rad/s)');
legend('Joint 1','Joint 2','Joint 3');

subplot(2,2,2);
plot(ThetaA.time, ThetaA.data, 'LineWidth', 1.2);
title('Ângulo articular');
xlabel('Tempo (s)');
ylabel('\theta (rad)');
legend('Joint 1','Joint 2','Joint 3');

subplot(2,2,3);
plot(XA.time, XA.data, 'LineWidth', 1.2);
title('Coordenadas cartesianas');
xlabel('Tempo (s)');
ylabel('X (m), Y (m), \phi (rad)');
legend('X','Y','\phi');

subplot(2,2,4);
plot(Tau.time, Tau.data, 'LineWidth', 1.2);
title('Torque nas juntas');
xlabel('Tempo (s)');
ylabel('\tau (Nm)');
legend('\tau_1','\tau_2','\tau_3');