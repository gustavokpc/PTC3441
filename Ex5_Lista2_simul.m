%% Exercicio 5
% Calcula os torques simbólicos para um manipulador planar RRR com três elos e três juntas rotacionais.
%
%% Calling Syntax
% Tau_val = tau_symbolic(theta1, theta2, theta3, dtheta1, dtheta2, dtheta3, ddtheta1, ddtheta2, ddtheta3, m1, m2, m3, L1, L2, g)
%
%% I/O Variables
% |IN Double| *theta1*, *theta2*, *theta3*: Ângulos das juntas [rad]
% |IN Double| *dtheta1*, *dtheta2*, *dtheta3*: Velocidades angulares [rad/s]
% |IN Double| *ddtheta1*, *ddtheta2*, *ddtheta3*: Acelerações angulares [rad/s²]
% |IN Double| *m1*, *m2*, *m3*: Massas dos elos [kg]
% |IN Double| *L1*, *L2*: Comprimentos dos elos [m]
% |IN Double| *g*: Gravidade [m/s²]
%
% |OU Double Array| *Tau_val*: Vetor 3x1 com torques resultantes nas juntas [Nm]
%
%% Example
%  theta1 = pi/4; theta2 = pi/6; theta3 = pi/3;
%  dtheta1 = 0.1; dtheta2 = 0.1; dtheta3 = 0.1;
%  ddtheta1 = 0.2; ddtheta2 = 0.2; ddtheta3 = 0.2;
%  m1 = 4.6; m2 = 2.3; m3 = 1.0;
%  L1 = 0.5; L2 = 0.3; g = 9.8;
%  Tau_val = tau_symbolic(theta1, theta2, theta3, dtheta1, dtheta2, dtheta3, ddtheta1, ddtheta2, ddtheta3, m1, m2, m3, L1, L2, g)
%
%% Hypothesis
% Manipulador planar RRR com juntas rotacionais, conforme descrição do Exercício 1.
%
%% Limitations
% Função baseada em simplificações planas e em expressões simbólicas. Pode não generalizar para modelos espaciais ou robôs com dinâmica complexa.
%
%% Version Control
%
% 1.0; Grupo 3; 2025/06/30 ; First issue.
%     
%% Function
function Tau_val = tau_symbolic(theta1, theta2, theta3, ...
                                 dtheta1, dtheta2, dtheta3, ...
                                 ddtheta1, ddtheta2, ddtheta3, ...
                                 m1, m2, m3, L1, L2, g)
%% Validity


%% Main Calculations
    % Variáveis simbólicas
    syms th1 th2 th3 dth1 dth2 dth3 ddth1 ddth2 ddth3 real
    syms mm1 mm2 mm3 LL1 LL2 gg real

    % Vetor z_hat
    z_hat = [0; 0; 1];

    % Matrizes de rotação
    R32 = [cos(th3),  sin(th3), 0;
          -sin(th3), cos(th3), 0;
           0,       0,        1];

    R23 = [cos(th3), -sin(th3), 0;
           sin(th3),  cos(th3), 0;
           0,         0,        1];

    R12 = [cos(th2), -sin(th2), 0;
           sin(th2),  cos(th2), 0;
           0,         0,        1];

    R01 = [cos(th1), -sin(th1), 0;
           sin(th1),  cos(th1), 0;
           0,         0,        1];

    % Velocidade e aceleração angular do elo 2
    w22 = [0; 0; dth1 + dth2];
    w22dot = [0; 0; ddth1 + ddth2];

    % Aceleração linear do centro de massa do elo 2
    v22dot = [ LL1*ddth1*sin(th2) - LL1*(dth1^2)*cos(th2) + gg*sin(th1+th2);
               LL1*ddth1*cos(th2) + LL1*(dth1^2)*sin(th2) + gg*cos(th1+th2);
               0 ];

    % Força no elo 2
    F22 = [ mm2*LL1*ddth1^2*sin(th2) - mm2*LL1*dth1^2*cos(th2) + mm2*gg*sin(th1+th2) - mm2*LL2*(dth1 + dth2)^2;
            mm2*LL1*ddth1^2*cos(th2) - mm2*LL1*dth1^2*sin(th2) + mm2*gg*cos(th1+th2) - mm2*LL2*(dth1 + dth2);
            0 ];

    % Força no elo 1
    F11 = [ -mm1*LL1*dth1^2 + mm1*gg*sin(th1);
             mm1*LL1*ddth1 + mm1*gg*cos(th1);
             0 ];

    % Vetores de posição e inércia
    OC33 = [0; 0; 0];
    OC22 = [LL2; 0; 0];
    OC11 = [LL1; 0; 0];
    O34 = [0; 0; 0];
    O23 = [LL2; 0; 0];
    O12 = [LL1; 0; 0];

    IC3 = [0.05, 0, 0;
           0, 0.1, 0;
           0, 0, 0.1];

    % Inicializações
    finho44 = [0; 0; 0];
    nzinho44 = [0; 0; 0];
    N22 = [0; 0; 0];
    N11 = [0; 0; 0];

    % Dinâmica do elo 3
    w33 = R32*w22 + dth3*z_hat;
    w33dot = R32*w22dot + ddth3*z_hat + cross(R32*w22, ddth3*z_hat);
    N33 = IC3*w33dot + cross(w33, IC3*w33dot);
    v33dot = R32*(cross(w22dot, O23) + cross(w22, cross(w22, O23)) + v22dot);
    vc33dot = cross(w33dot, OC33) + cross(w33, cross(w33, OC33)) + v33dot;
    F33 = mm3 * vc33dot;

    % Propagação das forças/torques
    finho33 = R23*finho44 + F33;
    nzinho33 = N33 + cross(OC33, F33) + cross(O34, R23*finho44);

    finho22 = R12*finho33 + F22;
    nzinho22 = N22 + cross(OC22, F22) + cross(O23, R12*finho33);

    finho11 = R01*finho22 + F11;
    nzinho11 = N11 + cross(OC11, F11) + cross(O12, R01*finho22);

    % Torques finais nas juntas
    tau3 = simplify(transpose(nzinho33)*z_hat);
    tau2 = simplify(transpose(nzinho22)*z_hat);
    tau1 = simplify(transpose(nzinho11)*z_hat);

    Tau_sym = [tau1; tau2; tau3];

    % Substituições e avaliação numérica
    syms_list = {th1, th2, th3, dth1, dth2, dth3, ddth1, ddth2, ddth3, ...
                 mm1, mm2, mm3, LL1, LL2, gg};

    values = {theta1, theta2, theta3, ...
              dtheta1, dtheta2, dtheta3, ...
              ddtheta1, ddtheta2, ddtheta3, ...
              m1, m2, m3, L1, L2, g};

    Tau_val = double(subs(Tau_sym, syms_list, values));
end
