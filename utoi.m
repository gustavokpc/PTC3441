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
% 1.0; Fábio Fialho; 2014/09/17 ; First issue.
%
% 1.1; Fábio Fialho; 2019/09/12 ; Minor modifications.
%     
%% Function
function iform = utoi(uform)

%% Validity


%% Main Calculations

    T = [cosd(uform(3)) -sind(uform(3)) 0 uform(1);
         sind(uform(3)) cosd(uform(3))  0 uform(2);
               0              0         1    0    ;
               0              0         0    1   ];

%% Output Data

    iform = T;
    
end