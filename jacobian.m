%% JACOBIAN
% Determine o Jacobiano (3x3)
%
% Para isso, usamos o exemplo 5.3 como base.
% Primeiramente, mudamos a matriz T23, pois agora ela não será simplesmente
% uma matriz identidade. Em seguida, calculamos v3. Entretanto,
% diferentemente do exemplo 5.3, não usamos a matriz identidade, agora 
% usamos a matriz R32: [c3 s3 0; -s3 c3 0; 0 0 1]; para multiplicar a 
% matriz: [s2*L1*thetadot1; c2*L1*thetadot1 + L2*(thetadot1+thetadot2); 0];
 
%% Calling Syntax
% jac = jacobian(theta,L)
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% |OU Double Matrix| *jac*: matrix jacobiana [3x3]
% 
%% Example
%  theta = [90 0 -90];
%  L = [0.3 0.5];
%
%% Hypothesis
% RRR planar robot.
%
%% Limitations
%
%% Version Control
%
% 1.0; Grupo 3; 2025/05/26 ; First issue.
%     
%% Function
function [jac] = jacobian(theta,L)

%% Validity


%% Main Calculations

J3 = [(L(1)*sind(theta(2)+theta(3))+sind(theta(3))*L(2))         (L(2)*sind(theta(3)))       0;
      (L(1)*cosd(theta(2)+theta(3))+cosd(theta(3))*L(2))         (L(2)*cosd(theta(3)))       0;
                              0                                           0                  0;
                              0                                           0                  0;
                              0                                           0                  0;
                              1                                           1                  1];    
                    

T01= [cosd(theta(1)) -sind(theta(1))  0   0 ;
      sind(theta(1))  cosd(theta(1))  0   0 ;
             0              0         1   0 ;
             0              0         0   1];

T12= [cosd(theta(2)) -sind(theta(2))  0   L(1) ;
      sind(theta(2))  cosd(theta(2))  0    0   ;
             0              0         1    0   ;
             0              0         0    1  ];

T23= [cosd(theta(3)) -sind(theta(3))  0   L(2) ;
      sind(theta(3))  cosd(theta(3))  0    0   ;
             0              0         1    0   ;
             0              0         0    1  ];

T= T01*T12*T23;

R = T(1:3, 1:3);   

MegaT = [R,      zeros(3);
         zeros(3),    R ];


%% Output Data

 jac = MegaT * J3;

end

