%% KIN
% Calcula a cinemática direta para um manipulador RRR planar.
% 
%% Calling Syntax
% wrelb = kin(theta, L)
%
%% I/O Variables
% |IN Double Array| *theta*, *L*: Array de valores de theta (em graus;
% [1x3]) e de L (em metros, [1x2])
% 
% |OU Double Matrix| *wrelb*: : Transformation Matrix 4x4
% 
%% Example
%  theta = [90 0 -90];
%  L = [0.3 0.5];
%  wrelb = kin(theta, L)
%
%% Hypothesis
% RRR planar robot.
%
%% Limitations
%
%% Version Control
%
% 1.0; Grupo 3; 2025/04/19 ; First issue.
%     
%% Function
function [ wrelb ] = kin( theta, L )

%% Validity


%% Main Calculations
T1 = [cosd(theta(1))    -sind(theta(1))   0     0   ;
     sind(theta(1))     cosd(theta(1))    0     0   ;
          0                  0            1     0   ;
          0                  0            0     1  ];
   
T2 = [cosd(theta(2))    -sind(theta(2))   0     L(1);
     sind(theta(2))     cosd(theta(2))    0     0   ;
          0                  0            1     0   ;
          0                  0            0     1  ];
      
T3 = [cosd(theta(3))    -sind(theta(3))   0     L(2);
     sind(theta(3))     cosd(theta(3))    0     0   ;
          0                  0            1     0   ;
          0                  0            0     1  ];
            
wrelb_t = tmult(T1,T2);
M = tmult(wrelb_t, T3);

%% Output Data
wrelb = M;

end
