%% SIL Parameter
BaudRate = 115200;
T_sil = 1;
f_sil = 1/T_sil;

%% DC Motor Parameter
J = 0.01;   % moment of inertia of the rotor
b = 0.1;    % motor viscous friction constant
Ke = 0.367;  % electromotive force constant
Kt = 0.367;  % motor torque constant
R = 0.1;      % electric resistance 
L = 0.5;    % electric inductance
T_L = 0;    % load torque
vb = 0;     % voltage drop in brush

%% PID Control Parameter

% Motor Speed Regulation
Kp_1 = 0.5;
Ki_1 = 0.1;
Kd_1 = 0.1;
  
% Current Regulation
Kp_2 = 7.5;
Ki_2 = 0.75;
Kd_2 = 0.1;




