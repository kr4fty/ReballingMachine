pkg load control
M=csvread('datos.csv',0,0) % "FILE", fila_inicial, columna_inical
%M=readmatrix('datos.csv')
u=M(:,1)    % Step
%t=M(:,2)   % Time
y=M(:,3)    % Heater Ring
%y=M(:,4)     % Board
datos=iddata(y,u,1)
%pidTuner
%
%             Kp
%  G(s) = ---------- * exp(-Td*s)
%          1+Tp1*s
%
%        Kp = 2.3467
%       Tp1 = 155.32
%        Td = 29.241
%
%
%    Kp = 0.744, Ki = 0.00495
%
