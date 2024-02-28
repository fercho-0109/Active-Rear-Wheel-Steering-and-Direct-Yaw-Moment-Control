function inv_B = calc_inv_B(v)
%% Dati veicolo
m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=v/3.6;          % [m/s] Longitudinal velocity
mu=1; 
 

%% Matrici B del sistema
% B
b11=mu*Cr/(m*v);
b12=0;
b21=-ar*Cr*mu/Iz;
b22=1/Iz;

% Ordine della matrice per il Reshape di Simulink
B=[b11 b12; b21 b22];
b=inv(B);
inv_B = [b(1,1) b(2,1) b(1,2) b(2,2)];

end
