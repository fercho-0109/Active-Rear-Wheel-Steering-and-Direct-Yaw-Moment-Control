function B = calc_B(v)
%% Dati veicolo

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=v/3.6;          % [m/s] Longitudinal velocity
              %

%% Matrici B del sistema
% B

b11=Cr/(m*v);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;


% Ordine della matrice per il Reshape di Simulink

B = [b11 b21 b12 b22];
end
