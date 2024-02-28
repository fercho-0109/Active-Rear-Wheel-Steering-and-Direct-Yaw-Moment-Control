function  A  = calc_A(v)
%% Dati veicolo
m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=v/3.6;          % [m/s] Longitudinal velocity
mu=1; 


%% Matrici del sistema

% A
a11=-(Cf+Cr)/(m*v);
a12=-1-(af*Cf-ar*Cr)/(m*v^2);
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*v);

% Ordine della matrice per il Reshape di Simulink

A = [a11 a21 a12 a22]; 

end
