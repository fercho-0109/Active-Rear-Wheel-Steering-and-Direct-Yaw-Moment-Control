function E = calc_E(v)
%% Dati veicolo

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=v/3.6;          % [m/s] Longitudinal velocity
mu=1;

%% Modello desiderato xdp= Ad*xd + Ed*deltaf
% Definiamo i parametri del modello desiderato

taub=0.05;          % [s]
kbeta=0;
taugamma=0.0395;    % [s]
kgamma=2.87;        

ad11=-1/taub;
ad12=0;
ad21=0;
ad22=-1/taugamma;

ed1=kbeta/taub;
ed2=kgamma/taugamma;

%% Matrici del sistema

%% Velocita' minima [vm]

% A
a11=-mu*(Cf+Cr)/(m*v);
a12=-1-mu*(af*Cf-ar*Cr)/(m*v^2);
a21=-mu*(af*Cf-ar*Cr)/(Iz);
a22=-mu*(af^2*Cf+ar^2*Cr)/(Iz*v);

% E
e1=Cf*mu/(m*v);
e2=af*Cf*mu/Iz;


% Ordine della matrice per il Reshape di Simulink
% B2=[(E-Ed) (A-Ad)];
% E=[(e1-ed1) (e2-ed2) (a11-ad11) (a21-ad21) (a12-ad12) (a22-ad22)]; 
E=[e1 e2];
end


