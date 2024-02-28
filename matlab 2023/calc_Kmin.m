function Km = calc_Kmin(vm)

% limit value of velocity 
vm=vm/3.6;          % [v] longitudinal velocity

% Vehicle parameters

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 


% desired model xdp= Ad*xd + Ed*deltaf
t_beta=0.05;        % [s]
k_beta=0;
t_gamma=0.0375;     % [s]
k_gamma=3.03; 

Ad=[-1/t_beta 0;
    0       -1/t_gamma];
Ed=[k_beta/t_beta;
    k_gamma/t_gamma];

% systen matrices at min velocity 

% A
a11=-(Cf+Cr)/(m*vm);
a12=-1-(af*Cf-ar*Cr)/(m*vm^2);
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*vm);
% B
b11=Cr/(m*vm);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;
% E
e1=Cf/(m*vm);
e2=af*Cf/Iz;

Am=[a11 a12;
   a21 a22];

Bm=[b11 b12;
   b21 b22];
% Cm=[1 0;
%    0 1];
% Dm=[0 0;0 0];
Em=[e1;
   e2];
B2m=[(Em-Ed) (Am-Ad)];


%% desired output
C1=[1 0;0 1;0 0;0 0];
rho=5e-13;
D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
D11=zeros(4,3);

% LMI FOR THE CONTROL

dim = size(Am);
    n = dim(1);
dim = size(Bm);
    m = dim(2);
dim = size(B2m);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% CONTRAISN 

F1 = ([(Am*P+Bm*Y)+(Am*P+Bm*Y)' B2m (C1*P+D12*Y)';
    B2m' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0);

F2 = ([gamma]>=0);
F3 = ([P]>=0);
F = F1+F2+F3;

% solution

opts=sdpsettings('solver','mosek','verbose',0);
solvesdp(F,gamma,opts);

% controla gain

k=double(Y)*inv(double(P));


% for remake in simulink

Km=[k(1,1) k(2,1) k(1,2) k(2,2)]
end
