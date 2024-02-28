function KM = calc_KMax(vM)
% limit value od velocity 
vM=vM/3.6;          % [v] longitudinal velocity

% Vehicle parameters

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness ;               %

% desired model xdp= Ad*xd + Ed*deltaf
t_beta=0.05;        % [s]
k_beta=0;
t_gamma=0.0375;     % [s]
k_gamma=3.03; 

Ad=[-1/t_beta 0;
    0       -1/t_gamma];
Ed=[k_beta/t_beta;
    k_gamma/t_gamma];

% systen matrices at max velocity

% A

a11=-(Cf+Cr)/(m*vM);
a12=-1-(af*Cf-ar*Cr)/(m*vM^2);
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*vM);
% B
b11=Cr/(m*vM);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;
% E
e1=Cf/(m*vM);
e2=af*Cf/Iz;

AM=[a11 a12;
   a21 a22];

BM=[b11 b12;
   b21 b22];

EM=[e1;
   e2];

B2M=[(EM-Ed) (AM-Ad)];

%% desired output
C1=[1 0;0 1;0 0;0 0];
rho=5e-13;
D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
D11=zeros(4,3);

% LMI FOR THE CONTROL

dim = size(AM);
    n = dim(1);
dim = size(BM);
    m = dim(2);
dim = size(B2M);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% constrains

F1 = ([(AM*P+BM*Y)+(AM*P+BM*Y)' B2M (C1*P+D12*Y)';
    B2M' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0);

F2 = ([gamma]>=0);
F3 = ([P]>=0);
F = F1+F2+F3;

% solution

opts=sdpsettings('solver','mosek','verbose',0);
solvesdp(F,gamma,opts);

% contol gain

k=double(Y)*inv(double(P));


% for remake in simulink

KM=[k(1,1) k(2,1) k(1,2) k(2,2)]
