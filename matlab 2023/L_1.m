function [K_L1]=L_1(A,B,B2,C3,D32,D31,lambda)
% z(t)L1=C3*x(t)+D32*u(t)+D31*w(t)
format long
dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
dim = size(B2);
    mw = dim(2);
dim = size(C3);
    nc = dim(1);

    
X=sdpvar(n,n);
Y=sdpvar(m,n);
gamma1=sdpvar(1,1);
mu1=sdpvar(1,1);

% constrains
V1=([(A*X+B*Y)'+(A*X+B*Y)+lambda*X,B2; ...
    B2',-mu1*eye(mw)]<=0);
V2=([lambda*X,zeros(n,mw),(C3*X+D32*Y)'; ...
    zeros(mw,n),(gamma1-mu1)*eye(mw),D31'; ...
    (C3*X+D32*Y),D31,gamma1*eye(nc)]>=0);
V3=([X]>=0);
V4=([gamma1]>=0);

Constrein=V1+V2+V3+V4;
opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(Constrein,gamma1,opts);
K_L1=double(Y)*inv(double(X));

end