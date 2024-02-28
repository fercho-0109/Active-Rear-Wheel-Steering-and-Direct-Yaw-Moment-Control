function [k_LQ]=LQ(Cz,Dzu,A,B,t)
format long
if t==0
Q=Cz'*Cz;
R=Dzu'*Dzu;
P=sdpvar(2,2);
Y=sdpvar(2,2);
gamma=sdpvar(2,2);
F1=([P*A'+A*P+Y'*B'+B*Y P Y';
        P -inv(Q) zeros(2,2);
        Y zeros(2,2) -inv(R)] <= 0);
F2=(P >= 0);
F3=([gamma eye(2);eye(2) P] >= 0);
F=F1+F2+F3;
opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,trace(gamma),opts);
k_LQ=double(Y)*inv(double(P))
end
if t==1
    dim = size(A);
    n = dim(1);
    dim = size(B);
    m = dim(2);
    Q=Cz'*Cz
    Q=[1e-15 0 0 0;0 1e-15 0 0;0 0 1 0;0 0 0 1]
    R=Dzu'*Dzu;
    P=sdpvar(4,4);
    Y=sdpvar(2,4);
    gamma=sdpvar(4,4);
    F1=([P*A'+A*P+Y'*B'+B*Y P Y';
        P -inv(Q) zeros(n,m);
        Y zeros(m,n) -inv(R)] <= 0);
    F2=(P >= 0);
    F3=([gamma eye(4);eye(4) P] >= 0);
    F=F1+F2+F3;
    opts=sdpsettings('solver','sedumi','verbose',0);
    solvesdp(F,trace(gamma),opts);
    k_LQ=double(Y)*inv(double(P));
end
end