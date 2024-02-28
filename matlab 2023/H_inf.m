function [K_inf] = H_inf(A,B,B2,C1,D12,D11)
format long

dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
dim = size(B2);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% Constrains to solve

F1 = ([(A*P+B*Y)+(A*P+B*Y)' B2 (C1*P+D12*Y)';
    B2' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F2 = ([gamma]>=0);
F3 = ([P]>=0);
F = F1+F2+F3;

% Risoluzione delle LMI

opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,gamma,opts);

% control gain

K_inf=double(Y)*inv(double(P));

end
