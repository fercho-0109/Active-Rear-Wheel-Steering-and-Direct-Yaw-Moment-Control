function K_inf = H_inf_robust(A1,A2,B11,B12,B21,B22,C1,D12,D11)
format long

dim = size(A1);
    n = dim(1);
dim = size(B12);
    m = dim(2);
dim = size(B21);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% Constrains to solve

F1 = ([(A1*P+B11*Y)+(A1*P+B11*Y)' B21 (C1*P+D12*Y)';
    B21' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F2 = ([(A2*P+B12*Y)+(A2*P+B12*Y)' B22 (C1*P+D12*Y)';
    B22' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F3 = ([gamma]>=0);
F4 = ([P]>=0);
F = F1+F2+F3+F4;

% Risoluzione delle LMI

opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,gamma,opts);

% control gain

K_inf=double(Y)*inv(double(P));

%k=[K_inf(1,1) K_inf(2,1) K_inf(1,2) K_inf(2,2)]

end
