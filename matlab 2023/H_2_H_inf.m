function kH_2_H_inf=H_2_H_inf(A,B,B2,C2,D22,C1,D12,D11,a,b)
format long
dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
dim = size(B2);
    mw = dim(2);
dim = size(C2);
    nn = dim(1);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n); 
Y = sdpvar(m,n); 
Q=sdpvar(nn,nn); 
gamma2=sdpvar(1,1); 
gammainf=sdpvar(1,1);

% LMI constrains
F1=([(A*P+B*Y)+(A*P+B*Y)' B2;
    B2'            -eye(mw) ]<=0);

F2=([Q    (C2*P+D22*Y);
       (C2*P+D22*Y)'   P]>=0);

F3=(trace(Q)<=gamma2);   
F4=([Q]>=0);
F5=([P]>=0);
F6=(gamma2>=0);

F7 = ([(A*P+B*Y)+(A*P+B*Y)' B2 (C1*P+D12*Y)';
    B2' -gammainf*eye(mw) D11';
    C1*P+D12*Y D11 -gammainf*eye(nw)]<= 0); 

F8 = ([gammainf]>=0);

F = F1+F2+F3+F4+F5+F6+F7+F8;

% solution
opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,a*gammainf+b*gamma2,opts);
% control gain
kH_2_H_inf=double(Y)*inv(double(P));


end