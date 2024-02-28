function kH_2=H_2_gain(A,B,B2,C2,D22)
format long
dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
dim = size(B2);
    mw = dim(2);
dim = size(C2);
    nn = dim(1);

P = sdpvar(n,n); % create the unknow variable
Y = sdpvar(m,n); % create the unknow variable
Q=sdpvar(nn,nn); % create the unknow variable
gamma2=sdpvar(1,1); % create the unknow variable 

% LMI constrains
F1=([(A*P+B*Y)+(A*P+B*Y)' B2;
    B2'            -eye(mw) ]<=0);

F2=([Q    (C2*P+D22*Y);
       (C2*P+D22*Y)'   P]>=0);

F3=(trace(Q)<=gamma2);   
F4=([Q]>=0);
F5=([P]>=0);
F6=(gamma2>=0);

F=F1+F2+F3+F4+F5+F6;

% solution
opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,gamma2,opts);
% control gain
kH_2=double(Y)*inv(double(P));

end