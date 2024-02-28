function S=LMI_stability(A,B)
dim = size(A);
nx = dim(1);
dim = size(B);
nm = dim(2);
P = sdpvar(nx,nx);

% LMI costrain

F1=([-P   zeros(nx);
    zeros(nx) P*A+A'*P ]<=0);
F2=(P>=0);
F = F1+F2;

% solution

opts=sdpsettings;
opts.solver='sedumi';
solvesdp(F,P,opts);

% check

X=double(P)
S=A'*X+X*A
end