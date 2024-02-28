function [M X]=LMI_region(A,B,alfa,theta,r)
dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
P = sdpvar(n,n);

% LMI constrains 

F1 = ([P*A+A'*P+2*alfa*P]<= 0);  
F2=([-r*P   P*A;
    A'*P -r*P ]<= 0);
F3=([sin(theta)*(P*A+A'*P)  cos(theta)*(P*A-A'*P);
     cos(theta)*(A'*P-P*A) sin(theta)*(P*A+A'*P)]<= 0);
F = F1+F2+F3;

% LMI solver
opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,P,opts);
% opts=sdpsettings('solver','sedumi');
% optimizer(F,P,opts,[],0);

% verification
 
X=double(P);

M1=X*A+A'*X+2*alfa*X; % 2x2 matrix

M2=[-r*X   X*A;
    A'*X -r*X ];      % 4x4 matrix 

M3=[sin(theta)*(X*A+A'*X)  cos(theta)*(X*A-A'*X);
     cos(theta)*(A'*X-X*A) sin(theta)*(X*A+A'*X)];   % 2x4 matrix

 M=[M1 zeros(n,4*n) ;
     zeros(2*n,n) M2 zeros(2*n,2*n);
     zeros(2*n,3*n) M3];
end