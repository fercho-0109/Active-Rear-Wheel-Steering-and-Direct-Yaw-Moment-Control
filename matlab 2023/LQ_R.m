function [k_LQ_R]=LQ_R(Cz,Dzu,A,B,alpha,theta,r)
    format long
    Q=Cz'*Cz;
    R=Dzu'*Dzu;
    X=sdpvar(2,2);
    Y=sdpvar(2,2);
    gamma=sdpvar(2,2);

   F1=([X*A'+A*X+Y'*B'+B*Y X Y';
        X -inv(Q) zeros(2,2);
        Y zeros(2,2) -inv(R)] <= 0);
   F2=(X >= 0);
    F3=([gamma eye(2);eye(2) X] >= 0);

%POLE PLACE
    F4=([2*alpha*X+A*X+B*Y+X*A'+Y*B']<=0);

    F5=([sin(theta)*(A*X+B*Y+X*A'+Y'*B') cos(theta)*(-A*X-B*Y+X*A'+Y'*B');
    cos(theta)*(A*X+B*Y+X*A'-Y'*B') sin(theta)*(A*X+B*Y+X*A'+Y'*B')]<=0);

    F6=([-r*eye(2) A*X+B*Y;
    (A*X+B*Y)' -r*eye(2)]<=0);

    F=F1+F2+F3+F4+F5+F6;
    opts=sdpsettings('solver','sedumi','verbose',0);
    solvesdp(F,trace(gamma),opts);
    k_LQ_R=double(Y)*inv(double(X));

end