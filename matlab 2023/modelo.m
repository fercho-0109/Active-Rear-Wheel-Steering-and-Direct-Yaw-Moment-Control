%Simulation of a Active Rear Wheel Steering and Direct Yaw Moment Control 
%
%-----SIMULATION IN MATLAB/Simulink--------------------------
%
% #################################################################################
% Edited by Alexis Marino (alexismarino0109@gmail.com) 
% #################################################################################
%
%____________Vehicel Control Proyect Mod_1_____________%
clear all;
clc
close all;
format short;
%% ------------------Select the operation with 1--------------------------%
Structural_properties = 0;
LMI_Lyapunov_stability = 0;
LMI_regionf = 0;
H_2_controller=0;
H_inf_controller=0;
L_1_controller=0;
LQ_controller=0;
H_2_H_inf_controller=0;
LQ_R_controller=0;
comparasion=0;
integral_control_H2_Hinf_L1_LQ=1;
Robust_control_H_inf=0;

%-------------------------------------------------------------------------%
%% Parameters definition of the single track vehicel model----------%%
m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=80/3.6;          % [m/s] Longitudinal velocity
Fz=3830;

parameters=[m,Iz,af,ar,Cf,Cr,v,Fz];

x0=[0, 0];      % Initial conditions beta/gamma

% parameters of the desire model
t_beta=0.05;        % [s]
k_beta=0;
t_gamma=0.0375;     % [s]
k_gamma=3.03; 

% State Space model
a11=-(Cf+Cr)/(m*v);
a12=-((af*Cf-ar*Cr)/(m*v^2))-1;
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*v);

b11=Cr/(m*v);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;

e1=Cf/(m*v);
e2=af*Cf/Iz;

A=[a11 a12;
   a21 a22];
B=[b11 b12;
   b21 b22];
C=[1 0;
   0 1];
E=[e1;
   e2];
D=[0 0;0 0];
model_M=[A,B,C,D,E];
% desired model xdp= Ad*xd + Ed*deltaf
Ad=[-1/t_beta 0;
    0       -1/t_gamma];
Ed=[k_beta/t_beta;
    k_gamma/t_gamma];
%% structural properties 
if Structural_properties==1
    ctrb(A,B)
    obsv(A,C)
    ragg=rank(ctrb(A,B))        % Reachability
    oss=rank(obsv(A,C))         % Obsevability 
    eigenvalues=eig(A)           % check the stability
end
%% LMI Lyapunov stability 
if LMI_Lyapunov_stability==1
    Q = LMI_stability(A,B);
    eig_s = eig(Q);
    isnegdef = all(eig_s < 0)
    if isnegdef == 1
        disp('-------------------------------------------')
	    disp('the LMI constrain matrix is negative define')
        disp('-------------------------------------------')
    else
        disp('-------------------------------------------')
	    disp('the LMI constrain matrix is positive define')
        disp('-------------------------------------------')
    end
end
%% LMI Region
if LMI_regionf==1
    eigenvalues=eig(A);
    alfa=-max(real(eigenvalues(1)),real(eigenvalues(2)))
    theta=max(angle(eigenvalues(1)),angle(eigenvalues(2)))
    r=max(abs(eigenvalues(1)),abs(eigenvalues(2)))
    [M P] = LMI_region(A,B,alfa,theta,r);
    eig_m = eig(M);
    flag = 0;
    for i = 1:rank(M)
	    if eig_m(i) <= 0 
	     flag = 1;
	    end
    end
    if flag == 1
        disp('---------------------------------------------')
	    disp('the LMI costrain matrix M is negative defined')
        disp('---------------------------------------------')
    else
        disp('---------------------------------------------')
	    disp('the LMI costrain matrix M is positive defined')
        disp('---------------------------------------------')
    end
end
%%  H2 Optimal Control 
if H_2_controller==1
    rho=5e-12;
    D22=sqrt(rho)*[0 0;0 0;1 0;0 1];
    B2=[(E-Ed) (A-Ad)];
    C2=[1 0 ;0 1;0 0;0 0];
    % H2 gain
    k=H_2_gain(A,B,B2,C2,D22)
    eigv_H2=eig(A+B*k)

    % Plot Steering Wheel Angle vs Yaw Rate
    s=sim('primera_parte');
    delta_f=s.simout.Data;
    GammaRef=s.simout1.Data;
    Gamma=s.simout2.Data;
    GammaNC=s.simout3.Data;

    figure(1)
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');
    legend({'Without control ','Ref','With control H_2'},'Location','southeast')
    title('Yaw rate response at the curve test')
    grid minor
end
%% H_inf Optimal control
if H_inf_controller==1
    C1=[1 0;0 1;0 0;0 0];
    rho=5e-12;
    D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
    D11=zeros(4,3);
    B2=[(E-Ed) (A-Ad)];
    k = H_inf(A,B,B2,C1,D12,D11)
    eigH_inf=eig(A+B*k)

    % Plot Steering Wheel Angle vs Yaw Rate
    s=sim('primera_parte');

    delta_f=s.simout.Data;
    GammaNC=s.simout3.Data;
    Gamma=s.simout2.Data;
    GammaRef=s.simout1.Data;
    
    figure(1)
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');
    legend({'Without control ','Ref','With control H_\infty'},'Location','southeast')
    title('Yaw rate response at the Curva test')
    grid minor
end
%% L_1 controller
if L_1_controller==1
    rho=5e-10;
    lambda=2.51;
    B2=[(E-Ed) (A-Ad)];
    C3=[1 0;0 1;0 0;0 0];
    D32=sqrt(rho)*[0 0;0 0;1 0;0 1];
    D31=zeros(4,3);

    % k_1 gain
    k=L_1(A,B,B2,C3,D32,D31,lambda)
    eigL_1=eig(A+B*k)

    % Plot Steering Wheel Angle vs Yaw Rate
    s=sim('primera_parte');

    delta_f=s.simout.Data;
    GammaNC=s.simout3.Data;
    Gamma=s.simout2.Data;
    GammaRef=s.simout1.Data;
   
    figure(1)
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');
    legend({'Without control ','Ref','With control L_1'},'Location','southeast')
    title('Yaw rate response at the Curva test')
    grid minor
end
%% LQ controller
if LQ_controller==1
    Cz=[1 0;0 1;0 0;0 0];
    rho=5e-12;
    Dzu=sqrt(rho)*[0 0;0 0;1 0;0 1];
    % LQ gain
    k=LQ(Cz,Dzu,A,B,0)
    eigLQ=eig(A+B*k)

    %Plot Steering Wheel Angle vs Yaw Rate
    s=sim('primera_parte');
    delta_f=s.simout.Data;
    GammaNC=s.simout3.Data;
    Gamma=s.simout2.Data;
    GammaRef=s.simout1.Data;
    figure(1)
   
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');                     
    legend({'Without control ','Ref','With control LQ'},'Location','southeast')
    title('Yaw rate response at the moose test')
    grid minor
end
%% H_2_H_inf_controller
if H_2_H_inf_controller==1
    C1=[1 0;0 1;0 0;0 0];
    rho=5e-12;
    D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
    D11=zeros(4,3);
    B2=[(E-Ed) (A-Ad)];
    D22=sqrt(rho)*[0 0;0 0;1 0;0 1];
    C2=[1 0 ;0 1;0 0;0 0];
    a=0.7;
    b=(1-a);
    k=H_2_H_inf(A,B,B2,C2,D22,C1,D12,D11,a,b)
    eigH2_INF=eig(A+B*k)

    %Plot Steering Wheel Angle vs Yaw Rate
    s=sim('primera_parte');
    delta_f=s.simout.Data;
    GammaNC=s.simout3.Data;
    Gamma=s.simout2.Data;
    GammaRef=s.simout1.Data;
    figure(1)
   
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');                     
    legend({'Without control ','Ref','With control H_2 H_\infty'},'Location','southeast')
    title('Yaw rate response at the moose test')
    grid minor
end
%% LQ-R controller
if LQ_R_controller==1
    Cz=[1 0;0 1;0 0;0 0];
    rho=5e-12;
    Dzu=sqrt(rho)*[0 0;0 0;1 0;0 1];

    alpha=250;
    theta=0;
    r=2e5;

     % LQ gain
    k=LQ_R(Cz,Dzu,A,B,alpha,theta,r)
    eigLQ_R=eig(A+B*k)

    % Figure Angolo di sterzo anteriore vs Velocita di imbardata
    s=sim('primera_parte');

    %Plot Steering Wheel Angle vs Yaw Rate
    delta_f=s.simout.Data;
    GammaNC=s.simout3.Data;
    Gamma=s.simout2.Data;
    GammaRef=s.simout1.Data;

    figure(1)
    plot(delta_f,GammaNC,'r',delta_f,GammaRef,'--black',delta_f,Gamma,'b')
    xlabel('Yaw rate [rad/s]');
    ylabel('Steering angle [rad]');                     
    legend({'Without control ','Ref','With control LQ-R'},'Location','southeast')
    title('Yaw rate response at the moose test')
    grid minor
end
%% comparation
if comparation==1
    rho=5e-12;
    D22=sqrt(rho)*[0 0;0 0;1 0;0 1];
    B2=[(E-Ed) (A-Ad)];
    C2=[1 0 ;0 1;0 0;0 0];
    % H2 gain
    k2=H_2_gain(A,B,B2,C2,D22)  

    C1=[1 0;0 1;0 0;0 0];
    D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
    D11=zeros(4,3);
    % k_inf gain
    kinf= H_inf(A,B,B2,C1,D12,D11)

    rho1=5e-10;
    lambda=2.51;
    C3=[1 0;0 1;0 0;0 0];
    D32=sqrt(rho1)*[0 0;0 0;1 0;0 1];
    D31=zeros(4,3);
    % k_1 gain
    k1=L_1(A,B,B2,C3,D32,D31,lambda)

    Cz=[1 0;0 1;0 0;0 0];
    Dzu=sqrt(rho)*[0 0;0 0;1 0;0 1];
    % LQ gain
    kLQ=LQ(Cz,Dzu,A,B,0)

    a=0.7;
    b=(1-a);
    kH2Hinf=H_2_H_inf(A,B,B2,C2,D22,C1,D12,D11,a,b)

    alpha=250;
    theta=0;
    r=2e5;
     % LQ gain
    kLQR=LQ_R(Cz,Dzu,A,B,alpha,theta,r)

    sim('comparation');

end
%% integral control 1 pole 
if integral_control_H2_Hinf_L1_LQ==1
    format long
 
    % Extended space
    % Xp_hat =  A*x + B1*u + B2*df 
    % Xe_hat = r - C*x - D*u + [-Fw]df

     Fw=zeros(2,1);
     Aex=[zeros(2) -C;zeros(2) A];
     B1=B;
     Bex=[-D;B1];
     B2=E;
     Eex=[(-Fw) eye(2);B2 zeros(2)];
     C1=[1 0;0 1];
     rho=1e-15;
     D12=sqrt(rho)*[zeros(4,2);1 0;0 1];
     Cz=[eye(2) zeros(2);zeros(4,4)];
     Dzu=D12;
     Dzw=zeros(6,3);
     lambda=2080.5101053;%6.3101053 8.5 980.5101053
    % uscita di prestazioni z=[0 Cz;1 0][xe x]' + Dzu*u + [Fz 0][wp xd]'
    % put in zero the errors


     %  Hinf
     K_Hinf = H_inf(Aex,Bex,Eex,Cz,Dzu,Dzw)  
     autH8in=eig(Aex+Bex*K_Hinf);
     kff_hinf=[K_Hinf(1,1),K_Hinf(1,2);K_Hinf(2,1),K_Hinf(2,2)];
     kfb_hinf=[K_Hinf(1,3),K_Hinf(1,4);K_Hinf(2,3),K_Hinf(2,4)];
 
     %  H2
     K_H2=H_2_gain(Aex,Bex,Eex,Cz,Dzu)  
     autH2in=eig(Aex+Bex*K_H2);
     kff_h2=[K_H2(1,1),K_H2(1,2);K_H2(2,1),K_H2(2,2)];
     kfb_h2=[K_H2(1,3),K_H2(1,4);K_H2(2,3),K_H2(2,4)];
 
     %  L1
     K_L1=L_1(Aex,Bex,Eex,Cz,Dzu,Dzw,lambda)  
     autL1in=eig(Aex+Bex*K_L1);
     kff_l1=[K_L1(1,1),K_L1(1,2);K_L1(2,1),K_L1(2,2)];
     kfb_l1=[K_L1(1,3),K_L1(1,4);K_L1(2,3),K_L1(2,4)];
 
     %  LQ
     % K_LQ=LQ(Cz,Dzu,Aex,Bex,1) 
     % autL1in=eig(Aex+Bex*K_LQ);
     % kff_lq=[K_LQ(1,1),K_LQ(1,2);K_LQ(2,1),K_LQ(2,2)];
     % kfb_lq=[K_LQ(1,3),K_LQ(1,4);K_LQ(2,3),K_LQ(2,4)];
    
     sim('integral_1_pole');
end
%% Robust control H inf 
if Robust_control_H_inf==1
    C1=[1 0;0 1;0 0;0 0];
    rho=5e-12;
    D12=sqrt(rho)*[0 0;0 0;1 0;0 1];
    D11=zeros(4,3);
 
    A1=[-(Cf+Cr)/(m*50) -((af*Cf-ar*Cr)/(m*50^2))-1;
        a21 a22];
    A2=[-(Cf+Cr)/(m*100) -((af*Cf-ar*Cr)/(m*100^2))-1;
        a21 a22];
    B11=[Cr/(50*v) b12;
        b21 b22];    
    B12=[Cr/(100*v) b12;
        b21 b22];  
    E1=[Cf/(50*v);
         e2];
    E2=[Cf/(100*v);
         e2];
    B21=[(E1-Ed) (A1-Ad)];   
    B22=[(E2-Ed) (A2-Ad)];

    k = H_inf_robust(A1,A2,B11,B12,B21,B22,C1,D12,D11)
    eigH_inf=eig(A+B*k)

    % Plot Steering Wheel Angle vs Yaw Rate
    sim('robust');


end
