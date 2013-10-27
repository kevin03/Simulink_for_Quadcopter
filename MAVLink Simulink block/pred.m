%TE with Kalman Filter
function [Xe,nt,kd]=pred(X0,k,dt)

% dt=0.02;
i=1;
g=9.8;
G=[0 0 -g]';
kd=zeros(200);
kd(1)=k;                             %Initial value of Kd
B=[0 0 -0.5*g*dt^2 0 0 -g*dt]';     %Coefficient Matrix B of the state space model
H=eye(6);                           %Measurement Matrix of the state space model

zthres=1.5;
ythres=1.5;                         %Target Zone Boundary

Q=0.01;
R=0.01*eye(6);                      %Coefficients of 

Rd0=0.1;
pd=0.1;

Xe=zeros(6,200);
X=X0;
v=sqrt(sum(X(4:6).^2));
e=0.5*v^2+g*X(3);

% SYS=ss(A,B,H,0,dt);                 %Discrete state space model
% [X,L,P]=kalman(SYS,Q,R);            %Kalman filter

while X(3)>zthres                     %||(X(2)<ythres)
%for i=1:200
    A=[eye(3),(dt-kd(i)*dt^2/2)*eye(3);zeros(3,3),(1-kd(i)*dt)*eye(3)];

    Xe(:,i)=A*X+B;

    V=sqrt(sum(Xe(4:6,i).^2));            %Estimated Velocity of the shuttlecock
    E=0.5*V^2+g*Xe(3,i);                  %Estimated Mechanical Energy of the shuttlecock
    Vm=(0.5*(V+v))^2;
    Kd_m=-(E-e)/(dt*Vm);                %Measured Kd

    Rd=Rd0/Vm;                          %Measurement noise variance
    Cd=pd/(pd+Rd);                      %Intermediary gain
    Kd=kd(i)+Cd*(Kd_m-kd(i));                 %Estimated Kd
    Pd=pd*Rd/(pd+Rd);                   %Estimated variance of Kd

% Recursive maneuvers
    X=Xe(:,i);
    v=V;
    e=E;
    kd(i+1)=Kd;
    pd=Pd;
    i=i+1;
    
    
end

nt=i-1;

Xe=[X0,Xe(:,1:nt)];
kd=kd(1:nt+1);
