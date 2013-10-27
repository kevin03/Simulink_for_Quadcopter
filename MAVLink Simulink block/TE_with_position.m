% Trajectory_Estimation
%function [P V A]=TE(p,v,a)

clc
clear all

load('D:\Chi\Biomimetic Perching\Simulink_for_Quadcopter\MAVLink Simulink block\ShuttlecockData\test_7_xy.mat')

%syms x y z

m=0.015;    % KG
kd=0.15; % coefficient of aero-friction
Hi=1.3;     % Input Height threshold for the shuttlecock throwing flight
Ho=1;       % Target Height Output
g=9.8;      % Gravity acceleration

%v=[1 0 1];
%p=[0 0 1];
t0=1.68;
tend=2.48;
dt=0.02;
t=0:dt:(tend-t0);

m=round(t0/0.02+1);
n=round(tend/0.02+1);
L=n-m+1;
k=5;

p=Position.signals.values(m:n,:)';
v=reshape(Vel.signals.values(:,:,m:n),3,L);
a=reshape(acc.signals.values(:,:,m:n),3,L);

p=[p(1,:);-p(2,:);-p(3,:)];     %Coordination transformation
v=[v(1,:);-v(2,:);-v(3,:)];
a=[a(1,:);-a(2,:);-a(3,:)];

tic;                            %Start timing the estimation programme
dx=mean(diff(p(1,1:k)));
dy=mean(diff(p(2,1:k)));
x=[p(1,1:k),p(1,k)+[1:(L-k)]*dx];
y=[p(2,1:k),p(2,k)+[1:(L-k)]*dy];

tr_zx=polyval(polyfit(p(1,1:k),p(3,1:k),2),x);
tr_zy=polyval(polyfit(p(2,1:k),p(3,1:k),2),y);

[Hx,kx]=min(abs(tr_zx-Ho));
[Hy,ky]=min(abs(tr_zy-Ho));

xe=x(kx);
ye=y(ky);

pe=[xe,ye,Ho];
[Hz,kz]=min(abs(p(3,:)-Ho));
err=pe-p(:,kz)';
T=toc;                          %Stop timing
disp(['Calculation Time is ',num2str(T),'s']);
disp(['Estimation Error is ',num2str(err),' m']);

figure;
plot(p(1,:),p(3,:),'r',x,tr_zx,'b');
figure;
plot(p(2,:),p(3,:),'r',y,tr_zy,'b');
 
vv=sqrt(v(1,:).^2+v(2,:).^2+v(3,:).^2);
figure;
plot(t,vv,'b',t,v(1,:),'r',t,v(2,:),'g',t,v(3,:),'k');
figure;
plot(t,a(3,:),'r',t,gradient(v(3,:),0.02,'b'));
% c0=m*v(:,1)/k;
% c1=p(1,:);
% 
% x=c0(1)*(1-exp(-k*t/m))+c1(1);
% y=c0(2)*(1-exp(-k*t/m))+c1(2);
% z=(m^2*g/k^2+c0(3))*(1-exp(-k*t/m))-m*g*t/k+c1(3);
% 
% plot(t,p(3,:),'r',t,z,'b');
