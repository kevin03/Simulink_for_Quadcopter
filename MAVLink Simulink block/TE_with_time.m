% Trajectory_Estimation
%function [P V A]=TE(p,v,a)

clc
clear all

load('D:\Chi\Biomimetic Perching\Simulink_for_Quadcopter\MAVLink Simulink block\ShuttlecockData\test_1_y.mat')

%syms x y z

m=0.015;    % KG
kd=0.15;  % coefficient of aero-friction
Hi=1.5;     % Input Height threshold for the shuttlecock throwing flight
Ho=1.3;       % Target Height Output
g=9.8;      % Gravity acceleration

%v=[1 0 1];
%p=[0 0 1];
t0=1.68;
tend=2.48;
dt=0.02;
t=0:dt:(tend-t0);

m=floor(t0/0.02+1);
n=floor(tend/0.02+1);
L=n-m+1;
k=10;

p=Position.signals.values(m:n,:)';
v=reshape(Vel.signals.values(:,:,m:n),3,L);
a=reshape(acc.signals.values(:,:,m:n),3,L);

p=[p(1,:);-p(2,:);-p(3,:)];     %Coordination transformation
v=[v(1,:);-v(2,:);-v(3,:)];
a=[a(1,:);-a(2,:);-a(3,:)];

tic;                            %Start timing the estimation programme
tr_zt=polyval(polyfit(t(1:k),p(3,1:k),2),t);
[Hz,kz]=min(abs(p(3,:)-Ho));
[Ht,kt]=min(abs(tr_zt-Ho));

te=dt*(kt-1);
xe=polyval(polyfit(t(1:k),p(1,1:k),1),t);
ye=polyval(polyfit(t(1:k),p(2,1:k),1),t);

pe=[xe(kt),ye(kt),Ho];
err=pe-p(:,kz)';
T=toc;                          %Stop timing
disp(['Calculation Time is ',num2str(T),'s']);
disp(['Estimation Error is ',num2str(err),' m']);

figure;
plot(t,p(3,:),'r',t,tr_zt,'b');
figure;
plot(t,p(1,:),'r',t,xe,'r',t,p(2,:),'b',t,ye,'b');

vv=sqrt(v(1,:).^2+v(2,:).^2+v(3,:).^2);
figure;
plot(t,vv,'b',t,v(1,:),'r',t,v(2,:),'g',t,v(3,:),'k');

c0=m*v(:,1)/kd;
c1=p(1,:);

x=c0(1)*(1-exp(-kd*t/m))+c1(1);
y=c0(2)*(1-exp(-kd*t/m))+c1(2);
z=(m^2*g/kd^2+c0(3))*(1-exp(-kd*t/m))-m*g*t/kd+c1(3);

figure;
plot(t,p(3,:),'r',t,z,'b');
