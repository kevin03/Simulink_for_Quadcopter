%%TrajectoryEst
clc
clear

load('D:\Chi\Biomimetic Perching\Simulink_for_Quadcopter\MAVLink Simulink block\ShuttlecockData\test_1_y.mat');

syms tt

m=0.015;    % KG
kd=0.006; % coefficient of aero-friction
Hi=1.5;     % Input Height threshold for the shuttlecock throwing flight
Ho=1;       % Target Height Output
g=9.8;      % Gravity acceleration

t0=1.6;
tend=2.6;
dt=0.02;
t=0:dt:(tend-t0);

m=round(t0/0.02)+1;
n=round(tend/0.02)+1;
L=n-m+1;

p=Position.signals.values(m:n,:)';
v=reshape(Vel.signals.values(:,:,m:n),3,L);
a=reshape(acc.signals.values(:,:,m:n),3,L);

p=[p(1,:);-p(2,:);-p(3,:)];     %Coordination transformation
v=[v(1,:);-v(2,:);-v(3,:)];
a=[a(1,:);-a(2,:);-a(3,:)];

x0=p(1,1);
y0=p(2,1);
z0=p(3,1);
vx0=v(1,1);
vy0=v(2,1);
vz0=v(3,1);
vv=v(:,1)'*v(:,1);
dg=180/pi;
agx=acos(vx0/vv)*dg;
agy=acos(vy0/vv)*dg;
agz=acos(vz0/vv)*dg;

%for(kd=0.05:0.001:0.1)
c=kd/m;
tic;
x=x0+vx0/c-vx0/c*exp(-c*tt);
y=y0+vy0/c-vy0/c*exp(-c*tt);
cc=(g+c*vz0)/c^2;
z=-cc*exp(-c*tt)-g*tt/c+z0+cc;
tt=solve('-cc*exp(-c*tt)-g*tt/c+z0+cc=Ho',tt);
tz=subs(tt);
xe=subs(x,tz);
ye=subs(y,tz);

pe=[xe,ye,Ho];
[Hz,kz]=min(abs(p(3,:)-Ho));
err=pe-p(:,kz)';
tr=kz*dt;

T=toc;                          %Stop timing
disp(['Calculation Time is ',num2str(T),'s']);
disp(['Estimated Time ',num2str(tz),'s  ','Actual Time ',num2str(tr),'s']);
disp(['Estimation Error is ',num2str(err),' m']);
% figure;
% plot(t,v);
figure;
plot(t,subs(z,t),'r',t,p(3,:),'b');
ylabel('z');
figure;
plot(t,subs(x,t),'r',t,p(1,:),'b');
ylabel('x');
figure;
plot(t,subs(y,t),'r',t,p(2,:),'b');
ylabel('y');
