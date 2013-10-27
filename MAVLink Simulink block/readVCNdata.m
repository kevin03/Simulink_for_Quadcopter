function [X0,p,v,a]=readVCNdata (b,t0,tend)


a='D:\Chi\Biomimetic Perching\Simulink_for_Quadcopter\MAVLink Simulink block\ShuttlecockData\';
c='.mat';
d=[a,b,c];
%path=strcat(root,nm,fomat);

load(d);

dt=0.02;
t=0:dt:(tend-t0);

m=round(t0/dt+1);
n=round(tend/dt+1);
L=n-m+1;

p=Position.signals.values(m:n,:)';
v=reshape(Vel.signals.values(:,:,m:n),3,L);
a=reshape(acc.signals.values(:,:,m:n),3,L);

p=[p(1,:);-p(2,:);-p(3,:)];     %Coordination transformation
v=[v(1,:);-v(2,:);-v(3,:)];
a=[a(1,:);-a(2,:);-a(3,:)];

X0=[p(:,1);v(:,1)];

% plot(t,v);
% figure;
% plot(t,p);