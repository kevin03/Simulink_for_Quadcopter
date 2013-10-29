% Trajectory_Estimation
%function [P V A]=TE(p,v,a)

clc
clear all

load('D:\Chi\Biomimetic Perching\Simulink_for_Quadcopter\MAVLink Simulink block\ShuttlecockData\test_8_xy.mat')

syms s r

m=0.015;    % KG
kd=0.15; % coefficient of aero-friction
Hi=1.3;     % Input Height threshold for the shuttlecock throwing flight
Ho=1.3;       % Target Height Output
g=9.8;      % Gravity acceleration

%v=[1 0 1];
%p=[0 0 1];
t0=1.46;
tend=2.28;
dt=0.02;
t=0:dt:(tend-t0);

m=round(t0/0.02+1);
n=round(tend/0.02+1);
L=n-m+1;
dk=5;
di=floor((L-1)/dk)-2;

p=Position.signals.values(m:n,:)';
v=reshape(Vel.signals.values(:,:,m:n),3,L);
a=reshape(acc.signals.values(:,:,m:n),3,L);

p=[p(1,:);-p(2,:);-p(3,:)];     %Coordination transformation
v=[v(1,:);-v(2,:);-v(3,:)];
a=[a(1,:);-a(2,:);-a(3,:)];

dim=2*L;
tr_zx=zeros(di,dim);
tr_zy=zeros(di,dim);
pe=zeros(di,3);
err=zeros(di,3);

                           
for i=1:di
    tic;                             %Start timing the estimation programme
    j=(i-1)*dk+1;

    x=linspace(p(1,j),p(1,end),dim);
    y=linspace(p(2,j),p(2,end),dim);

%     xx=max(subs(solve(polyfit(p(1,j:j+dk),p(3,j:j+dk),2)*[s^2;s;1]-Ho,s)));
%     yy=max(subs(solve(polyfit(p(2,j:j+dk),p(3,j:j+dk),2)*[r^2;r;1]-Ho,r)));
    
    tr_zx(i,1:dim)=polyval(polyfit(p(1,j:j+dk),p(3,j:j+dk),2),x);
    tr_zy(i,1:dim)=polyval(polyfit(p(2,j:j+dk),p(3,j:j+dk),2),y);

    [Hx,kx]=min(abs(tr_zx(i,:)-Ho));
    [Hy,ky]=min(abs(tr_zy(i,:)-Ho));

    xe(i)=x(kx);
    ye(i)=y(ky);

    pe(i,:)=[xe(i),ye(i),Ho];
    [Hz,kz]=min(abs(p(3,:)-Ho));
    err(i,:)=pe(i,:)-p(:,kz)';
    T(i)=toc;                          %Stop timing
    
%     figure;
%     plot(p(2,:),p(3,:),'r',y,tr_zy(i,1:2*(L-j+1)),'b');
%     title(['Estimation ',num2str(i)]);
%     ylabel('Z');
%     xlabel('Y');
end

%disp(['Calculation Time is ',num2str(T),'s']);
% disp(['Estimation Error is ',num2str(err),' m']);

% figure;
% plot(p(1,:),p(3,:),'r',x,tr_zx(3,:),'b');
% figure;
% plot(p(2,:),p(3,:),'r',y,tr_zy(3,:),'b');
%  
% vv=sqrt(v(1,:).^2+v(2,:).^2+v(3,:).^2);
% figure;
% plot(t,vv,'b',t,v(1,:),'r',t,v(2,:),'g',t,v(3,:),'k');

% figure;
% plot(t,a(3,:),'r',t,gradient(v(3,:),0.02,'b'));
% c0=m*v(:,1)/k;
% c1=p(1,:);
% 
% x=c0(1)*(1-exp(-k*t/m))+c1(1);
% y=c0(2)*(1-exp(-k*t/m))+c1(2);
% z=(m^2*g/k^2+c0(3))*(1-exp(-k*t/m))-m*g*t/k+c1(3);
% 
% plot(t,p(3,:),'r',t,z,'b');
