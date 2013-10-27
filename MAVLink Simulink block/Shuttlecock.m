clc
clear

dt=0.02;

[X0,p,v,a]=readVCNdata('test_1_y',1.68,2.54);

j=1;
Xe=zeros(6,100,10);
kd=zeros(10,100);
k=15;

tic;
for i=1:4:37
    X0=[p(:,i);v(:,i)];
    [S,nt(j),Kd]=pred(X0,k,dt);
    Xe(:,1:nt(j)+1,j)=S;
    kd(j,1:nt(j)+1)=Kd;
    j=j+1;
end
T=toc;
disp(['Calculation Time is ',num2str(T),'s']);

n=size(p);
t=(0:n(2)-1)*dt;
te=(1:nt+1)*dt;
% figure;
% plot(t,p,'r',te,Xe(1:3,:),'b');
% figure;
% plot(t,v,'r',te,Xe(4:6,:),'b');
% figure;
% plot(te,kd);