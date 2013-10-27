%Kcoeff

clc
clear

dt=0.02;

[X0,p,v,a]=readVCNdata('test_1_y',2.0,2.7);

n=length(p(1,:));
t=(0:n-1)*dt;

m=0.015;
g=9.8;

for i=1:3
    pp(i,:)=polyval(polyfit(t,p(i,:),4),t);
    vv(i,:)=polyval(polyfit(t,v(i,:),3),t);
end
figure;
plot(t,p,'r',t,pp,'b');
figure;
plot(t,v,'r',t,vv,'b');

for i=1:n-1
    kx(i)=m/(-dt-0.5*vv(1,i)*dt^2/(pp(1,i+1)-pp(1,i)-vv(1,i)*dt));
    ky(i)=m/(-dt-0.5*vv(2,i)*dt^2/(pp(2,i+1)-pp(2,i)-vv(2,i)*dt));
    c=pp(3,i+1)-pp(3,i)-vv(3,i)*dt;
    cc(i)=2*(pp(3,i+1)-pp(3,i))-vv(3,i)*dt;
    kz(i)=(-2*m*c-m*g*dt^2)/(vv(3,i)*dt^2+2*dt*c);
end

tt=t(1:n-1);
figure;
plot(tt,kx,'r',tt,ky,'b',tt,kz,'g');
figure;
plot(tt,cc);
k=mean([kx;ky;kz],2);