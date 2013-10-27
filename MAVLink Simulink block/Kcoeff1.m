clc
clear

dt=0.02;

[X0,p,v,a]=readVCNdata('test_1_y',2.18,2.48);

n=length(p(1,:));
t=(0:n-1)*dt;

m=0.015;
g=9.8;

k=0.01:0.01:5;
j=length(k);
z=zeros(j,n);
vz=zeros(j,n);
zz=zeros(j,n);
vvz=zeros(j,n);

for i=1:j
    c=m/k(i);
    u=dt+c;
    uu=t(2:n)+c;
    z(i,1)=p(3,1);
    vz(i,1)=v(3,1);
    zz(i,1)=p(3,1);
    vvz(i,1)=v(3,1);
    
    zz(i,2:n)=z(i,1)+c^2*g+0.5*(vz(i,1)-c*g).*uu-0.5*c^2*(vz(i,1)+c*g)./uu;
    vvz(i,2:n)=c./(c+t(2:n)).*(vz(i,1)-g*t(2:n));
        
    for l=2:n
        z(i,l)=z(i,l-1)+c^2*g+0.5*(vz(i,l-1)-c*g)*u-0.5*c^2*(vz(i,l-1)+c*g)/u;
        vz(i,l)=c/(c+dt)*(vz(i,l-1)-g*dt);
    end
    
    errz(i)=sum((z(i,:)-p(3,:)).^2);
    errvz(i)=sum((vz(i,:)-v(3,:)).^2);
    errzz(i)=sum((zz(i,:)-p(3,:)).^2);
    errvvz(i)=sum((vvz(i,:)-v(3,:)).^2);
end

[ezm,zi]=min(errz);
[evzm,vzi]=min(errvz);
[ezzm,zzi]=min(errzz);
[evvzm,vvzi]=min(errvvz);

km=k(zi);
kkm=k(zzi);
plot(t,p,'r',t,z(zi,:),'b',t,zz(zzi,:),'k');
figure;
plot(t,v,'r',t,vz(zi,:),'b');