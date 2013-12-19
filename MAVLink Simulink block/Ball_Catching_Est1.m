function [x,y,z,time] = Ball_Catching_Est(xi,yi,zi,vxi,vyi,vzi,zd)

g = 9.81;
m = 1;

t1 = (0-vzi)/g;
zmax = zi + vzi*t1 + 0.5*g*t1^2;

t2 = sqrt(2*(zd-zmax)/g);
time = t1+t2;
position = zeros(1,3);
x = xi + vxi*time;
y = yi + vyi*time;
z = zd;
