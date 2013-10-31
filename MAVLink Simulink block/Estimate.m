function [ racket_orientation, racket_velocity, position, time] = Estimate( pi, vi, zd, m, drag_coef, cor )
%pi is the initial position of the shuttlecock
%vi = [vx, vy, vz]' is the initial velocity
%drag_coef is the drag coefficient
%cor is the coefficient of restitution of the racket
%racket orientation is the normal of racket's plane [x, y, z]
%p is the desired position at the same height
%v is the velocity at the desired position
%T is the time taken to reach the desired position
k = drag_coef;
b = cor;
f = 50;
g = -9.81;

%Let zi is the initial height of the shuttlecock
xi = pi(1);
yi = pi(2);
zi = pi(3);
%Let vzi is the initial position in z-direction
vxi = vi(1);
vyi = vi(2);
vzi = vi(3);
%Calculate the maximum height
zmax = zi + m*vzi/k + m^2*g/k^2*log((m*g-k*vzi)/m/g)
%Find the pre-impact velocity
syms x;
vz = vpa(solve(zmax-zd == m*x/k + m^2*g/k^2*log((m*g-k*x)/m/g),x),3);
%Find the time taken until impact
T = m/k*log((m*g-k*vzi)/(m*g-k*vz));

vx = vxi*exp(-k*T/m);
vy = vyi*exp(-k*T/m);

xd = xi + vxi*m/k*(1 - exp(-k*T/m));
yd = yi + vyi*m/k*(1 - exp(-k*T/m));

position = [xd, yd, zd]';

vp = [vx, vy, vz]'
vf = [-vxi, -vyi, vzi]'
nd = (vp - vf)/norm(vp - vf,2);
vd = (b*vp+vf)'*nd/(1+b);

racket_orientation = nd;
racket_velocity = vd;

time = T;

end

