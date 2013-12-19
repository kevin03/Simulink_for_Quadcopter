function [ Af, Bf, Ap, Bp, At, Bt ] = QuadBallJuggling( n0, nf, p0, v0, pf, vf, T )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
g = 9.81;

dx1 = pf(1) - p0(1) - v0(1)*T;
dx2 = pf(2) - p0(2) - v0(2)*T;
dx3 = pf(3) - p0(3) - v0(3)*T + 0.5*g*T^2;

t0 = atan(n0(1)/n0(3));
p0 = asin(n0(2));
tf = atan(nf(1)/nf(3));
pf = asin(nf(2));

dt = tf - 
end

