function [X,Y,Z,T] = Ball_est(Xi,Yi,Zi,Vxi,Vyi,Vzi,Zd)

%define all variables needed
g = 9.81;                                                                       %gravity
vInit = [Vxi Vyi Vzi];                                                                %initial velocity
K = 0.136;                                                                          %friction constant                                                                       %desired height
pInit = [Xi Yi Zi];                                                                %initial position

%determine the necessary parameter
t1 = 1/sqrt(K*g) * atan(vInit(3)*sqrt(K/g));                                     %time needed for the ball to achieve its maximum height
zMax = pInit(3) + 1/(2*K) * log((g+K*vInit(3)*vInit(3))/g);                      %maximum height of the ball
vZimpact = sqrt(g*(1 - exp(2*K*(Zd - zMax)))/K);                                 %z-velocity at the impact point
theta = asin(-vZimpact*sqrt(K/g));                                               %integral limit
t2 = sqrt(1/(K*g)) * log(cos(theta)/(sin(theta) + 1));                           %time needed to go to the z - desired


%determining the velocity in z-direction
t = [0:0.01:t1];                                                                %time plot
v = sqrt(g/K) * tan(atan(vInit(3)*sqrt(K/g)) - sqrt(K*g)*t);                    %velocity in the z-axis
%figure;
%plot(t,v)

%determining the path in z-direction
z = pInit(3) + 1/(2*K) * log((g + K*vInit(3)*vInit(3))./(g+K*v.*v));            %calculate the trajectory
%figure;
%plot(t,z);

%determining the velocity in the z-direction after the maximum height
dvar = [0:0.01:vZimpact];                                                       %velocity plot
time = sqrt(1/(g*K))*log(sqrt(g-dvar.*dvar*K)./(sqrt(g)-dvar*sqrt(K))) + t1;
%figure;
%plot(time,dvar);

%overall plot of the velocity
totaltime = [t,time];                                                           %total time combining both event
overallvelocity = [v, dvar];                                                    %overall velocity span
figure;
plot(totaltime, overallvelocity);

%determining the trajectory for the event 2
path = zMax + 1/(2*K)*log((-g + K*dvar.*dvar)/-g);
%figure;
%plot(time, path);

%determining the total time
trajectory = [z,path];
figure;
plot(totaltime, trajectory);