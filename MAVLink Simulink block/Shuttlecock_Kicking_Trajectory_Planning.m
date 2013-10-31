%made by Huynh Minh Quan, Nanyang Technological University
%in according to the paper 

T = 0.34; %the time given to perform the task, 5s
h = 0.02; % the time interval between the steps, 0.1s
K = T/h + 1;% number of time steps
N = 1;% number of UAVs
g = 9.81;

p1 = [1 0 -1.5]';%initial positions [p11 p21 p31 p41]
pK = [1.3 0 -1.5]';% final  positions [p1K p2K p3K p4K]

v1 = zeros(1,3*N)';%initial velocities
vK = -1.6*[0.685 0 0.7285]';%final velocities

a1 = zeros(1,3*N)';%initial acc
aK = -g*[0.9402 0 0]';%final acc


Aeq = zeros(12*N,3*N*K); %12*N equalities constraints for a1,aK,vK and pK,p1 and v1 are within the equations

Aeq(1:3*N,1:3*N) = eye(3*N);
Aeq(3*N+1:3*N*2,3*N*(K-1)+1:3*N*K) = eye(3*N);
for i=1:3*N;
    for j=1:3*N*(K-1);
        if (mod(j,3*N) == mod(i,3*N))
            Aeq(3*N*2+i,j) = h;
        end;
    end;
end;

for i=1:3*N;
    x = 3;
    for j=1:3*N*(K-1);
        if (mod(j,3*N) == mod(i,3*N))
            Aeq(3*N*3+i,j) = (2*K-x)*h^2/2;
            x = x+2;
        end;
    end;
end;


beq = zeros(1,12*N)';
beq(1:3*N) = a1;
beq(3*N+1:3*N*2) = aK;
beq(3*N*2+1:3*N*3) = vK - v1;
beq(3*N*3+1:3*N*4) = pK - p1 - h*(K-1)*v1;


q = zeros(3*N*K,1);  
for i = 1:3*N*K,
    if (mod(i,3) == 0)
        q(i) = 2*g;
    end;
end;
tic
% [x,fval] = quadprog(eye(3*N*K,3*N*K),q'/2,[],[],Aeq,beq,[],[],[]);
E = eye(3*N*K,3*N*K);
tic
cvx_begin quiet
    variable x(3*N*K,1);
    minimize(x'*E*x + q'*x);
    subject to
    Aeq*x == beq;
cvx_end
t = toc

