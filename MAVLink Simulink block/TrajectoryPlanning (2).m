tic
T = 5; %the time given to perform the task, 5s
h = 0.1; % the time interval between the steps, 0.1s
K = T/h + 1;% number of time steps
N = 4;% number of UAVs
g = 9.81;

p1 = 2*[1 1 1 1 -1 1 -1 -1 1 -1 1 1]';%initial positions [p11 p21 p31 p41]
pK = 2*[-1 -1 1 -1 1 1 1 1 1 1 -1 1]';% final  positions [p1K p2K p3K p4K]

v1 = zeros(1,3*N)';%initial velocities
vK = zeros(1,3*N)';%final velocities

a1 = zeros(1,3*N)';%initial acc
aK = zeros(1,3*N)';%final acc


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

%% v = v10 + T*a;
%% p = p10 + T10 + T20*a;

a10 = zeros(3*N*K,1);
for i = 1:K,
    a10(3*N*(i-1)+1:3*N*i) = a1;
end;

v10 = zeros(3*N*K,1);
for i = 1:K,
    v10(3*N*(i-1)+1:3*N*i) = v1;
end;

T = zeros(3*N*K,3*N*K);
for i=3*N+1:3*N*K;
    for j=1:3*N*(K-1);
        if (mod(j,3*N) == mod(i,3*N))
            T(i,j) = h;
        end;
    end;
end;

p10 = zeros(3*N*K,1);
for i = 1:K,
    p10(3*N*(i-1)+1:3*N*i) = p1;
end;
T10 = zeros(3*N*K,1);
for i = 1:K,
    T10(3*N*(i-1)+1:3*N*i) = v1*(i-1);
end;
T10 = T10*h;

T20 = zeros(3*N*K,3*N*K);
for i=3*N+1:3*N*K;
    for j=1:i-3*N;
        if (mod(j,3*N) == mod(i,3*N))
            T20(i,j) = (2*(i-j)/(3*N)-1)*h^2/2;
        end;
    end;
end;

%% p = (0:8;0:8;0:4)
pmax = zeros(1,3*N*K)';
for i = 1:3*N*K,
    if(mod(i,3) == 1)
        pmax(i) = 8;
    else if(mod(i,3) == 2)
            pmax(i) = 8;
        else
            pmax(i) = 4;
        end;
    end;
end;

pmin = -pmax;

vmin = -pmax/2;
vmax = pmax/2;

amin = vmin;
amax = vmax;

jmin = 4*amin;
jmax = 4*amax;

jmax(3*N*(K-1)+1:3*N*K) = [];
jmin(3*N*(K-1)+1:3*N*K) = [];

%tranform the constraints

I = eye(3*N*K,3*N*K);
I1 = T;
I2 = T20;
I3 = zeros(3*N*(K-1),3*N*K);
for i = 1:3*N*(K-1);
    I3(i,i) = -1/h;
    I3(i,i+3*N) = 1/h;
end;


Aineq = zeros(9*N*K-3*N,3*N*K);

Aineq(1:3*N*K,:) = I1;
Aineq(3*N*K+1:6*N*K,:) = I2;
Aineq(6*N*K+1:9*N*K-3*N,:) = I3;

bmax = [vmax-v10;pmax-p10-T10;jmax(1:3*N*(K-1))];
bmin = [vmin-v10;pmin-p10-T10;jmin(1:3*N*(K-1))];

fmin = g^2;
fmax = 4*fmin;

q = zeros(3*N*K,1);  
for i = 1:3*N*K,
    if (mod(i,3) == 0)
        q(i) = 2*g;
    end;
end;
r = g^2;
opts = optimset('Algorithm','active-set');
% [x,fval] = quadprog(eye(3*N*K,3*N*K),q'/2,Aineq,bineq,Aeq,beq,[],[],[],opts);
E = eye(3*N*K,3*N*K);
cvx_begin quiet
    variable x(3*N*K,1);
    minimize(x'*E*x + q'*x);
    subject to
    Aeq*x == beq;
    bmin <= Aineq*x <= bmax;
    for i = 1:N*K;
        x(3*(i-1)+1:3*i)'*x(3*(i-1)+1:3*i) + 2*g*x(3*i) + g^2 <= fmax;
        x(3*i) == 0;
    end;
cvx_end


x0 = x;

max_iteration = 5;
for m = 1:max_iteration;

    pq = p10 + T10 + T20*x;
    %find the values of 2-norm and 1-norm pq(i) - pq(j)
    p2_norm = [];
    dp = [];
    dp_nn = [];
    for i=1:K;
        for j=1:N-1;
           for k=j+1:N;
               p2_norm = [p2_norm; norm(pq(3*N*(i-1)+3*(j-1)+1:3*N*(i-1)+3*j)-pq(3*N*(i-1)+3*(k-1)+1:3*N*(i-1)+3*k),2)]; 
               dp = [dp; pq(3*N*(i-1)+3*(j-1)+1:3*N*(i-1)+3*j)-pq(3*N*(i-1)+3*(k-1)+1:3*N*(i-1)+3*k)];
               dp_nn = [dp_nn; (pq(3*N*(i-1)+3*(j-1)+1:3*N*(i-1)+3*j)-pq(3*N*(i-1)+3*(k-1)+1:3*N*(i-1)+3*k))/norm(pq(3*N*(i-1)+3*(j-1)+1:3*N*(i-1)+3*j)-pq(3*N*(i-1)+3*(k-1)+1:3*N*(i-1)+3*k),2)]; 
           end;
        end;
    end;

    dp_n = zeros(3*K*N*(N-1)/2,K*N*(N-1)/2);
    for i=1:K*N*(N-1)/2;
        dp_n(3*(i-1)+1:3*i,i) = dp_nn(3*(i-1)+1:3*i);
    end;
    
    Tr = zeros(3*N*K*(N-1)/2,3*N*K);
    for i = 1:K;
        y = 0;
        for j = 1:(N-1);
            for k = j+1:N;
                Tr(3*(i-1)*N*(N-1)/2+y+3*(k-j-1)+1  :  3*(i-1)*N*(N-1)/2+3*(k-j-1)+y+3  , 3*N*(i-1)+3*(j-1)+1  :  3*N*(i-1)+3*(j-1)+3) = eye(3,3);
                Tr(3*(i-1)*N*(N-1)/2+y+3*(k-j-1)+1  :  3*(i-1)*N*(N-1)/2+3*(k-j-1)+y+3  , 3*N*(i-1)+3*(k-1)+1  :  3*N*(i-1)+3*(k-1)+3) = -eye(3,3);
            end;
            y = y + 3*(N-j);
        end;
    end;

    min_distance = (ones(K*N*(N-1)/2,1)/2+dp_n'*dp-p2_norm);
    
    Aineqt = zeros(17.5*N*K-6*N+0.5*K*N^2,3*N*K);
    
    Aineqt(1:3*N*K,:) = I1;
    Aineqt(3*N*K+1:6*N*K,:) = -I1;
    Aineqt(6*N*K+1:9*N*K,:) = I2;
    Aineqt(9*N*K+1:12*N*K,:) = -I2;
    Aineqt(12*N*K+1:15*N*K-3*N,:) = I3;
    Aineqt(15*N*K-3*N+1:18*N*K-6*N,:) = -I3;
    Aineqt(18*N*K-6*N+1:17.5*N*K-6*N+0.5*K*N^2,:) = -dp_n'*Tr*I2;

    bineqt = [vmax-v10;-vmin+v10;pmax-p10-T10;-pmin+p10+T10;jmax(1:3*N*(K-1));-jmin(1:3*N*(K-1));-min_distance+dp_n'*Tr*(p10+T10)];
    
%    [x,fval] = quadprog(eye(3*N*K,3*N*K),q'/2,Aineqt,bineqt,Aeq,beq,[],[],[],opts);
    E = eye(3*N*K,3*N*K);
    cvx_begin quiet
        variable x(3*N*K,1);
        minimize(x'*E*x + q'*x);
        subject to
        Aeq*x == beq;
        Aineqt*x <= bineqt;
        for i = 1:N*K;
            x(3*(i-1)+1:3*i)'*x(3*(i-1)+1:3*i) + 2*g*x(3*i) + g^2 <= fmax;
            x(3*i) == 0;
        end;
    cvx_end   
end;
t = toc;
