function [ ] = PlotUAV( number_of_UAVs, time_interval, time_step, terminal_positions, terminal_velocities, accelerations_of_UAVs )
%This function generates the positions of the UAVs considering each UAV as a sphere.
%number_of_UAVs is the number of UAVs in the fleet.
%time_interval is the time given to finish the task.
%time_step is the time_step required
%terminal_positions are the initial and final positions
%terminal_velocities are the initial and final velocities
%acceleration_of_UAVs are the series of accelerations at all steps.
N = number_of_UAVs;
T = time_interval;
h = time_step;
K = T/h+1;
p1 = terminal_positions(1:3*N);
pK = terminal_positions(3*N+1:6*N);
v1 = terminal_velocities(1:3*N);
vK = terminal_velocities(3*N+1:6*N);

%[v1 v2 ... vK]' = [v1 v1 .... v1]' + T*[0 a1 a2 ... aK-1]'
%[p1 p2 ... pK]' = [p1 p1 .... p1]' + T10*v1 + T20*[0 a1 a2 ... aK-1]'

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

acc = accelerations_of_UAVs;

positions_of_UAVs = p10 + T10 + T20*acc;
velocities_of_UAVs = v10 + T*acc;

figure;
hold on;
pause on;
for i = 1:K;
    for j = 0:3:3*(N-1)
        P = positions_of_UAVs(3*(i-1)*N+j+1:3*(i-1)*N+j+3);
        C(:,:,1) = j;
        C(:,:,2) = 2*j;
        C(:,:,3) = 3*j;
        scatter3(P(1),P(2),P(3),50,j/15,'filled');
        pause(0.1);
        hold on;
    end;
end;

end

