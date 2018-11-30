clear all, close all, clc
%% Init Project
initProject;
%% Setup Robotarium object

N=5;                        % Number of agents per team                          
% Initialize robotarium
rb = Robotarium('NumberOfRobots', N*2, 'ShowFigure', true);

senseRatio = 5*rb.robot_diameter;

videoFLag = 0;
% Initialize video
if videoFLag 
    vid = VideoWriter('RealColicos.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end

%% Grab tools for converting to single-integrator dynamics and ensuring safety 
formationControlGain = 8;
transformation_gain = 0.06;
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', transformation_gain);
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 1.4*rb.robot_diameter);

max_iter = 1000; 

% Initialize robots
xuni = rb.get_poses();                                    % States of real unicycle robots
x = uni_to_si_states(xuni(:,1:N));                                            % x-y positions only
x2 = uni_to_si_states(xuni(:,N+1:2*N));
agen = plot(xuni(1,1:N),xuni(2,1:N),'o','markersize',12,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
agen2 = plot(xuni(1,N+1:2*N),xuni(2,N+1:2*N),'o','markersize',12,'MarkerFaceColor',[0.65,0.49,0.12],'MarkerEdgeColor','none');
rb.set_velocities(1:2*N, zeros(2,2*N));                       % Assign dummy zero velocity
rb.step();                                                % Run robotarium step

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid. COmplete graph to ensure
% formation from all initial conditions
d=0.5;
dd=sqrt(d^2+d^2);
ddh=dd/2;
dtfb=0.2;
dtfh=d/2;
ldf=sqrt(dtfh^2+(d+dtfb)^2);
dfsev=0.1;
dfs=d/2+dtfb;
lds= sqrt(dtfh^2+(d+dtfb+dfsev)^2);
sdf=sqrt(dtfb^2+dtfh^2);
sds=sqrt(dtfh^2+(dtfb+dfsev)^2);
LFDef = [6 -1 -1 -1 -1 -1 -1; ... 
    -1 6 -1 -1 -1 -1 -1; ... 
    -1 -1 6 -1 -1 -1 -1; ... 
    -1 -1 -1 6 -1 -1 -1; ... 
    -1 -1 -1 -1 6 -1 -1;
    -1 -1 -1 -1 -1 6 -1;
    -1 -1 -1 -1 -1 -1 6];
LF=[LFDef zeros(7);zeros(7) LFDef]; %complete undirected graph for formation control
%formation geometry
weightsDef=[0 d d dd ldf ddh lds;
    d 0 dd d sdf ddh sds;
    d dd 0 d ldf ddh lds;
    dd d d 0 sdf ddh sds;
    ldf sdf ldf sdf 0 dfs dfsev;
    ddh ddh ddh ddh dfs 0 dfs+dfsev;
    lds sds lds sds dfsev dfs+dfsev 0];
weightsDef = weightsDef*1.1;
nullMatrix=zeros(7);
weights=[weightsDef nullMatrix;
    nullMatrix weightsDef];

% Cyclic graph
LC=[1 0 -1 0 0 0 0;
    -1 1 0 0 0 0 0;
    0 0 1 -1 0 0 0;
    0 0 0 1 -1 0 0;
    0 -1 0 0 1 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0];

% Target cycle definition 1
center = [-0.9;0];
radius = 1.5;
interAgentDistance = radius*2*sin(pi/N);
kp1 = 6;
kp2 = 0.9;
plot(center(1),center(2),'>','markersize',15,'MarkerFaceColor',[0,0,1])
% th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')

% Target cycle definition 2
center2 = [1.1;0];
radius2 = 0.4;
interAgentDistance2 = radius2*2*sin(pi/N);
plot(center2(1),center2(2),'<','markersize',15,'MarkerFaceColor',[1,0,0])
% th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% plot(radius2.*cos(th)+center2(1),radius2.*sin(th)+center2(2),'b')

obs_center = [0;0.1];
obs_r = 0.2;
th = 0:pi/50:2*pi;
xunit = obs_r * cos(th) + obs_center(1);
yunit = obs_r * sin(th) + obs_center(2);
plot(xunit, yunit,'MarkerFaceColor',[0, 204, 153]/255);

%%
stateH = 0;
stateV = 0;
t1 = 0;
t2 = 0;
t = 0;
game=1;

while (game)
    % Get new data and initialize new null velocities
    clear L x x2;
    xuni = rb.get_poses();                                % Get new robots' states
    x = uni_to_si_states(xuni(:,1:N));
    x2 = uni_to_si_states(xuni(:,N+1:2*N));
    
    set(agen,'xdata',xuni(1,1:N),'ydata',xuni(2,1:N))
    set(agen2,'xdata',xuni(1,N+1:2*N),'ydata',xuni(2,N+1:2*N))
    drawnow
    %%
    dx1 = zeros(2,N);
    switch(stateH)
        case 0 %Formation control
            L = LF;
            x=[x [center(1);center(2)] [center(1)+dtfb+dfsev+d/2;center(2)]];
            for i = 1:N
                for j = topological_neighbors(L, i)
                    dx1(:, i) = dx1(:, i) + formationControlGain*(norm(x(:, i) - x(:, j))^2 - weights(i, j)^2) ... 
                          *(x(1:2, j) - x(1:2, i));
                end
            end
            if t1 >= max_iter
                stateH = 2;
                t1 = 0;
                disp('Formation complete')
            end
        case 1 % CyclicPursuit
            L=LC;
            for i = 1:N
                for j = topological_neighbors(L, i)
                    alpha = pi/N + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                    R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                    dx1(:,i) = dx1(:,i) + R*( x(:,j)-x(:,i) ) - kp2*( x(:,i) - center );
                end
            end
            [max,min,avg] = distTeam(x,x2);
            if min > 10 *rb.robot_diameter
                stateH = 1;
%                 t1 = 0;
            elseif t1 >= 2*max_iter
                game = 0;
            end
            
        case 2 % explore/wait
            [max,min,avg] = distTeam(x,x2);
            if min < 8*rb.robot_diameter
                stateH = 1;
                t1 = 0;
            end
    end
    %%
    dx2 = zeros(2,N);
    switch(stateV)
        case 0 %Formation control
            L = LF;
            x2=[x2 [center2(1);center2(2)] [center2(1)-dtfb-dfsev-d/2;center2(2)] ];
            for i = 1:N
                for j = topological_neighbors(L, i)
                    dx2(:, i) = dx2(:, i) + formationControlGain*(norm(x2(:, i) - x2(:, j))^2 - weights(i, j)^2) ... 
                          *(x2(1:2, j) - x2(1:2, i));
                end
            end
            if t2 >= max_iter
                stateV = 2;
                t2 = 0;
            end
        case 1 % CyclicPursuit
            
        case 2 % exploration
            dx2(:,N) = 0.04*(x2(:,N)-center2)/norm(x2(:,N)-center2);
            xen = enemies (x2(:,N), x, senseRatio);
%             dx2(:,N) = explore(dx2(:,N), x2(:,N),xen);
            Nen = size (xen,2);
            if norm(x2(:,N)-center) > senseRatio
                if Nen > 1
                    dx2(:,N) = 0;
                end
                for j = 1:Nen
                    dx2(:, N) = dx2(:, N) + 0.3*(xen(1:2, j) - x2(1:2, N));
                end
            elseif norm(x2(:,N)-center) < 0.05
                game = 0;
                disp('Game Over')
            else
                dx2(:, N) =0.5*(center - x2(1:2, N));
                
            end
    end
    
    %% Avoid errors
    dx = [dx1 dx2];
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dx(:, x)), 1:2*N);
    threshold = rb.max_linear_velocity/2;
    to_thresh = norms > threshold;
    
    dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
    
    %% Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = BC_obstable(dx,xuni, obs_center, obs_r);
    dx = si_barrier_cert(dx, xuni);
    dxu = si_to_uni_dyn(dx, xuni);
    
    rb.set_velocities(1:2*N, dxu); rb.step();              % Set new velocities to robots and update
    
    %% Video cpature
    if videoFLag && mod(t,10)                               % Record a video frame every 10 iterations
            writeVideo(vid, getframe(gcf)); 
    end
    %% Update times
    t1 = t1 + 1;
    t2 = t2 + 1;
    t = t + 1;
end
if videoFLag; close(vid); end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rb.debug();

function [maxVal, minVal, avgVal] = distTeam (x,x2)

N = size(x, 2);
d= zeros(N);
for i = 1:N
    for j = 1:N
        d(i,j) = norm(x(:,i)-x2(:,i));
    end
end

maxVal = max(max(d));
minVal = min(min(d));
avgVal = mean(mean(d));


end

function [set] = enemies (agent,x, senseRatio)
set=[];
N = size(x, 2);
for j = 1:N
    if norm(agent - x(:,j)) < senseRatio
        set = [set x(:,j)];
    end
end
end

function [dx] = explore (dxi,agent,x)

N = size(x,2);
safety_radius = 1;
gamma = 100;
opts = optimoptions(@quadprog,'Display','off');
if (N <= 0) 
   dx = dxi;
   return 
end
num_constraints = N;
A = zeros(num_constraints, 2);
b = zeros(num_constraints, 1);
count = 1;
for j = 1:N
    h = norm(agent-x(1:2,j))^2 - safety_radius^2;
    A(count, :) =  2*(agent-x(:,j))';
    b(count) = gamma*h^3;
    count = count + 1;
end

%Solve QP program generated earlier
H = 2*eye(2);
f = 2*dxi;

vnew = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opts);

%Set robot velocities to new velocities
dx = reshape(vnew, 2, 1);

end

function [ dx ] = BC_obstable(dxi,x, obs_center, obs_r)
    gamma = 100;
    opts = optimoptions(@quadprog,'Display','off');
    N = size(dxi, 2);

    if(N < 2)
       dx = dxi;
       return 
    end

    x = x(1:2, :);
    num_constraints = 1;
    A = zeros(num_constraints, 2*N);
    b = zeros(num_constraints, 1);
    count = 1;
    for i = 1 : N
        h = norm(x(1:2,i)-obs_center)^2-(obs_r+0.05)^2;
        A(count, (2*i-1):(2*i)) = -2*(x(:,i)-obs_center)';
        b(count) = gamma*h^3;
        count = count + 1;
    end

    %Solve QP program generated earlier
    vhat = reshape(dxi,2*N,1);
    H = 2*eye(2*N);
    f = -2*vhat;

    vnew = quadprog(sparse(H), double(f), A, b, [],[], [], [], [], opts);

    %Set robot velocities to new velocities
    dx = reshape(vnew, 2, N);
end