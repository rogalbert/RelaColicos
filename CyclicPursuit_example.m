clear all, close all, clc
%% Init Project
initProject;
%%

N=5;                        % Number of agents
dt=0.01;                   % numerical steplength
max_iter = 5000;                           

% Initialize robotarium
rb = Robotarium('NumberOfRobots', N*2, 'ShowFigure', true);

transformation_gain = 0.06;
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', transformation_gain);

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 2*rb.robot_diameter);

% Initialize robots
xuni = rb.get_poses();                                    % States of real unicycle robots
x = uni_to_si_states(xuni(:,1:N));                                            % x-y positions only
x2 = uni_to_si_states(xuni(:,N+1:2*N));
agen = plot(xuni(1,1:N),xuni(2,1:N),'o','markersize',5,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
rb.set_velocities(1:2*N, zeros(2,2*N));                       % Assign dummy zero velocity
rb.step();                                                % Run robotarium step

% Cyclic graph
A = diag(ones(N-1,1),-1);
A(1,N) = 1; 
L = diag(sum(A)) - A;

% Target cycle definition 1
center = [-0.8;0];
radius = 0.5;
interAgentDistance = radius*2*sin(pi/N);
kp1 = 7;
kp2 = 0.4;
plot(center(1),center(2),'>','markersize',12)
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')

% Target cycle definition 2
center2 = [0.8;0];
radius2 = 0.4;
interAgentDistance2 = radius2*2*sin(pi/N);
plot(center2(1),center2(2),'<','markersize',12)
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% plot(radius2.*cos(th)+center2(1),radius2.*sin(th)+center2(2),'b')


% Reach initial positions on a circle
if 1        
    circularTargets = radius*[ cos( 0:2*pi/N:2*pi*(1- 1/N) ) ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ] + center;
    errorToInitialPos = x - circularTargets;                % Error
    errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
    circularTargets2 = radius2*[ cos( 0:2*pi/N:2*pi*(1- 1/N) ) ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ] + center2;
    errorToInitialPos2 = x2 - circularTargets2;                % Error
    errorNorm2 = [1,1]*(errorToInitialPos2.^2);               % Norm of error
    while (max( errorNorm ) > 0.01) || (max( errorNorm2 ) > 0.1)
        % Update state variables        
        xuni = rb.get_poses();                            % States of real unicycle robots
        x = uni_to_si_states(xuni(:,1:N));                                    % x-y positions
        x2 = uni_to_si_states(xuni(:,N+1:2*N));
        
        set(agen,'xdata',xuni(1,1:N),'ydata',xuni(2,1:N))
        drawnow
        % Update errors
        errorToInitialPos = x - circularTargets;
        errorNorm = [1,1]*(errorToInitialPos.^2);
        errorToInitialPos2 = x2 - circularTargets2;
        errorNorm2 = [1,1]*(errorToInitialPos2.^2);
        
        % Conput control inputs
        u = -0.3.*errorToInitialPos;
        u2 = -0.3.*errorToInitialPos2;
%         dx = si_to_uni_dyn(u, xuni);
        
        %% Avoid errors
        dx = [u u2];
        % To avoid errors, we need to threshold dxi
        norms = arrayfun(@(x) norm(dx(:, x)), 1:2*N);
        threshold = rb.max_linear_velocity/2;
        to_thresh = norms > threshold;
        dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);

        %% Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
        dx = si_barrier_cert(dx, xuni);
        dxu = si_to_uni_dyn(dx, xuni);

        rb.set_velocities(1:2*N, dxu); rb.step();                                       % Run robotarium step
    end
    disp('Initial positions reached')
    
end
  
for k = 1:max_iter
    
    % Get new data and initialize new null velocities
    xuni = rb.get_poses();                                % Get new robots' states
    x = uni_to_si_states(xuni(:,1:N));                                        % Extract single integrator states
    
    set(agen,'xdata',xuni(1,1:N),'ydata',xuni(2,1:N))
    drawnow
    dx = zeros(2,N);                                           % Initialize velocities to zero         
    for i = 1:N                
        for j = find(A(:,i))'
            if ~isempty(j)
                alpha = pi/N + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                dx(:,i) = dx(:,i) + R*( x(:,j)-x(:,i) ) - kp2*( x(:,i) - center );
            end
        end
    end
    %% Avoid errors
    dx = [dx zeros(2,N)];
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dx(:, x)), 1:2*N);
    threshold = rb.max_linear_velocity/2;
    to_thresh = norms > threshold;
    dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
    
    %% Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = si_barrier_cert(dx, xuni);
    dxu = si_to_uni_dyn(dx, xuni);
    
    rb.set_velocities(1:2*N, dxu); rb.step();              % Set new velocities to robots and update
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rb.debug();
