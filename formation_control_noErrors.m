%% Formation control utilizing edge tension energy with a static, undirected
%communication topology
%Paul Glotfelter 
%3/24/2016
%% Init Project
initProject;

%% Set up Robotarium object

N = 6;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);


%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
formation_control_gain = 4;
transformation_gain = 0.06;
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', transformation_gain);

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid.
L = [3 -1 0 -1 0 -1 ; ... 
    -1 3 -1 0 -1 0 ; ... 
    0 -1 3 -1 0 -1 ; ... 
    -1 0 -1 3 -1 0 ; ... 
    0 -1 0 -1 3 -1 ; ... 
   -1 0 -1 0 -1 3];

% The desired inter-agent distance for the formation
d = 0.4; 

% Pre-compute diagonal values for the rectangular formation
ddiag = sqrt((2*d)^2 + d^2);

% Weight matrix containing the desired inter-agent distances to achieve a
% rectuangular formation
weights = [ 0 d 0 d 0 ddiag; ... 
            d 0 d 0 d 0; ... 
            0 d 0 ddiag 0 d; ... 
            d 0 ddiag 0 d 0; ... 
            0 d 0 d 0 d; ... 
            ddiag 0 d 0 d 0];
%% Set enviroment
agenPlot = plot(0,0,'>','markersize',10,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');

x = r.get_poses();
xi = uni_to_si_states(x);
agen = plot(x(1,:),x(2,:),'o','markersize',5,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
r.set_velocities(1:N, zeros(2,N));
r.step();
%% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.

dxi = zeros(2, N);
xc = zeros(2,iterations);

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter);
% si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.5, ... 
%     'AngularVelocityLimit', 0.75*r.max_angular_velocity);

% Iterate for the previously specified number of iterations
for t = 0:iterations
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    xc(:,t+1) = (1/N*xi*ones(N,1));
    
    set(agenPlot,'xdata',xc(1,t+1),'ydata',xc(2,t+1))
    set(agen,'xdata',x(1,:),'ydata',x(2,:))
    drawnow
    
    %% Algorithm
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:N
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        % over agent i's neighbors
        dxi(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i from the communication
        % topology
        for j = topological_neighbors(L, i)
                
            % For each neighbor, calculate appropriate formation control term and
            % add it to the total velocity

            dxi(:, i) = dxi(:, i) + ...
            formation_control_gain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
            *(x(1:2, j) - x(1:2, i));
        end 
    end
    
     %% Avoid errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = r.max_linear_velocity/2;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dxi = si_barrier_cert(dxi, xi);
    dxu = si_to_uni_dyn(dxi, x);
    
    % Set velocities of agents 1:N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();   
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
