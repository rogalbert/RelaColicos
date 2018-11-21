%% Formation control utilizing edge tension energy with a static, undirected
%communication topology
%Paul Glotfelter 
%3/24/2016

%% Rabeya Jamshad
%11/19/2018
% Formation control seems to be working. 
% Circular pursuit works with a specific gain. no idea why?
% the formation control does have one very strange behavior with 1/3
% collapsing on to each other. need to know why
format long;
close all;

%% Setup Robotarium object

% Get Robotarium object used to communicate with the robots/simulator
% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 5; 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1000;%2000

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid. COmplete graph to ensure
% formation from all initial conditions
LF = [5 -1 -1 -1 -1 -1; ... 
    -1 5 -1 -1 -1 -1; ... 
    -1 -1 5 -1 -1 -1; ... 
    -1 -1 -1 5 -1 -1; ... 
    -1 -1 -1 -1 5 -1;
    -1 -1 -1 -1 -1 5];
%formation geometry
weights=[0 0.5 0.5 0.7071 0.8382 0.35355;
    0.5 0 0.7071 0.5 0.3905 0.35355;
    0.5 0.7071 0 0.5 0.8382 0.35355;
    0.7071 0.5 0.5 0 0.3905 0.35355;
    0.8382 0.3905 0.8382 0.3905 0 0.55;
    0.35355 0.35355 0.35355 0.35355 0.55 0];

% cycle graph for cyclic pursuit
LC=[1 0 -1 0 0;
    -1 1 0 0 0;
    0 0 1 -1 0;
    0 0 0 1 -1;
    0 -1 0 0 1];

kp2=0.4; %0.6
kp1 = 7;
center=[0;0];
radius = 0.4;
interAgentDistance = radius*2*sin(pi/N);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.08);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);


rotate=0;
game=1;
Flag=plot(center(1),center(2),'x','markersize',16,'MarkerFaceColor',[0.62,0.49,0.15],'linewidth',3);
% Iterate for the previously specified number of iterations
while (game)
switch(rotate)
    case 0
        L=LF; %complete undirected graph for formation control
        for t = 0:iterations

            % Retrieve the most recent poses from the Robotarium.  The time delay is
            % approximately 0.033 seconds
            x = r.get_poses();
            x=[x [center(1);center(2);0.23]]; %add the virtual node to implement formation control
             %% Algorithm
            % switch to circular pursuit once the formation is complete.
            if t==iterations
                rotate=1;
            end  
             
             
            %This section contains the actual algorithm for formation control!

            %Calculate single integrator control inputs using edge-energy consensus
            for i = 1:N
        
                % Initialize velocity to zero for each agent.  This allows us to sum
                % over agent i's neighbors
                dx(:, i) = [0 ; 0];
        
             % Get the topological neighbors of agent i from the communication
                % topology
                for j = topological_neighbors(L, i)
                
                    % For each neighbor, calculate appropriate formation control term and
                    % add it to the total velocity

                         dx(:, i) = dx(:, i) + ...
                          formationControlGain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
                          *(x(1:2, j) - x(1:2, i));

                end 
            end
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
            x=x(:,1:N); %remove the virtual node before sending velocities.
            dx = si_barrier_cert(dx, x);
            dx = si_to_uni_dyn(dx, x);  
    
            % Set velocities of agents 1:N
             r.set_velocities(1:N, dx);
    
            % Send the previously set velocities to the agents.  This function must be called!
            r.step();   
        end
    case 1
        L=LC; %directed cyclic graph for pursuit
        for t = 0:3*iterations
    
            % Retrieve the most recent poses from the Robotarium.  The time delay is
            % approximately 0.033 seconds
            x = r.get_poses();
    
             %% Algorithm
    
            %This section contains the actual algorithm for formation control!

            %Calculate single integrator control inputs using edge-energy consensus
            for i = 1:5
        
                % Initialize velocity to zero for each agent.  This allows us to sum
                % over agent i's neighbors
                dx(:, i) = [0 ; 0];
        
             % Get the topological neighbors of agent i from the communication
                % topology
                for j = topological_neighbors(L, i)
                alpha = pi/N + kp1*(interAgentDistance - norm(x(1:2,j)-x(1:2,i)) );
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                dx(1:2,i) = dx(1:2,i) + R*( x(1:2,j)-x(1:2,i) ) - kp2*( x(1:2,i) - center );
                end 
            end
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
            dx = si_barrier_cert(dx, x);
            dx = si_to_uni_dyn(dx, x);  
    
            % Set velocities of agents 1:N
             r.set_velocities(1:N, dx);
    
            % Send the previously set velocities to the agents.  This function must be called!
            r.step();   
        end
%         game=0;
end
end
% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
