%% Formation control utilizing edge tension energy with a static, undirected
%communication topology
%Paul Glotfelter 
%3/24/2016

%% Rabeya Jamshad
%11/23/2018
% Formation control seems to be working. 
% Circular pursuit works with a specific gain.
% the formation control does have one very strange behavior with 1/3
% collapsing on to each other. need to know why
% separate mode added before circular pursuit
close all;
%% Setup Robotarium object

% Get Robotarium object used to communicate with the robots/simulator
% Get Robotarium object used to communicate with the robots/simulator
% rb = RobotariumBuilder();
N = 10; 
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm


% Video Stuff
videoFLag = 0;
% Initialize video
if videoFLag 
    vid = VideoWriter('HW4_formaionControl.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end
  

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;%2000

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
LF=[LFDef zeros(7);zeros(7) LFDef];
%formation geometry
weightsDef=[0 d d dd ldf ddh lds;
    d 0 dd d sdf ddh sds;
    d dd 0 d ldf ddh lds;
    dd d d 0 sdf ddh sds;
    ldf sdf ldf sdf 0 dfs dfsev;
    ddh ddh ddh ddh dfs 0 dfs+dfsev;
    lds sds lds sds dfsev dfs+dfsev 0];
nullMatrix=zeros(7);
weights=[weightsDef nullMatrix;
    nullMatrix weightsDef];

% cycle graph for cyclic pursuit
LCDef=[1 0 -1 0 0 0 0;
    -1 1 0 0 0 0 0;
    0 0 1 -1 0 0 0;
    0 0 0 1 -1 0 0;
    0 -1 0 0 1 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0];
LC=[LCDef zeros(7);zeros(7) zeros(7)];

kp2=0.6; %0.6 %0.2
kp1 = 3;%7 %2 %5
center=[-0.9;0];
center2=[1.1;0];
radius = 1.1;%0.7;%0.4%0.6
interAgentDistance = radius*2*sin(pi/N);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N+4); %14 nodes in total=2 virtual nodes each team

% %% defining a boundary
%             xb=[1.6 - 0.05*ones((size(ry))) rx rx -1.6 + 0.05*ones((size(ry)));
%                 ry 1 - 0.05*ones((size(rx))) -1 + 0.05*ones((size(rx))) ry;
%                 ones(1,108)];
%             dxb=zeros(2,108);
%             bx=[zeros(3,10) xb];
%             bdx=[dx dxb];

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);


rotate=0;
game=1;
Flag=plot(center(1),center(2),'x','markersize',16,'MarkerFaceColor',[0.62,0.49,0.15],'linewidth',3);
Flag2=plot(center2(1),center2(2),'x','markersize',16,'MarkerFaceColor',[0.22,0.29,0.15],'linewidth',3);
% Iterate for the previously specified number of iterations
while (game)
switch(rotate)
    case 0
        L=LF; %complete undirected graph for formation control
        for t = 0:iterations

            % Retrieve the most recent poses from the Robotarium.  The time delay is
            % approximately 0.033 seconds
            x = r.get_poses();
            x=[x(:,1:5) [center(1);center(2);0.23] [center(1)+dtfb+dfsev+d/2;center(2);0.23] x(:,6:10) [center2(1);center2(2);0.23] [center2(1)-dtfb-dfsev-d/2;center2(2);0.23] ]; %add the virtual node to implement formation control
             %% Algorithm
            % switch to circular pursuit once the formation is complete.
            if t==iterations
                rotate=2;
%                 game=0;
            end  
             
            %This section contains the actual algorithm for formation control!

            %Calculate single integrator control inputs using edge-energy consensus
            for i = 1:14
        
                % Initialize velocity to zero for each agent.  This allows us to sum
                % over agent i's neighbors
                dx(:, i) = [0 ; 0];
             % Get the topological neighbors of agent i from the communication
                % topology
                for j = topological_neighbors(L, i)
                
                    % For each neighbor, calculate appropriate formation control term and
                    % add it to the total velocity
                        if i~=6 && i~=7 && i~=13 && i~=14
                         dx(:, i) = dx(:, i) + ...
                          formationControlGain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
                          *(x(1:2, j) - x(1:2, i));
                        end

                end 
            end
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
            x=[x(:,1:5) x(:,8:12)]; %remove the virtual nodes before sending velocities.
            dx=[dx(:,1:5) dx(:,8:12)];

            dx = si_barrier_certificate(dx, x);
            dx = si_to_uni_dyn(dx, x);  
    
            %saturating actuator limits
            b=dx(1,:)>0.08;
            dx(1,b)=0.08;
            b=dx(1,:)<-0.08;
            dx(1,b)=-0.08;
            % Set velocities of agents 1:N
             r.set_velocities(1:N, dx);
             if videoFLag && mod(t,10)                               % Record a video frame every 10 iterations
                 writeVideo(vid, getframe(gcf)); 
             end
            % Send the previously set velocities to the agents.  This function must be called!
             r.step();   
        end
    case 1
        L=LCDef; %directed cyclic graph for pursuit
        
        for t = 0:3*iterations
    
            % Retrieve the most recent poses from the Robotarium.  The time delay is
            % approximately 0.033 seconds
            x = r.get_poses();
            dx = zeros(2, N+4); %initialise velocities to 0;
    
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
            % explorer node knows where to go kind of like having the flag
            % as a neighbour. works on the decreasing energy principle
            dx(1:2,10)=(dx(1:2,10)-(x(1:2,10)-center(1:2)))*0.5;
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
%             x=[x(:,1:5) x(:,8:12)] %remove the virtual nodes before sending velocities.
            dx=dx(:,1:10);
% cheating a little
            dxe=dx(:,10);
            dx = si_barrier_certificate(dx, x);
            ddxe=dxe-dx(:,10);
            if abs(ddxe)>0.00001
                dx(:,10)=-dx(:,10);
            end
            dx = si_to_uni_dyn(dx, x);  
            
     %saturating actuator limits
            b=dx(1,:)>0.08;
            dx(1,b)=0.08;
            b=dx(1,:)<-0.08;
            dx(1,b)=-0.08;
    
            % Set velocities of agents 1:N
             r.set_velocities(1:N, dx);
             if videoFLag && mod(t,200)                               % Record a video frame every 10 iterations
                 writeVideo(vid, getframe(gcf)); 
             end
    
            % Send the previously set velocities to the agents.  This function must be called!
            r.step();   
        end
        if videoFLag; close(vid); end
        game=0;
        
    case 2
        x = r.get_poses();
        dx(:,:)=0;          
        dx(1:2,10)=(dx(1:2,10)-(x(1:2,10)-center(1:2)))*0.5;
        
        % cheating a little
            dxe=dx(:,10);
            dx = si_barrier_certificate(dx, x);
            ddxe=dxe-dx(:,10);
            if abs(ddxe)>0.00001
                dx(:,10)=-dx(:,10);
                rotate=1;
            end
            dx = si_to_uni_dyn(dx, x);  
            
     %saturating actuator limits
            b=dx(1,:)>0.08;
            dx(1,b)=0.08;
            b=dx(1,:)<-0.08;
            dx(1,b)=-0.08;
    
            % Set velocities of agents 1:N
             r.set_velocities(1:N, dx);
             
             if videoFLag && mod(t,10)                               % Record a video frame every 10 iterations
                 writeVideo(vid, getframe(gcf)); 
             end
    
            % Send the previously set velocities to the agents.  This function must be called!
            r.step();   
        
        
end
end
% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
% r.call_at_scripts_end();
r.debug();