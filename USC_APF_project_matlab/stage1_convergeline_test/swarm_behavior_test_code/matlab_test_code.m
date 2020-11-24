%% create the environment
clc;
clear;
size = 100;
p = zeros(100,100);
p(:,1) = 1;
p(1,:) = 1;
p(:,100) = 1;
p(100,:) = 1;
p(45:55,45:55) = 1;
map = occupancyMap(p);
while exist('func', 'var') == 0
    try
        func = set2;
    end
end
%% Define several_Vehicle
numRobots = 5;
R = 1;                % Wheel radius [m]
L = 2;  % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:15;         % Time array

initPose = rand(3,numRobots).*[45;45;pi]- [L;L;0];             % Initial pose (x y theta)
pose = initPose;    % Pose matrix

% % Define waypoints
% waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];

% Create visualizer
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = L;
env.mapName = 'map';

%% field_theory Controller


%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    for j = 1:numRobots
        pre_pose(:,j) = pose(:,j);
        [vRef,wRef] = func(pose(:,j));
        [wL,wR] = inverseKinematics(dd,vRef,wRef);
        % Compute the velocities
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % Body velocities [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,j));  % Convert from body to world
        pose(:,j) = pre_pose(:,j) + vel*sampleTime; 
    end
    env(1:numRobots, pose);
    % Update visualization
    waitfor(r);
end