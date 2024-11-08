% Combined Visual Servoing and Robot Joint Control Code with Position Control

%% Initialise ROS
robot = UR3();

rosinit('192.168.27.1'); % Initialize ROS connection

%% Create Subscribers and Services
jointStateSubscriber = rossubscriber('/ur/joint_states', 'sensor_msgs/JointState');

%%
%currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
%currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% jointStateSubscriber.LatestMessage   - bugfix

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};


%% Init joint trajectory time

[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.25);
bufferSeconds = 2; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 0.1; % This is how many seconds the movement will take

%% Load Target Image
img_target = imread("target1.jpg"); % Load the target image
img_target = rgb2gray(img_target); % Convert to grayscale
TargetPts = detectSURFFeatures(img_target).selectStrongest(100);
[features1, validPoints1] = extractFeatures(img_target, TargetPts);

%% Initialize USB Webcam
cam = webcam('C270 HD WEBCAM'); % Connect to the USB webcam

%% Define Control Parameters
Lambda = 0.1; % Gain for control
f = 400; % Focal length
p = 400; % Principal point

%% Main Loop
for i = 1:1000 % Limit to 1000 iterations to avoid infinite loop
    try
        % Capture image from the webcam
        img_obs = snapshot(cam); % Capture a frame from the webcam
        imshow(img_obs)
        img_obs = rgb2gray(img_obs); % Convert the image to grayscale
        ObsPts = detectSURFFeatures(img_obs).selectStrongest(100);
        [features2, validPoints2] = extractFeatures(img_obs, ObsPts);
        indexPairs = matchFeatures(features1, features2);

        Target = validPoints1(indexPairs(:, 1)).Location;
        Obs = validPoints2(indexPairs(:, 2)).Location;

        %% Calculate Control Command for 2D Plane (No Depth)
        xy = (Target - p) / f; % Normalized target points
        Obsxy = (Obs - p) / f; % Normalized observed points
        n = length(Target(:, 1));

        Lx = [];
        for j = 1:n
            Lxi = FuncLx2D(xy(j, 1), xy(j, 2));
            Lx = [Lx; Lxi];
        end

        e2 = Obsxy - xy; % Error calculation
        e = reshape(e2', [], 1);
        de = -e * Lambda;

        Lx2 = inv(Lx' * Lx) * Lx'; % Pseudo-inverse of Jacobian
        Vc = -Lambda * Lx2 * de; % Control velocity

        % for i = 1:length(Vc)
        %     if Vc(i) > 0.1
        %         Vc(i) = 0.1;
        %     end
        % 
        %     if Vc(i) < -0.1
        %         Vc(i) = -0.1;
        %     end 
        % end
        
        %% Send Position Command to the Robot

        currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
        currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

        
        JCurrent = robot.model.jacob0(currentJointState_123456);
        jointVelocities = pinv(JCurrent) * Vc;
        next_joint_angles = currentJointState_123456;
        next_joint_angles = next_joint_angles + jointVelocities * durationSeconds;




        startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        startJointSend.Positions = currentJointState_123456; % Define the start pose as the current pose
        startJointSend.TimeFromStart = rosduration(0);     
              
        endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        
        % Edit this line to select the destination pose
        nextJointState_123456 = next_joint_angles; % Define end pose as configuration of joint values
        
        endJointSend.Positions = nextJointState_123456;
        endJointSend.TimeFromStart = rosduration(durationSeconds);
        
        
        goal.Trajectory.Points = [startJointSend; endJointSend];
        goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
        cancelGoal(client);
        sendGoal(client,goal);
        pause(7);
        %Pause is important - this gives time for the robot to execute the previous move command. No pause will cause the robot to 'spasm'
        pause(durationSeconds); 
        
    catch ME
        fprintf('Error capturing frame %d: %s\n', i, ME.message);
        break; % Exit the loop on error
    end
end

% Cleanup
clear cam; % Clear the webcam object
rosshutdown(); % Shutdown ROS when done

% Modified FuncLx2D function for 2D Plane Control
function [Lx] = FuncLx2D(x, y)
    Lx = zeros(2, 6);
    Lx(1, 1) = -1;
    Lx(1, 2) = 0;
    Lx(1, 3) = x;
    Lx(1, 4) = x * y;
    Lx(1, 5) = -(1 + x^2);
    Lx(1, 6) = y;
    Lx(2, 1) = 0;
    Lx(2, 2) = -1;
    Lx(2, 3) = y;
    Lx(2, 4) = 1 + y^2;
    Lx(2, 5) = -x * y;
    Lx(2, 6) = -x;
end
