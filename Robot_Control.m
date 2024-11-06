% Combined Visual Servoing and Robot Joint Control Code with Position Control

%% Initialise ROS
rosinit('192.168.27.1'); % Initialize ROS connection

%% Create Subscribers and Services
jointStateSubscriber = rossubscriber('/ur/joint_states', 'sensor_msgs/JointState');

%% Load Target Image
img_target = imread("target1.jpg"); % Load the target image
img_target = rgb2gray(img_target); % Convert to grayscale
TargetPts = detectSURFFeatures(img_target).selectStrongest(100);
[features1, validPoints1] = extractFeatures(img_target, TargetPts);

%% Initialize USB Webcam
cam = webcam('Logi C270 HD WebCam'); % Connect to the USB webcam

%% Define Control Parameters
Lambda = 0.1; % Gain for control
f = 400; % Focal length
p = 400; % Principal point
durationSeconds = 5; % Duration for each movement

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
        Vc = -Lambda * Lx2 * e; % Control velocity

        %% Convert Velocity Command to Position Command
        currentJointState = jointStateSubscriber.LatestMessage.Position; % Current joint state
        deltaTheta = Vc * durationSeconds; % Convert velocity to position change
        targetJointState = currentJointState + deltaTheta; % Calculate target joint position

        %% Send Position Command to the Robot
        [client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
        jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

        % Define trajectory to reach the target position
        goal.Trajectory.JointNames = jointNames;
        goal.Trajectory.Header.Stamp = rostime('Now', 'system');
        goal.Trajectory.Points = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        goal.Trajectory.Points.Positions = targetJointState; % Set the target position
        goal.Trajectory.Points.Velocities = zeros(1,7); % Set the target position
        goal.Trajectory.Points.TimeFromStart = rosduration(durationSeconds); % Set movement duration

        % Send the position goal to the robot
        sendGoal(client, goal);

        % Pause for movement execution
        pause(durationSeconds + 1); % Wait until movement completes
        
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
