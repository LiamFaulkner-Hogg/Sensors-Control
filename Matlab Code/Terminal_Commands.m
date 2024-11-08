%% reset
clc;
clear all;

%% Video feed setup
% Initialize webcam
cam = webcam;  % Create a connection to your webcam

% Read the static target image once
img_target = imread("target1.jpg");  % Static image of the goal image
img_target = rgb2gray(img_target);
TargetPts = detectSURFFeatures(img_target).selectStrongest(100);
[features1, validPoints1] = extractFeatures(img_target, TargetPts);

%% intrinsic properties (set up constants)
Z = 50;
f = 400;
p = 400;
Lambda = 0.1;

%% Loop to continuously process each frame
while true
    % Capture a frame from the webcam
    img_obs = snapshot(cam);
    img_obs = rgb2gray(img_obs);

    % Detect and extract features from the observed image
    ObsPts = detectSURFFeatures(img_obs).selectStrongest(100);
    [features2, validPoints2] = extractFeatures(img_obs, ObsPts);

    % Match features between the target image and observed image
    indexPairs = matchFeatures(features1, features2);
    if isempty(indexPairs)
        %disp('No matches found.');
        continue;
    end

    % Extract matched points
    Target = validPoints1(indexPairs(:, 1)).Location;
    Obs = validPoints2(indexPairs(:, 2)).Location;

    %% Process image points for servoing
    xy = (Target - p) / f;
    Obsxy = (Obs - p) / f;

    n = length(Target(:, 1));
    Lx = [];
    for i = 1:n
        Lxi = FuncLx(xy(i, 1), xy(i, 2), Z);
        Lx = [Lx; Lxi];
    end

    % Compute error
    e2 = Obsxy - xy;
    e = reshape(e2', [], 1);
    de = -e * Lambda;

    % Compute control velocity
    Lx2 = inv(Lx' * Lx) * Lx';
    Vc = -Lambda * Lx2 * e;

    % Display results
    figure(1);
    showMatchedFeatures(img_target, img_obs, Target, Obs, 'montage');
    drawnow;

    % Display the velocity command
    disp('Velocity Command (Vc):');
    disp(Vc);  % Display the entire velocity command vector
    
    % Alternatively, you can format the output for better readability
    fprintf('Velocity Command: Vx = %.3f, Vy = %.3f, Vz = %.3f\n', Vc(1), Vc(2), Vc(3));


    %% Draw detected points on the observed image
    %figure(2);
    %imshow(img_obs);
    %hold on;
    %plot(Target(:, 1), Target(:, 2), 'ro', 'MarkerSize', 5);  % Target points (red)
    %plot(Obs(:, 1), Obs(:, 2), 'bo', 'MarkerSize', 5);  % Observed points (blue)
    %hold off;

    % Add a delay to prevent overload (adjust as necessary)
    pause(0.1);
end

%% Clean up
clear cam;



