%Visual Servoing code

%% reset
clc;
clear all;
%% load image pointss

img_target = imread("image.png"); % static image of the goal image
img_target = rgb2gray(img_target);
img_obs = imread("image1.png"); % image loaded from the webcam, updates each tick
img_obs = rgb2gray(img_obs);
TargetPts = detectSURFFeatures(img_target).selectStrongest(100);
ObsPts = detectSURFFeatures(img_obs).selectStrongest(100);

[features1, validPoints1] = extractFeatures(img_target, TargetPts);
[features2, validPoints2] = extractFeatures(img_obs, ObsPts);

indexPairs = matchFeatures(features1, features2);

Target = validPoints1(indexPairs(:, 1)).Location;
Obs = validPoints2(indexPairs(:, 2)).Location;

figure;
showMatchedFeatures(img_target, img_obs, Target, Obs, 'montage');
%title('Matched Points between Two Images');


%% intrinsic properties
Z = 50;
f = 400;
p = 400;
Lambda = 0.1;

%%
xy = (Target-p)/f;
Obsxy = (Obs-p)/f;

%%
n = length(Target(:,1));

Lx = [];
for i=1:n
    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
    Lx = [Lx;Lxi];
end

%%
e2 = Obsxy-xy;
e = reshape(e2',[],1);
de = -e*Lambda;

%%
Lx2 = inv(Lx'*Lx)*Lx';
Vc = -Lambda*Lx2*e

%% draw stuff
% Plot the image first
figure;
imshow(img_target);
hold on;  % Hold the image for overlaying the points

% Plot the detected points as blue circles
plot(Target(:,1), Target(:,2), 'ro', 'MarkerSize', 5);  % 'bo' for blue circles
plot(Obs(:,1), Obs(:,2), 'bo', 'MarkerSize', 5);  % 'bo' for blue circles


