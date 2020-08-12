clear all;close all;clc;
ptCloud=pcread('map_large.pcd');
% load('map_200000.mat');
% ptCloud = pcdownsample(ptCloud, 'random', 0.2);
% % pcshow(ptCloud);
% save map_200000.mat ptCloud
% temp=ptCloud.Location;
% temp=temp(temp(:,1)>15,:);
% tempCloud=pointCloud(temp);
% temp=ptCloud.Location;
% temp=temp(temp(:,1)<18.5,:);
% tempCloud2=pointCloud(temp);
A = [cos(pi/6) sin(pi/6) 0 0; ...
    -sin(pi/6) cos(pi/6) 0 0; ...
            0         0  1 0; ...
            5         5 10 1];
tform1 = affine3d(A);
ptCloudTformed = pctransform(ptCloud,tform1);
% ptCloudTformed = pctransform(tempCloud,tform1);
[tform,movingReg,rmse] = icp_profiling(ptCloudTformed,ptCloud,'Extrapolate',true,'MaxIterations',50,'Tolerance', [0,0.02]);
% tform = icp_profiling(ptCloudTformed,tempCloud2,'Extrapolate',false);
disp(tform1.T);
% icp_profiling()