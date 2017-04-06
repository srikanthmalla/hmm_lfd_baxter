clc; close all; clear all;
traj_plot=0;
%% Key Point Extraction 
numberOfDemonstartions = 5;
keypoints = cell(1,numberOfDemonstartions);

for i = 1:numberOfDemonstartions
    file_name = ['trial' num2str(i) '_fk.csv'];
    full_trajectory = csvread(file_name);
    trajectory = full_trajectory(:,1:3);
    if (traj_plot==1)   
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3));
        grid on
    end  
    hold on
    
    j = 1;
    epsilon = 10;
    timeSinceLastKeypoint = 0;
    
    for k = 2:size(trajectory,1) - 1
        point = trajectory(k,:);    
        u = (trajectory(k-1,:) - point); %vector from x_trajectory(i-1) to point;
        v = (point - trajectory(k+1,:)); %vector from point to x_trajectory(i+1);
        angle = atan2d(norm(cross(u,v)),dot(u,v)); %angle between u and v

        if angle > epsilon && timeSinceLastKeypoint > 80
            keypoints{i}(j,:) = point;
            j = j + 1;
            timeSinceLastKeypoint = 0;
        end

        timeSinceLastKeypoint = timeSinceLastKeypoint + 1;
    end
    if (traj_plot==1)   
        plot3(keypoints{i}(:,1),keypoints{i}(:,2),keypoints{i}(:,3),'m*', 'MarkerSize', 5);
    end
end

%% K Means Clustering
kmeans_keypoints = [keypoints{1};keypoints{2};keypoints{3};keypoints{4}; keypoints{5}];
[idx,C] = kmeans(kmeans_keypoints,20);
plot3(C(:,1),C(:,2),C(:,3),'kx','MarkerSize',15,'LineWidth',3);

[TRGUESS,EMITGUESS]=hmmestimate([idx],[1:size(keypoints{1}) 1:size(keypoints{2}) 1:size(keypoints{3}) 1:size(keypoints{4}) 1:size(keypoints{5})]')
x=hmmgenerate(19,TRGUESS,EMITGUESS)
plot3(C(x,1),C(x,2),C(x,3),'rx-','MarkerSize',15,'LineWidth',3);

%% Cubic Spline or DTW
% dtw
interpreted=C(x,:);
% [dist,ix,iy]=dtw(interpreted',trajectory');
% plot3(interpreted(ix,1),interpreted(ix,2),interpreted(ix,3),'rx-','MarkerSize',15,'LineWidth',3);
% Cubic Spline
hold on
yy=cscvn(interpreted')
fnplt(yy);