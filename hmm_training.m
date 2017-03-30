%hmm training primitives
clc;clear all; close all;
t=[];
traj_lengths=[1];
%initially dtw of trajectories distorts final end point by aligning
%remaining trajectory

%smoothing average
% windowWidth = 100; 
% kernel = ones(windowWidth,1) / windowWidth;
% out = filter(kernel, 1, t1);
% plot3(t1(:,1),t1(:,2),t1(:,3),'*b')
% hold on
% plot3(out(:,1),out(:,2),out(:,3),'*r')

%this will be our keypoint extractor z=sum(abs(diff(out,1)'))
%[sortedX,sortingIndices] = sort(z,'descend')
%20 keypoints indices extractor sortingIndices(1:20) but noise dominates
for i=1:2
    file_name=['trial' num2str(i) '_fk.csv'];
    t1=resample(csvread(file_name),1,1);
    % t1=t1(:,10:13)%%right arm angles 
    t=[t;t1(:,1:3)]%%end effector path right arm
    traj_lengths=[traj_lengths;traj_lengths(size(traj_lengths,1))+size(t1,1)];
    plot3(t(:,1),t(:,2),t(:,3),'*-b');
    hold on;
end
[idx,C] = kmeans(t,10);
plot3(C(:,1),C(:,2),C(:,3),'kx','MarkerSize',15,'LineWidth',3);
hold on;
plot3(t(idx==1,1),t(idx==1,2),t(idx==1,3),'r.','MarkerSize',12);
ids=[];
for i=2:size(traj_lengths,1)
    ids= idx(traj_lengths(i-1):traj_lengths(i)-1);
end
idx=idx(find(diff(idx)));
[trans, emis]=hmmestimate([idx], [idx]);
x=hmmgenerate(15,trans,emis);
out=C(x(:),:);
close all
plot3(out(:,1),out(:,2),out(:,3),'*-b')