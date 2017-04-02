numberOfDemonstartions = 5;
keypoints = cell(1,numberOfDemonstartions);

for i = 1:numberOfDemonstartions
    file_name = ['trial' num2str(i) '_fk.csv'];
    full_trajectory = csvread(file_name);
    trajectory = full_trajectory(:,1:3);
    plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3));
    grid on
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
    
    plot3(keypoints{i}(:,1),keypoints{i}(:,2),keypoints{i}(:,3),'m*', 'MarkerSize', 5);

end

%%
kmeans_keypoints = [keypoints{1}; keypoints{2}; keypoints{3}; keypoints{4}; keypoints{5}];
[idx,C] = kmeans(kmeans_keypoints,10);
plot3(C(:,1),C(:,2),C(:,3),'kx','MarkerSize',15,'LineWidth',3);
% % ids=[];
% % for i=2:size(trajectory,1)
% %     ids= idx(traj_lengths(i-1):traj_lengths(i)-1);
% % end
% % idx = idx(find(diff(idx)));
% [trans, emis]=hmmestimate([idx], [idx]);
% x = hmmgenerate(1500,trans,emis);
% % x = x(find(diff(x)))
% out=C(x(:),:);
% % close all
% hold on
% plot3(out(:,1),out(:,2),out(:,3),'*-b')