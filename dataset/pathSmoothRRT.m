clc; close all; clear all;
traj_plot=1;

numberOfDemonstrations = 1;
keypoints = cell(1,numberOfDemonstrations);

for i = 2%1:numberOfDemonstartions
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

xlabel('X')
ylabel('Y')
zlabel('Z')
% kmeans_keypoints = [keypoints{1};keypoints{2};keypoints{3};keypoints{4}; keypoints{5}];
% [idx,C] = kmeans(kmeans_keypoints,20);
% plot3(C(:,1),C(:,2),C(:,3),'kx','MarkerSize',15,'LineWidth',3);

[x,y,z] = sphere;

%%%%%%%%%%%%%%%%% More Obstacles
surf(0.08*x + 0.85, 0.08*y-0.35, 0.08*z+0.3)
%%%%%%%%%%%%%%%%%

%% RRT*

q_obstacles = [0.08*x(:) + 0.85, 0.08*y(:)-0.35, 0.08*z(:)+0.3];
in = inhull(keypoints{i}, q_obstacles);
%%%%%%%%%%%%%%%%% Generalize k; if k = 1, then what?
k = find(in == 1);
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%% These limits should be determined by sampling space
lower_x = 0.55;
upper_x = 0.95;
lower_y = -0.8;
upper_y = 0;
lower_z = 0.15;
upper_z = 0.4;
%%%%%%%%%%%%%%%%%

EPS = 0.01;
goalThreshold = 0.05;
numNodes = 2000;        

q_start.coord = keypoints{2}(k(1)-1,:);
plot3(q_start.coord(1), q_start.coord(2), q_start.coord(3), 'r*', 'MarkerSize', 10);
hold on
grid on
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = keypoints{2}(k(end)+1,:);
plot3(q_goal.coord(1), q_goal.coord(2), q_goal.coord(3), 'm*', 'MarkerSize', 10);
q_goal.cost = 0;

nodes(1) = q_start;
flag = 0;

for i = 1:1:numNodes
    %Sample new node
    r_x = (upper_x-lower_x).*rand(1) + lower_x;
    r_y = (upper_y-lower_y).*rand(1) + lower_y;
    r_z = (upper_z-lower_z).*rand(1) + lower_z;
    q_rand = [r_x r_y r_z];
%     plot3(q_rand(1), q_rand(2), q_rand(3), 'bx')
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if dist_3d(nodes(j).coord, q_goal.coord) < goalThreshold
            flag = 1;
            break
        end
    end
    
    if flag == 1
        break;
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    %Extend one step in q_rand's direction
    q_extend.coord = steer3d(q_rand, q_near.coord, val, EPS);
    %%%%%%%%%%
    collision = checkCollision(q_extend.coord, q_obstacles);
    
    if collision == 1
        continue;
    else
        q_new.coord = q_extend.coord;
    end
    %%%%%%%%%
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
    drawnow
    hold on
    %Cost to get to q_new = cost from q_near to q_new + cost to get to
    %q_near
    q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
    % Within a radius of r, find all existing nodes
    q_nearest = [];
    r = 0.1;
    neighbor_count = 1;
    for j = 1:1:length(nodes)
        %if distance between a node (eg. NODE 10)in node_list and q_new is less than r,
        %then push that node (NODE 10) in q_nearest
        if (dist_3d(nodes(j).coord, q_new.coord)) <= r
            q_nearest(neighbor_count).coord = nodes(j).coord;
            q_nearest(neighbor_count).cost = nodes(j).cost;
            neighbor_count = neighbor_count+1;
        end
    end
    
    % Initialize cost to currently known value
    q_min = q_near;
    C_min = q_new.cost;
    
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
    
    %Search all nodes in q_nearest
    for k = 1:1:length(q_nearest)
        %if distance from a node in q_nearest to q_new is less than the
        %distance to go from q_near to q_new, then update q_min and C_min
        %Also, check if the line from q_nearest(k) to q_new is collision
        %free
        if (q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min) && (checkLineCollision(q_nearest(k).coord, q_new.coord, q_obstacles) == 0)
            q_min = q_nearest(k);
            C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
            line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
            hold on
        end
    end
    
    % Update parent to least cost-from node
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_min.coord
            q_new.parent = j;
        end
    end
    
    % Append to nodes
    nodes = [nodes q_new];
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
i = 1;
while q_end.parent ~= 0
    start = q_end.parent;
    nodeList(i) = start;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
    i = i + 1;
end

%%
nodeList = [length(nodes), nodeList];
nodeList = fliplr(nodeList);
for i = 1:100
    idx1 = ceil(rand*10);
    idx2 = ceil(rand*10);
    if idx1 > length(nodeList) || idx2 > length(nodeList) || idx1 >= idx2 || idx1 == idx2 - 1
        continue;
    else
        lineCollision = checkLineCollision(nodes(nodeList(idx1)).coord, nodes(nodeList(idx2)).coord, q_obstacles);
        if lineCollision == 0
            nodes(nodeList(idx2)).parent = nodeList(idx1);
            nodeList(idx1+1:1:idx2-1) = [];
        end
    end
end

for idx = 1:length(nodeList)-1
    line([nodes(nodeList(idx)).coord(1), nodes(nodeList(idx+1)).coord(1)], [nodes(nodeList(idx)).coord(2), nodes(nodeList(idx+1)).coord(2)], [nodes(nodeList(idx)).coord(3), nodes(nodeList(idx+1)).coord(3)], 'Color', 'b', 'LineWidth', 5);
end

%blue line not reaching goal, stopping at goalThreshold distance
%linspace between all points in nodeList to find if tree can be smoothed
%more
%Add cubic spline to the trajectory later