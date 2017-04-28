clear
clc
close all
tic
x_max = 640;
y_max = 480;
z_max = 400;

EPS = 30;
numNodes = 2000;        

% q_start.coord = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
q_start.coord = [0 0 0];
plot3(q_start.coord(1), q_start.coord(2), q_start.coord(3), 'r*', 'MarkerSize', 5);
hold on
grid on
q_start.cost = 0;
q_start.parent = 0;
% q_goal.coord = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
q_goal.coord = [640 480 400];
plot3(q_goal.coord(1), q_goal.coord(2), q_goal.coord(3), 'm*', 'MarkerSize', 10);
q_goal.cost = 0;
q_goal.parent = 1000;

%%%%%%%%%%%%%%%
%%% DEFINE OBSTACLES %%%
%%%%%%%%%%%%%%%
[x,y,z] = sphere;
surf(50*x+320,50*y+240,50*z+200)
hold on
surf(50*x+100,50*y+80,50*z+350)
surf(50*x+500,50*y+400,50*z+100)
surf(50*x+100,50*y+400,50*z+100)
axis([0 640 0 480 0 400])
%
obstacle1 = [50*x(:)+320,50*y(:)+240,50*z(:)+200];
obstacle2 = [50*x(:)+100,50*y(:)+80,50*z(:)+350];
obstacle3 = [50*x(:)+500,50*y(:)+400,50*z(:)+100];
obstacle4 = [50*x(:)+100,50*y(:)+400,50*z(:)+100];
q_obstacles = [obstacle1; obstacle2; obstacle3; obstacle4];
%%%%%%%%%%%
%%%%%%%%%%%

figure(1)
flag = 0;

tree1(1) = q_start;
tree2(1000) = q_goal;

lastFilled = 1000;
for i = 1:1:numNodes
    
    %% Sample new node
    q_rand_tree1 = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    q_rand_tree2 = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    plot3(q_rand_tree1(1), q_rand_tree1(2), q_rand_tree1(3), 'x', 'Color',  [0 0.4470 0.7410])
    plot3(q_rand_tree2(1), q_rand_tree2(2), q_rand_tree2(3), 'x', 'Color',  [0 0.4470 0.7410])
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Break if goal node is already reached
    for j = 1:1:length(tree1)
        for k = length(tree2):-1:lastFilled
            if dist_3d(tree1(j).coord, tree2(k).coord) < 50
                line([tree1(j).coord(1) tree2(k).coord(1)], [tree1(j).coord(2) tree2(k).coord(2)], [tree1(j).coord(3) tree2(k).coord(3)], 'Color', 'r')
                flag = 1;
                break
            end
        end
    end
    
    if flag == 1
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Pick the closest node from existing list to branch out from
    %Tree 1 nearest neighbour
    ndist_tree1 = [];
    for k = 1:1:length(tree1)
        n = tree1(k);
        tmp = dist_3d(n.coord, q_rand_tree1);
        ndist_tree1 = [ndist_tree1 tmp];
    end
    [val_tree1, idx_tree1] = min(ndist_tree1);
    q_near_tree1 = tree1(idx_tree1);
    
    %Tree 2 nearest neighbour
    for l = length(tree2):-1:1
        %start from end of the tree, see which member of struct is empty, make that
        %lastFilled; If any struct member is Inf (iterating from end), then break. This
        %will reduce the number of iterations of this for loop
        if isempty(tree2(l).coord) == 1
            lastFilled = l + 1;
            break;
        end
    end
    
    ndist_tree2 = Inf*ones(1000,1);
    for m = length(tree2):-1:lastFilled
        n = tree2(m);
        tmp = dist_3d(n.coord, q_rand_tree2);
        ndist_tree2(m) = tmp;
    end
    [val_tree2, idx_tree2] = min(ndist_tree2);
    q_near_tree2 = tree2(idx_tree2);
    
    %% Extend one step in q_rand's direction
    q_extend_tree1.coord = steer3d(q_rand_tree1, q_near_tree1.coord, val_tree1, EPS);
    q_extend_tree2.coord = steer3d(q_rand_tree2, q_near_tree2.coord, val_tree2, EPS);

    %%%%%%%%%%
    collision_tree1 = checkCollision(q_extend_tree1.coord, q_obstacles);
    if collision_tree1 == 1
        continue;
    else
        q_new_tree1.coord = q_extend_tree1.coord;
    end
    
    collision_tree2 = checkCollision(q_extend_tree2.coord, q_obstacles);
    if collision_tree2 == 1
        continue;
    else
        q_new_tree2.coord = q_extend_tree2.coord;
    end
    %%%%%%%%%
    line([q_near_tree1.coord(1), q_new_tree1.coord(1)], [q_near_tree1.coord(2), q_new_tree1.coord(2)], [q_near_tree1.coord(3), q_new_tree1.coord(3)], 'Color', 'k', 'LineWidth', 2);
    line([q_near_tree2.coord(1), q_new_tree2.coord(1)], [q_near_tree2.coord(2), q_new_tree2.coord(2)], [q_near_tree2.coord(3), q_new_tree2.coord(3)], 'Color', 'b', 'LineWidth', 2);
    drawnow
    hold on
    %Cost to get to q_new = cost from q_near to q_new + cost to get to
    %q_near
    q_new_tree1.cost = dist_3d(q_new_tree1.coord, q_near_tree1.coord) + q_near_tree1.cost;
    q_new_tree2.cost = dist_3d(q_new_tree2.coord, q_near_tree2.coord) + q_near_tree2.cost;

    % Within a radius of r, find all existing nodes
    q_nearest_tree1 = [];
    r = 100;
    neighbor_count_tree1 = 1;
    for j = 1:1:length(tree1)
        %if distance between a node (eg. NODE 10)in node_list and q_new is less than r,
        %then push that node (NODE 10) in q_nearest
        if (dist_3d(tree1(j).coord, q_new_tree1.coord)) <= r
            q_nearest_tree1(neighbor_count_tree1).coord = tree1(j).coord;
            q_nearest_tree1(neighbor_count_tree1).cost = tree1(j).cost;
            neighbor_count_tree1 = neighbor_count_tree1+1;
        end
    end
    
    q_nearest_tree2 = [];
    r = 100;
    neighbor_count_tree2 = length(tree2);
    for j = length(tree2):-1:lastFilled
        %if distance between a node (eg. NODE 10)in node_list and q_new is less than r,
        %then push that node (NODE 10) in q_nearest
        if (dist_3d(tree2(j).coord, q_new_tree2.coord)) <= r
            q_nearest_tree2(neighbor_count_tree2).coord = tree2(j).coord;
            q_nearest_tree2(neighbor_count_tree2).cost = tree2(j).cost;
            neighbor_count_tree2 = neighbor_count_tree2-1;
        end
    end
    
    % Initialize cost to currently known value
    q_min_tree1 = q_near_tree1;
    q_min_tree2 = q_near_tree2;    
    C_min_tree1 = q_new_tree1.cost;
    C_min_tree2 = q_new_tree2.cost;
    
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
    
    %Search all nodes in q_nearest
    for k = 1:1:length(q_nearest_tree1)
        %if distance from a node in q_nearest to q_new is less than the
        %distance to go from q_near to q_new, then update q_min and C_min
        %Also, check if the line from q_nearest(k) to q_new is collision
        %free
        if (q_nearest_tree1(k).cost + dist_3d(q_nearest_tree1(k).coord, q_new_tree1.coord) < C_min_tree1) && (checkLineCollision(q_nearest_tree1(k).coord, q_new_tree1.coord, q_obstacles) == 0)
            q_min_tree1 = q_nearest_tree1(k);
            C_min_tree1 = q_nearest_tree1(k).cost + dist_3d(q_nearest_tree1(k).coord, q_new_tree1.coord);
            line([q_min_tree1.coord(1), q_new_tree1.coord(1)], [q_min_tree1.coord(2), q_new_tree1.coord(2)], [q_min_tree1.coord(3), q_new_tree1.coord(3)], 'Color', 'g');            
            hold on
        end
    end
    
    for k = length(q_nearest_tree2):-1:neighbor_count_tree2+1
        %if distance from a node in q_nearest to q_new is less than the
        %distance to go from q_near to q_new, then update q_min and C_min
        %Also, check if the line from q_nearest(k) to q_new is collision
        %free
        if (q_nearest_tree2(k).cost + dist_3d(q_nearest_tree2(k).coord, q_new_tree2.coord) < C_min_tree2) && (checkLineCollision(q_nearest_tree2(k).coord, q_new_tree2.coord, q_obstacles) == 0)
            q_min_tree2 = q_nearest_tree2(k);
            C_min_tree2 = q_nearest_tree2(k).cost + dist_3d(q_nearest_tree2(k).coord, q_new_tree2.coord);
            line([q_min_tree2.coord(1), q_new_tree2.coord(1)], [q_min_tree2.coord(2), q_new_tree2.coord(2)], [q_min_tree2.coord(3), q_new_tree2.coord(3)], 'Color', 'g');            
            hold on
        end
    end
    
    % Update parent to least cost-from node
    for j = 1:1:length(tree1)
        if tree1(j).coord == q_min_tree1.coord
            q_new_tree1.parent = j;
        end
    end
    
    for j = length(tree2):-1:lastFilled
        if tree2(j).coord == q_min_tree2.coord
            q_new_tree2.parent = j;
        end
    end
    
    % Append to nodes
    tree1 = [tree1 q_new_tree1];
    tree2(lastFilled-1) = q_new_tree2;
end
    
%%
% Search backwards from goal to start to find the optimal least cost path
q_goal = tree2(k);
q_final = tree1(j);
q_goal.parent = j;
q_end = q_goal;
tree1 = [tree1 q_goal];
i = 1;
start = k;
while q_end.parent ~= 0
    nodeList(i) = start;
    start = q_end.parent;
%     line([q_end.coord(1), tree1(start).coord(1)], [q_end.coord(2), tree1(start).coord(2)], [q_end.coord(3), tree1(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
%     hold on
    q_end = tree1(start);
    i = i+1;
end

ptr = tree2(nodeList(1)).parent;
while ptr ~= 1000
    nodeList = [ptr nodeList];
    ptr = tree2(nodeList(1)).parent;
end

%%
nodeList = fliplr(nodeList);
nodeList = [1 nodeList length(tree2)];

for idx = 1:length(nodeList)-1
    if nodeList(idx) <= length(tree1) && nodeList(idx+1) <= length(tree1)
        line([tree1(nodeList(idx)).coord(1), tree1(nodeList(idx+1)).coord(1)], [tree1(nodeList(idx)).coord(2), tree1(nodeList(idx+1)).coord(2)], [tree1(nodeList(idx)).coord(3), tree1(nodeList(idx+1)).coord(3)], 'Color', 'r', 'LineWidth', 5);
    elseif nodeList(idx) <= length(tree1) && nodeList(idx+1) > length(tree1)
        line([tree1(nodeList(idx)).coord(1), tree2(nodeList(idx+1)).coord(1)], [tree1(nodeList(idx)).coord(2), tree2(nodeList(idx+1)).coord(2)], [tree1(nodeList(idx)).coord(3), tree2(nodeList(idx+1)).coord(3)], 'Color', 'r', 'LineWidth', 5);
    else
        line([tree2(nodeList(idx)).coord(1), tree2(nodeList(idx+1)).coord(1)], [tree2(nodeList(idx)).coord(2), tree2(nodeList(idx+1)).coord(2)], [tree2(nodeList(idx)).coord(3), tree2(nodeList(idx+1)).coord(3)], 'Color', 'r', 'LineWidth', 5);
    end
end
toc