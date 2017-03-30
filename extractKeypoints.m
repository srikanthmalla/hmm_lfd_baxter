full_trajectory = csvread('trial4_fk.csv');
trajectory = full_trajectory(:,1:3);
plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),'b');
grid on
hold on

%%
j = 1;
epsilon = 10;
timeSinceLastKeypoint = 0;

for i = 2:size(trajectory,1) - 1
    point = trajectory(i,:);    
    u = (trajectory(i-1,:) - point); %vector from x_trajectory(i-1) to point;
    v = (point - trajectory(i+1,:)); %vector from point to x_trajectory(i+1);
    angle = atan2d(norm(cross(u,v)),dot(u,v)); %angle between u and v
    
    if angle > epsilon && timeSinceLastKeypoint > 20
        keypoints(j,:) = point;
        j = j + 1;
        timeSinceLastKeypoint = 0;
    end
    
    timeSinceLastKeypoint = timeSinceLastKeypoint + 1;
end

plot3(keypoints(:,1),keypoints(:,2),keypoints(:,3),'k+', 'MarkerSize', 5);
