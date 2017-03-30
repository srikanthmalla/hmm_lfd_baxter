%% key point extraction 
function keypoints=keypoint_extraction(traj,epsilon)
keypoints=[traj(1,:)];
if size(traj,1)<4
    keypoints=traj;
    return 
end
for i=2:size(traj,1)-1
   u=traj(i,:)-keypoints(size(keypoints,1),:);
   v=traj(i+1,:)-traj(i,:);
   angle=atan2d(norm(cross(u,v)),dot(u,v));
   if (angle<epsilon)
       keypoints=[keypoints;traj(i,:)];
   end
end
plot3(traj(:,1),traj(:,2),traj(:,3),'x-b');hold on;
plot3(keypoints(:,1),keypoints(:,2),keypoints(:,3),'x-r')

end