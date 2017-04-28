function collision = checkCollision(q_node, q_obstacles)

collision = 0;
for i = 1:size(q_obstacles,1)
    if dist_3d(q_node, q_obstacles(i,:)) < 0.03
        collision = 1;
        break;
    end
end

end