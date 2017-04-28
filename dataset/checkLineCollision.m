function collision = checkLineCollision(q_nearest, q_node, q_obstacles)

x_lin = linspace(q_nearest(1), q_node(1), 30);
y_lin = linspace(q_nearest(2), q_node(2), 30);
z_lin = linspace(q_nearest(3), q_node(3), 30);
xyz = [x_lin', y_lin', z_lin'];
in = inhull(xyz, q_obstacles);

k = find(in == 1);

if isempty(k)
    collision = 0;
else
    collision = 1;
end

end