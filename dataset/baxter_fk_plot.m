function baxter_fk_plot(q1,q2,q3,q4,q5,q6,q7)

%clc; clear;

% DH parameters = [theta, alpha, a, d]
DH(:,1) = [q1 - pi/2, -pi/2, 0.069, 0.27035]; % frame 0 to 1
DH(:,2) = [q2 + pi/2, pi/2, 0, 0]; % frame 1 to 2
DH(:,3) = [q3, -pi/2, 0.069, 0.364]; % frame 2 to 3
DH(:,4) = [q4 + pi/2, pi/2, 0, 0]; % frame 3 to 4
DH(:,5) = [q5, -pi/2, 0.01, 0.371]; % frame 4 to 5
DH(:,6) = [q6, pi/2, 0, 0]; % frame 5 to 6
DH(:,7) = [q7, 0, 0, 0.28]; % frame 6 to 7
DH(:,8) = [0, 0, 0, 1]; % from ground to base frame
DH(:,9) = [0, 0, 0.4, 0]; % from base to frame 0

DH(:,10) = [-q1 - pi/2, -pi/2, 0.069, 0.27035]; % frame 0 to 1
DH(:,11) = [q2 + pi/2, pi/2, 0, 0]; % frame 1 to 2
DH(:,12) = [q3, -pi/2, 0.069, 0.364]; % frame 2 to 3
DH(:,13) = [q4 + pi/2, pi/2, 0, 0]; % frame 3 to 4
DH(:,14) = [q5, -pi/2, 0.01, 0.371]; % frame 4 to 5
DH(:,15) = [q6, pi/2, 0, 0]; % frame 5 to 6
DH(:,16) = [q7, 0, 0, 0.28]; % frame 6 to 7
DH(:,17) = [0, 0, 0, 1]; % from ground to base frame
DH(:,18) = [0, 0, -0.4, 0]; % from base to frame 0

for i = 1:9
     T(:,:,i) = [cos(DH(1,i)) -sin(DH(1,i))*cos(DH(2,i)) sin(DH(1,i))*sin(DH(2,i)) DH(3,i)*cos(DH(1,i));
        sin(DH(1,i)) cos(DH(1,i))*cos(DH(2,i)) -cos(DH(1,i))*sin(DH(2,i)) DH(3,i)*sin(DH(1,i));
        0  sin(DH(2,i)) cos(DH(2,i)) DH(4,i);
        0 0 0 1];
end

P0 = [0;0;0;1];
P1 = T(:,:,8)*P0;
P2 = T(:,:,8)*T(:,:,9)*P0;
P3 = T(:,:,8)*T(:,:,9)*T(:,:,1)*P0;
P4 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*[0;0;0.102;1];
P5 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*T(:,:,3)*P0;
P6 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*[0;0;0.103;1];
P7 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*P0;
P8 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*P0;
P9 = T(:,:,8)*T(:,:,9)*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*P0

% %P10 = [0;0;0;1];
% P11 = T(:,:,8)*P0;
% P12 = T(:,:,8)*T(:,:,18)*P0;
% P13 = T(:,:,8)*T(:,:,18)*T(:,:,10)*P0;
% P14 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*[0;0;0.102;1];
% P15 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*T(:,:,12)*P0;
% P16 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*T(:,:,12)*T(:,:,13)*[0;0;0.103;1];
% P17 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*T(:,:,12)*T(:,:,13)*T(:,:,14)*P0;
% P18 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*T(:,:,12)*T(:,:,13)*T(:,:,14)*T(:,:,15)*P0;
% P19 = T(:,:,8)*T(:,:,18)*T(:,:,10)*T(:,:,11)*T(:,:,12)*T(:,:,13)*T(:,:,14)*T(:,:,15)*T(:,:,16)*P0;

plot3([P0(1),P1(1),P2(1),P3(1),P4(1),P5(1),P6(1),P7(1),P8(1),P9(1)], [P0(2),P1(2),P2(2),P3(2),P4(2),P5(2),P6(2),P7(2),P8(2),P9(2)], [P0(3),P1(3),P2(3),P3(3),P4(3),P5(3),P6(3),P7(3),P8(3),P9(3)], 'Marker', 'o', 'MarkerFaceColor', 'r');
grid on
% hold on
% plot3([P11(1),P12(1),P13(1),P14(1),P15(1),P16(1),P17(1),P18(1),P19(1)], [P11(2),P12(2),P13(2),P14(2),P15(2),P16(2),P17(2),P18(2),P19(2)], [P11(3),P12(3),P13(3),P14(3),P15(3),P16(3),P17(3),P18(3),P19(3)], 'Marker', 'o', 'MarkerFaceColor', 'b');
axis([-2 2 -2 2 0 2]) 
title('Baxter Robot Manipulators')
xlabel('x axis (m)') %label of the x axis
ylabel('y axis (m)') %label of the y axis
zlabel('z axis (m)') %label of the z axis 
drawnow

