function  [Tee_base_7_left,T_e, Tl, position] = baxter_fk(q1,q2,q3,q4,q5,q6,q7)

% [tee_base_7_right,Tr, Pr, Or] = baxter_rightarm_fk(q1,q2,q3,q4,q5,q6,q7);
[tee_base_7_left,t_e, Tl, Pl, Ol] = baxter_leftarm_fk(q1,q2,q3,q4,q5,q6,q7);

% right_position = Pr 
% right_Orientation = Or
% left_position = Pl
% left_Orientation = Ol
Tee_base_7_left=tee_base_7_left;
T_e=t_e;
Tl = Tl;
position = Pl;
% tee_base_7_right
end
        
% %% Function for Right Arm
% function [tee_base_7_right,Tr, EE_position, EE_Orientation] = baxter_rightarm_fk(q1,q2,q3,q4,q5,q6,q7)
% 
% % DH parameters = [theta, d, a, alpha]
% DH(1,:) = [q1, 0.27035, 0.069, -pi/2]; % frame 0 to 1
% DH(2,:) = [q2 + pi/2, 0, 0, pi/2]; % frame 1 to 2
% DH(3,:) = [q3, 0.3640, 0.069, -pi/2]; % frame 2 to 3
% DH(4,:) = [q4, 0, 0, pi/2]; % frame 3 to 4
% DH(5,:) = [q5, 0.371 0.01, -pi/2]; % frame 4 to 5
% DH(6,:) = [q6, 0, 0, pi/2]; % frame 5 to 6
% DH(7,:) = [q7, 0.2295, 0, 0]; % frame 6 to 7
% 
% for i = 1:7
%      T(i).right = [cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4)) DH(i,3)*cos(DH(i,1));
%         sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4)) DH(i,3)*sin(DH(i,1));
%         0  sin(DH(i,4)) cos(DH(i,4)) DH(i,2);
%         0 0 0 1];
% end
% 
% %origin_base = [0;0;0;1];
% for i = 1:1
% T(i).torso = translation(0.064,-0.258,0.121)*rpy2tran(0,0,-pi/4);
% %T(i).base = translation(0,0,0.3)*rpy2tran(0,0,0);
% end
% 
% Tr = T;
% Tee = T(1).torso*T(1).right*T(2).right*T(3).right*T(4).right*T(5).right*T(6).right*T(7).right;
% tee_base_7_right=Tee;
% EE_position = Tee(1:3,4);
% EE_Orientation = trans2rpy(Tee(1:3,1:3));
% end

%% Function for Left Arm
function [tee_base_7_left,t_e,Tl, EE_position, EE_Orientation] = baxter_leftarm_fk(q1,q2,q3,q4,q5,q6,q7)

% DH parameters = [theta, d, a, alpha]
DH(1,:) = [q1, 0.27035, 0.069, -pi/2]; % frame 0 to 1
DH(2,:) = [q2 + pi/2, 0, 0, pi/2]; % frame 1 to 2
DH(3,:) = [q3, 0.3640, 0.069, -pi/2]; % frame 2 to 3
DH(4,:) = [q4, 0, 0, pi/2]; % frame 3 to 4
DH(5,:) = [q5, 0.37429, 0.01, -pi/2]; % frame 4 to 5
DH(6,:) = [q6, 0, 0, pi/2]; % frame 5 to 6
DH(7,:) = [q7, 0.2295, 0, 0]; % frame 6 to 7

for i = 1:7
     T(i).left = [cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4)) DH(i,3)*cos(DH(i,1));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4)) DH(i,3)*sin(DH(i,1));
        0  sin(DH(i,4)) cos(DH(i,4)) DH(i,2);
        0 0 0 1];
end

for i = 1:1
T(i).torso = translation(0.064,0.258,0.121)*rpy2tran(0,0,pi/4);
T(i).base = translation(0,0,0.3)*rpy2tran(0,0,0);
end
Tl = T;
Tee = T(1).torso*T(1).left*T(2).left*T(3).left*T(4).left*T(5).left*T(6).left*T(7).left;
%Tee = T(1).left*T(2).left*T(3).left*T(4).left*T(5).left*T(6).left*T(7).left;
EE_position = Tee(1:3,4);
EE_Orientation = trans2rpy(Tee(1:3,1:3));
tee_base_7_left=Tee;
t_e= T(1).torso*T(1).left*T(2).left*T(3).left;
end

function T = translation(x,y,z)
T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end

function T = rpy2tran(r,p,y)

% r = deg2rad(a);
% p = deg2rad(b);
% y = deg2rad(c);

T = [cos(y)*cos(p) -sin(y)*cos(r)+cos(y)*sin(p)*sin(r) sin(y)*sin(r)+cos(y)*sin(p)*cos(r) 0;
    sin(y)*cos(p)  cos(y)*cos(r)+sin(y)*sin(p)*sin(r) -cos(y)*sin(r)+sin(y)*sin(p)*cos(r) 0;
       -sin(p)       cos(p)*sin(r)      cos(p)*cos(r) 0;
       0  0  0  1];
   
end

function y = deg2rad(x)
y = x*pi/180;
end

function Rxyz = trans2rpy(R)
	a = atan2(R(3,2), R(3,3));
	b = atan2(-R(3,1), sqrt(R(3,2)*R(3,2) + R(3,3)*R(3,3)));
	c = atan2(R(2,1), R(1,1));
    
%     x = a*180/pi;
%     y = b*180/pi;
%     z = c*180/pi;
    Rxyz = [a*180/pi; b*180/pi; c*180/pi];
end
%%INVERSE KINEMATICS OF BAXTER
% function ik_right_arm()
% T_0_1_inverse=inv(Tr(1).right)
% T_0_7=(tee_base_7_right)* (T_0_1_inverse)
% wrist_pos=T_0_7(1:3,4)
% end

