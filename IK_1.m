% Author ~ Ahmed Magd Aly
% Innopolis University

function [q1,q2] = IK(x, y, z, motor_axis)
% motor_axis = "X", "Y", "Z"
% Robot parameters:
l1 = 1;
l2 = 1;
tr_link = 0;
d_max = 1;

if motor_axis == "X"
    tri_angle = -60;
elseif motor_axis == "Y"
    tri_angle = 60; % in degrees
elseif motor_axis == "Z"
    tri_angle = 180;
end

tri_angle = deg2rad(tri_angle);
%% Model Kinematics:
end_effector_joint = Tx(x)*Ty(y)*Tz(z)*Rz(tri_angle)*Ty(tr_link);
end_effector_joint_coordinates = end_effector_joint(:,4);

if motor_axis == "X"
    tri_x = -d_max + end_effector_joint_coordinates(2);
    tri_y = end_effector_joint_coordinates(3);
elseif motor_axis == "Y"
    tri_x = end_effector_joint_coordinates(1);
    tri_y = -d_max + end_effector_joint_coordinates(3);
elseif motor_axis == "Z"
    tri_x = end_effector_joint_coordinates(2);
    tri_y = -end_effector_joint_coordinates(1);
end

% Solving for the angle of the motor:
alpha = acos( (tri_x^2 + tri_y^2 + l1^2 - l2^2)/(2*l1*sqrt(tri_x^2 + tri_y^2)) );
beta = acos( (-tri_x^2 - tri_y^2 + l1^2 + l2^2)/(2*l1*l2) );
gamma = atan2(tri_y, tri_x);

q1 = gamma - alpha;
q2 = pi - beta;

%% Output:
% q = [q1; q2];
