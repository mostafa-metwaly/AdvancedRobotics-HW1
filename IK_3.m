% Author ~ Ahmed Magd Aly
% Innopolis University

function [Q1, Q2] = IK_3(x, y, z, motor_axis)
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
    tri_y = d_max - end_effector_joint_coordinates(2)
    tri_x = end_effector_joint_coordinates(3)
elseif motor_axis == "Y"
    tri_x = end_effector_joint_coordinates(1);
    tri_y = d_max - end_effector_joint_coordinates(3);
elseif motor_axis == "Z"
    tri_x = end_effector_joint_coordinates(2);
    tri_y = end_effector_joint_coordinates(1);
end

% Solving for the angle of the motor:
alpha = acos( (tri_x^2 + tri_y^2 + l1^2 - l2^2)/(2*l1*sqrt(tri_x^2 + tri_y^2)) );
beta = acos( (-tri_x^2 - tri_y^2 + l1^2 + l2^2)/(2*l1*l2) );
gamma = atan2(tri_y, tri_x);

q1 = gamma - alpha;
q2 = pi - beta;

%% Output Qs:

T_base_z = eye(4);
T_base_y = Tz(d_max)*Rx(pi/2);
T_base_x = Ty(d_max)*Ry(pi/2)*Rz(pi);

q_motor = end_effector_joint_coordinates(1);
if motor_axis == "X"
    T1 = T_base_x*Tz(q_motor)*Rz(q1);
    T2 = T1*Tx(l1)*Rz(q2);
%     T1 = Ty(d_max)*Tx(q_motor)*Rx(q1);
%     T2 = Ty(d_max)*Tx(q_motor)*Rx(q1)*Ty(l1)*Rx(q2)*Ty(l2);
elseif motor_axis == "Y"
    T1 = T_base_y*Tz(q_motor)*Rz(q1);
    T2 = T1*Tx(l1)*Rz(q2);
%     T1 = Tz(d_max)*Ty(q_motor)*Ry(q1);
%     T2 = Tz(d_max)*Ty(q_motor)*Ry(q1)*Tx(l1)*Ry(q2)*Tx(l2);
elseif motor_axis == "Z"
    T1 = T_base_z*Tz(q_motor)*Rz(q1);
    T2 = T1*Tx(l1)*Rz(q2);
%     T1 = Tz(q_motor)*Rz(q1);
%     T2 = Tz(q_motor)*Rz(q1)*Ty(l1)*Rz(q2)*Ty(l2);
end

if motor_axis == "Y"
    T1(3,1) = -T1(3,1);
    T1(3,2) = -T1(3,2);
    T1(2,3) = -T1(2,3);
    T2(3,1) = -T2(3,1);
    T2(3,2) = -T2(3,2);
    T2(2,3) = -T2(2,3);
end
Q1 = [T1(1:3,1:3), zeros(3,3); zeros(3,3), T1(1:3,1:3)];
Q2 = [T2(1:3,1:3), zeros(3,3); zeros(3,3), T2(1:3,1:3)];
%% Output:
% q = rad2deg([q1; q2]);

