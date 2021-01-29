% *******************************************************
% Mostafa Osama Ahmed Metwally 
% Advanced Robotics Course
% Innopolis University
% 1-2021
% *******************************************************

clc;
clear all;


Square=[];
Kc_all=0;   
Kc_all1=0;   
syms E A L G Iz Iy Ip Ka
E = 7*10^10 ;
G = 2.55*10^10;
F = [100 0 0 0 0 0]';
L = 1;
D = 0.15;
R = 0.075;
A = pi*D^2/4;
Ka = 1000000;
Iy = (pi*D^4)/64;
Iz = (pi*D^4)/64;
Ip = Iz+Iy;
tr_link = 0.1;
d8e_z = [0, tr_link, 0];
d8e_y = [tr_link*cosd(30), tr_link*cosd(-120),0];
d8e_x = [tr_link*cosd(30+180), tr_link*cosd(120),0];
d8ee = [d8e_x ; d8e_y ; d8e_z];

K_11=[E*A/L 0 0 0 0 0;
  0 12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
  0 0 12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
  0 0 0 G*Ip/L 0 0;
  0 0 -6*E*Iy/L^2 0 4*E*Iy/L 0;
  0 6*E*Iz/L^2 0 0 0 4*E*Iz/L];

K_12=[-E*A/L 0 0 0 0 0;
  0 -12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
  0 0 -12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
  0 0 0 -G*Ip/L 0 0;
  0 0 6*E*Iy/L^2 0 2*E*Iy/L 0;
  0 -6*E*Iz/L^2 0 0 0 2*E*Iz/L];

K_21=transpose(K_12);

K_22=[E*A/L 0 0 0 0 0;
  0 12*E*Iz/L^3 0 0 0 -6*E*Iz/L^2;
  0 0 12*E*Iy/L^3 0 6*E*Iy/L^2 0;
  0 0 0 G*Ip/L 0 0;
  0 0 6*E*Iy/L^2 0 4*E*Iy/L 0;
  0 -6*E*Iz/L^2 0 0 0 4*E*Iz/L];


I=eye(6);

%Active Elastic Joint <1,2>:

lambda_r_12_x=[0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_r_12_y=[1 0 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];


lambda_r_12_z=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_e_12_x=[1 0 0 0 0 0];
lambda_e_12_y=[0 1 0 0 0 0];
lambda_e_12_z=[0 0 1 0 0 0];

lambda_r_12_a = { lambda_r_12_x ; lambda_r_12_y ; lambda_r_12_z }
lambda_e_12_a = { lambda_e_12_x ; lambda_e_12_y ; lambda_e_12_z }

%Passive joints <3,4>,<5,6>,<7,8>:

lambda_r_34_x=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_r_34_y=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 0 1];

lambda_r_34_z=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0];

lambda_p_34_x=[0 0 0 1 0 0];
lambda_p_34_y=[0 0 0 0 1 0];
lambda_p_34_z=[0 0 0 0 0 1];


lambda_r_34_a={lambda_r_34_x ; lambda_r_34_y ;lambda_r_34_z }
lambda_p_34_a={lambda_p_34_x ; lambda_p_34_y ;lambda_p_34_z }

K_11_3=K_11;
K_12_3=K_12;
K_21_3=K_21;
K_22_3=K_22;
K_11_5=K_11;
K_12_5=K_12;
K_21_5=K_21;
K_22_5=K_22; 

%Kinematics solving :
% syms q1_x q1_y q1_z q2_x q2_y q2_z
movement_start = 0.1;
movement_end = 0.9;
step = 0.1;
% number_of_points = fix((movement_end - movement_start)/step + 1)^3
all_deflections = [];
all_deflections_x = [];
all_deflections_y = [];
all_deflections_z = [];
counter = 0;

F_x= [100 0 0 0 0 0];
F_y= [0 100 0 0 0 0];
F_z= [0 0 100 0 0 0];
Ft = [F_x ; F_y ; F_z];

x_all = [];
y_all = [];
z_all = [];

for f = 1:3 
    F = transpose(Ft(f,:));
    
    for z = movement_start:step:movement_end
        for y = movement_start:step:movement_end
            for x = movement_start:step:movement_end
                z_all(length(z_all)+1) = z;
                y_all(length(y_all)+1) = y;
                x_all(length(x_all)+1) = x;
                counter = counter + 1;
    % x = 0.5;y = 0.5;z = 0.5;
                [Q1_x , Q2_x] = IK_3(x, y, z, "X");
                [Q1_y , Q2_y] = IK_3(x, y, z, "Y");
                [Q1_z , Q2_z] = IK_3(x, y, z, "Z");

    %rotation angles 1,2 for each joint in each chain(leg):
                Q1_a={Q1_x;Q1_y;Q1_z};
                Q2_a={Q2_x;Q2_y;Q2_z};

    %Aggregating all the constraints and links and joints:

                for i= 1:3
    % where i is refered to each axis of rotation 1=x, 2=y, 3=z

                    Q1=cell2mat(Q1_a(i));
                    K_11_3=Q1*K_11*transpose(Q1);
                    K_12_3=Q1*K_12*transpose(Q1);
                    K_21_3=Q1*K_21*transpose(Q1);
                    K_22_3=Q1*K_22*transpose(Q1);

                    Q2=cell2mat(Q2_a(i));
                    K_11_5=Q2*K_11*transpose(Q2);
                    K_12_5=Q2*K_12*transpose(Q2);
                    K_21_5=Q2*K_21*transpose(Q2);
                    K_22_5=Q2*K_22*transpose(Q2); 

                    d8e = d8ee(i,:);

                    d = [0 -d8e(3) d8e(2)
                         d8e(3) 0 -d8e(1)
                         -d8e(2) d8e(1) 0];

                    D = [eye(3) transpose(d);
                        zeros(3) eye(3)];

                    matI=[zeros(6,9*6);
                    zeros(6,1*6), I, I, zeros(6,6*6);
                    zeros(6,3*6) -I zeros(6,5*6);
                    zeros(6,4*6) -I zeros(6,4*6);
                    zeros(6,5*6) -I zeros(6,3*6);
                    zeros(6,6*6) -I zeros(6,2*6);
                    zeros(6,9*6);
                    zeros(6,7*6) I transpose(D)];
                    % 
                    %   K_links=[0 I -I 0 0 0 0 0;
                    %            0 0 0 0 0 0 0 0;
                    %            0 0 0 K_11_3 K_12_3 0 0 0 0;
                    %            0 0 0 K_21_3 K_22_3 0 0 0 0;
                    %            0 0 0 0 0 K_11_5 K_12_5 0 0;
                    %            0 0 0 0 0 K_21_5 K_22_5 0 0;
                    %            0 0 0 0 0 0 D -I;
                    %            0 0 0 0 0 0 0 0];
                    %     

                    K_links=[zeros(6,6*1) I -I zeros(6,6*6);
                           zeros(6,6*9);
                           zeros(6,6*3) K_11_3 K_12_3 zeros(6,6*4);
                           zeros(6,6*3) K_21_3 K_22_3 zeros(6,6*4);
                           zeros(6,6*5) K_11_5 K_12_5 zeros(6,6*2);
                           zeros(6,6*5) K_21_5 K_22_5 zeros(6,6*2);
                           zeros(6,6*7) D -I;
                           zeros(6,6*9)];

                    lambda_r_12=cell2mat(lambda_r_12_a(i));
                    lambda_e_12=cell2mat(lambda_e_12_a(i));
                    lambda_r_34=cell2mat(lambda_r_34_a(i));
                    lambda_p_34=cell2mat(lambda_p_34_a(i));

                    %  A = [lambda_r_12 -lambda_r_12 0 0 0 0 0 0 0;
                    %       0 0 lambda_r_34 -lambda_r_34 0 0 0 0 0;
                    %       0 0 0 0 lambda_r_34 -lambda_r_34 0 0 0;
                    %       0 0 0 0 0 0 lambda_r_34 -lambda_r_34 0
                    %       I 0 0 0 0 0 0 0 0];
                    %adding the platform:
                    %       0 0 0 0 0 0 D -I];



                    A = [lambda_r_12 -lambda_r_12 zeros(5,6*7);
                          zeros(5,6*2) lambda_r_34 -lambda_r_34 zeros(5,6*5);
                          zeros(5,6*4) lambda_r_34 -lambda_r_34 zeros(5,6*3);
                          zeros(5,6*6) lambda_r_34 -lambda_r_34 zeros(5,6*1);
                          I zeros(6,6*8)];     
                    % 
                    %   B = [I I 0 0 0 0 0 0 0;
                    %       0 0 lambda_r_34 lambda_r_34 0 0 0 0 0;
                    %       0 0 lambda_p_34 0 0 0 0 0 0;
                    %       0 0 0 lambda_p_34 0 0 0 0 0;
                    %       0 0 0 0 lambda_r_34 lambda_r_34 0 0 0;
                    %       0 0 0 0 lambda_p_34 0 0 0 0;
                    %       0 0 0 0 0 lambda_p_34 0 0 0;
                    %       0 0 0 0 0 0 0 lambda_r_34 lambda_r_34 0;
                    %       0 0 0 0 0 0 0 lambda_p_34 0 0;
                    %       0 0 0 0 0 0 lambda_p_34 0]
                    %   

                     B = [I I zeros(6,6*7);
                          zeros(5,6*2) lambda_r_34 lambda_r_34 zeros(5,6*5);
                          zeros(1,6*2) lambda_p_34 zeros(1,6*6);
                          zeros(1,6*3) lambda_p_34 zeros(1,6*5);
                          zeros(5,6*4) lambda_r_34 lambda_r_34 zeros(5,6*3);
                          zeros(1,6*4) lambda_p_34 zeros(1,6*4);
                          zeros(1,6*5) lambda_p_34 zeros(1,6*3);
                          zeros(5,6*6) lambda_r_34 lambda_r_34 zeros(5,6*1);
                          zeros(1,6*6) lambda_p_34 zeros(1,6*2);
                          zeros(1,6*7) lambda_p_34 zeros(1,6*1)];
                    %   
                    C = [lambda_e_12 zeros(1,6*8)];

                    D = [Ka*lambda_e_12 -Ka*lambda_e_12 zeros(1,6*7)];

                    ABCD=[matI,K_links;
                          zeros(26,54),A;
                          B,zeros(27,54);
                          C,D;
                          zeros(6,6*8) -I zeros(6,6*9)];

                    AA=ABCD(1:102,1:102);
                    BB=ABCD(1:102,103:108);
                    CC=ABCD(103:108,1:102);
                    DD=ABCD(103:108,103:108);



                    Kc=DD-(CC*inv(AA)*BB);
                    rank(Kc);
                    Kc_all=Kc_all+Kc;
                    end

                delta_t = inv(Kc_all)*F;
                mag_delta_t = sqrt(delta_t(1,1)^2 + delta_t(2,1)^2 + delta_t(3,1)^2);
                all_deflections(counter) = mag_delta_t;
                all_deflections_x(counter) = delta_t(1,1);
                all_deflections_y(counter) = delta_t(2,1);
                all_deflections_z(counter) = delta_t(3,1);
                Square=[];
                square = mag_delta_t;
            end
        end
    end
    figure;
    quiver3(x_all, y_all, z_all,all_deflections_x,all_deflections_y,all_deflections_z,10,'LineWidth',2)
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    figure;
    plot3c(x_all, y_all, z_all,all_deflections,'O')
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
end

