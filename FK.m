% Author ~ Ahmed Magd Aly
% Innopolis University

function [O, T0_6] = FK(qx, qy, qz, enable_plot)

% Robot parameters:
l1 = 1;
l2 = 1;
tr_link = 0; % diagonal virtual links for end effector
d_max = 1;

%% Origin Frame
O0 = eye(4); % world reference frame

%% IK from motor joints:
ik_x = qx - tr_link*cosd(30)
ik_y = qy - tr_link*sind(30)
ik_z = qz

%% First chain (X - Chain):
x_qs = IK(ik_x,ik_y,ik_z,"X")
x_qs = deg2rad(x_qs);

Ox_0 = Ty(d_max)*Tx(qx);
Ox_1 = Ox_0*Rx(x_qs(1))*Ty(l1);
Ox_2 = Ox_1*Rx(x_qs(2))*Ty(l2);
%% Second chain (Y - Chain):
y_qs = IK(ik_x,ik_y,ik_z,"Y");
y_qs = deg2rad(y_qs);

Oy_0 = Tz(d_max)*Ty(qy);
Oy_1 = Oy_0*Ry(-y_qs(1))*Tx(l1);
Oy_2 = Oy_1*Ry(-y_qs(2))*Tx(l2);

%% Third chain (Z - Chain):
z_qs = IK(ik_x,ik_y,ik_z,"Z");
z_qs = deg2rad(z_qs);

Oz_0 = Tz(qz);
Oz_1 = Oz_0*Rz(z_qs(1))*Ty(l1);
Oz_2 = Oz_1*Rz(z_qs(2))*Ty(l2);

%% Assemble:
O_all = [Ox_0, Ox_1, Ox_2];
O_all(:,:,2) = [Oy_0, Oy_1, Oy_2];
O_all(:,:,3) = [Oz_0, Oz_1, Oz_2];

global axes_plot links_plot joints_plot end_effector_plot triangle_plot


for i = 1:3
    O = O_all(:,:,i);

    if enable_plot
        % figure('units','normalized','outerposition',[0 0 1 1])



        as = 0.000150; % axes scaler
        color = ['r','g','b']; % axes color

        index = 0;
        for i = 1:4:length(O)
            index = index + 1;
            points_x(index) = O(1,i+3);
            points_y(index) = O(2,i+3);
            points_z(index) = O(3,i+3);

            if index ~= 2 && index ~= 5
                for axes = 0:2

                    axes_plot = [axes_plot plot3([O(1,i+3) as*O(1,i+axes)+O(1,i+3)], [O(2,i+3) as*O(2,i+axes)+O(2,i+3)], [O(3,i+3) as*O(3,i+axes)+O(3,i+3)],'Color',color(axes+1))];
                    hold on
                end
            end
        end
        joints_x = points_x;
        joints_y = points_y;
        joints_z = points_z;

        links_plot = [links_plot plot3(points_x, points_y, points_z,'Color', "0 0 0",'linewidth',2)];
        hold on
        joints_plot = [joints_plot plot3(joints_x, joints_y, joints_z,'.','Color','0.992 0.788 0.04 1','MarkerSize',20)];
        hold on
        end_effector_plot = [end_effector_plot plot3(joints_x(1), joints_y(1), joints_z(1),'.','Color','0.8 0 0 1','MarkerSize',20)];
%         path_plot = plot3(joints_x(6), joints_y(6), joints_z(6),'.','Color','0.8 0 0 1','MarkerSize',7.5);


        xlim([-1 1])
        ylim([-1 1])
        zlim([-1 1])
        view(90,90)
        grid on
        xlabel("X - Axis")
        ylabel("Y - Axis")
        zlabel("Z - Axis")
    end
    ee_x = [O_all(1,12,1), O_all(1,12,2), O_all(1,12,3), O_all(1,12,1)];
    ee_y = [O_all(2,12,1), O_all(2,12,2), O_all(2,12,3), O_all(2,12,1)];
    ee_z = [O_all(3,12,1), O_all(3,12,2), O_all(3,12,3), O_all(3,12,1)];
    triangle_plot = [triangle_plot fill3(ee_x, ee_y, ee_z,'red')];
end
