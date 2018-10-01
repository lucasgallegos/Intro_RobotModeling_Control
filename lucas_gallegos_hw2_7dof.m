% Lucas Gallegos
% Intro to Robot Modeling and Control
% HW2
% 7 DOF Robot

clear
clc
close all

n_steps = 100;
q_theta = linspace(3*pi/2,2*pi,n_steps);
L1 = 5;
L2 = 5;

dofs = 7+1; %There are only 7 dofs, the +1 is for the end effector transformation matrix

for n = 1:n_steps %This loops through incremented theta values
    theta = [q_theta(n) (q_theta(n)-(pi/2)) q_theta(n)-(pi/2) q_theta(n)+pi/2 q_theta(n) q_theta(n)-(pi/2) q_theta(n) 0];

    d = [0 0 0 L1 L2 0 0 1];

    a = [0 0 0 0 0 0 0 0];

    alpha = [-pi/2 -pi/2 pi/2 pi/2 -pi/2 -pi/2 0 0];

    A_i = [];
    A = [];

    for i = 1:dofs %This gets an A matrix for each DOF

        A_i = [cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
            sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(alpha(i))*sin(alpha(i)) a(i)*sin(theta(i));
            0 sin(alpha(i)) cos(alpha(i)) d(i);
            0 0 0 1];
        A = [A; A_i];
    end

    A1 = A(1:4,[1 2 3 4]);
    A2 = A(5:8,[1 2 3 4]);
    A3 = A(9:12,[1 2 3 4]);
    A4 = A(13:16,[1 2 3 4]);
    A5 = A(17:20,[1 2 3 4]);
    A6 = A(21:24,[1 2 3 4]);
    A7 = A(25:28,[1 2 3 4]);
    A8 = A(28:31,[1 2 3 4]);

    T_10 = A1; %Transformation Matrices
    T_20 = A1*A2;
    T_30 = A1*A2*A3;
    T_40 = A1*A2*A3*A4;
    T_50 = A1*A2*A3*A4*A5;
    T_60 = A1*A2*A3*A4*A5*A6;
    T_70 = A1*A2*A3*A4*A5*A6*A7;
    T_80 = A1*A2*A3*A4*A5*A6*A7*A8;

    x_points = [0;T_10(1,4);T_20(1,4);T_30(1,4);T_40(1,4);T_50(1,4);T_60(1,4);T_70(1,4)];
    y_points = [0;T_10(2,4);T_20(2,4);T_30(2,4);T_40(2,4);T_50(2,4);T_60(2,4);T_70(2,4)];
    z_points = [0;T_10(3,4);T_20(3,4);T_30(3,4);T_40(3,4);T_50(3,4);T_60(3,4);T_70(3,4)];

    x_points_ef = [T_70(1,4);T_80(1,4)];
    y_points_ef = [T_70(2,4);T_80(2,4)];
    z_points_ef = [T_70(3,4);T_80(3,4)];
    
    %animated plots
%     fig = figure();
%     Mov(n) = struct('cdata',[],'colormap',[]);
    scatter3(x_points,y_points,z_points, 100, 'blue', 'filled')
    line(x_points,y_points,z_points,'color', 'black', 'linewidth', 2)
    line(x_points_ef,y_points_ef,z_points_ef,'color', 'red', 'linewidth', 5)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xmin = -6; ymin = -6; zmin=-10;
    xmax = 6; ymax = 6; zmax = 6;
    axis([xmin xmax ymin ymax zmin zmax])
    title('7 DOF Human Arm Actuator')
%     Mov(end) = getframe(fig);
    drawnow
    
%     myVideo = VideoWriter('7 DOF Robot');
%     myVideo.FrameRate = 15;
%     open(myVideo);
%     writeVideo(myVideo, Mov);
%     close(myVideo);

end