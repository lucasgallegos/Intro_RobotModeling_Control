clear
clc
close all

%  figure
%  pause(4)


Ai=@(theta, d, a, alpha) [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta)
    sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0                   sin(alpha)             cos(alpha)     d
    0                            0                      0     1];
M = @(t0,tf) [ 1 t0  t0^2    t0^3    t0^4    t0^5
    0    1   2*t0    3*t0^2  4*t0^3  5*t0^4
    0    0   2       6*t0    12*t0^2 20*t0^3
    1    tf  tf^2    tf^3    tf^4    tf^5
    0    1   2*tf    3*tf^2  4*tf^3  5*tf^4
    0    0   2       6*tf    12*tf^2 20*tf^3   ];
a_1 = 1.5;
a_2 = 1;
a_3 = 1;
A_1 = @(theta1) Ai(theta1, 0, a_1, 0);
A_2 = @(theta2) Ai(theta2, 0, a_2, 0);
A_3 = @(theta3) Ai(theta3, 0, a_3, 0);
A = {A_1;A_2;A_3};
J = @(theta1, theta2, theta3) [(-a_1*sin(theta1) -a_2*sin(theta1+theta2) -a_3*sin(theta1+theta2+theta3)), (-a_2*sin(theta1+theta2) -a_3*sin(theta1+theta2+theta3)), (-a_3*sin(theta1+theta2+theta3))
    ( a_1*cos(theta1) +a_2*cos(theta1+theta2) +a_3*cos(theta1+theta2+theta3)), (a_2*cos(theta1+theta2) +a_3*cos(theta1+theta2+theta3)), (a_3*cos(theta1+theta2+theta3))];
x_bc = [1 0 0 2 0 0; 
        1 0 0 2 0 0; 
        2 0 0 1 0 0; 
        2 0 0 1 0 0];
q = [pi/2;-pi/2;-pi/2];
dt = 0.01;
for i = 1:4
    for t = 0:dt:1
        a = inv(M(0,1))*x_bc(i,:)';
        xdot = [0    1   2*t    3*t^2  4*t^3  5*t^4]*a
        qdot = pinv(J(q(1),q(2),q(3)))*[xdot*rem(i,2); xdot*rem(i+1,2)];
        q = q + qdot*dt;
        T1 = A_1(q(1));
        T2 = A_1(q(1))*A_2(q(2));
        T3 = A_1(q(1))*A_2(q(2))*A_3(q(3));
        X = [0,T1(1,4),T2(1,4),T3(1,4)];
        Y = [0,T1(2,4),T2(2,4),T3(2,4)];
        Z = [0,T1(3,4),T2(3,4),T3(3,4)];
        xpoints = [1,2,2,1,1];
        ypoints= [0.5,0.5,1.5,1.5,0.5];
        zpoints = [0,0,0,0,0];
          %animated plots
%         fig = figure();
%         Mov(i) = struct('cdata',[],'colormap',[]);
        scatter3(X,Y,Z,200,'blue','filled');
        line(X,Y,Z,'color','black','linewidth',5)
        line(xpoints,ypoints,zpoints,'color', 'red', 'linewidth', 5)
        hold on
        grid on
        axis ([-1 5 0 5 0 1])
        view(-1,90)
        
%         Mov(end) = getframe(fig);
        pause(dt)
        hold off
        title('RRR Quintic trajectory')
%           myVideo = VideoWriter('RRR');
%         myVideo.FrameRate = 50;
%         open(myVideo);
%         writeVideo(myVideo, Mov);
%         close(myVideo);
       
        
    end
end

      