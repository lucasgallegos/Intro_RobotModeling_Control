clear
clc
close all

% figure
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
    
%specify constant DH Parameters    
theta3 = 0;
a_1 = 0;
a_2 = 0;
a_3 = 0;
alpha1 = pi/2;
alpha2 = pi/2;
alpha3 = 0;
d1 = 3;
d2 = 0;
A_1 = @(theta1) Ai(theta1, d1, a_1, alpha1);
A_2 = @(theta2) Ai(theta2+(pi/2), d2, a_2, alpha2);
A_3 = @(d3) Ai(theta3, d3, a_3, alpha3);

A = {A_1;A_2;A_3};

J = @(theta1, theta2,d3) [ -d3*sin(theta2)*sin(theta1), -d3*cos(theta1)*cos(theta2), cos(theta1)*sin(theta2);
                                    d3*cos(theta1)*sin(theta2), d3*sin(theta1)*cos(theta2), sin(theta2)*sin(theta1);
                                                    0,              d3*sin(theta2),             -cos(theta2);
                                                    0,                 sin(theta1),                       0;
                                                    0,                -cos(theta1),                       0;
                                                    1,                           0,                       0];                                               

 
bx =     [-1 0 0 3 0 0; 
        0 0 0 1 0 0; 
        0 0 0 0.6 0 0; 
        0 0 0 0 0 0];
by =   [1 0 0 2 0 0; 
        3 0 0 1 0 0; 
        3 0 0 3 0 0; 
        -3 0 0 -1 0 0];  
bz =   [3 0 0 3.3 0 0; 
        3 0 0 3 0 0; 
        3 0 0 3 0 0; 
        3 0 0 3 0 0];     
 
q = [pi/2;0;3];
dt = 0.0075;
for i = 1:4
    for t = 0:dt:1
        
        a1 = M(0,1)\bx(i,:)'; % 6 by 6 with a 6 by 1
        a2 = M(0,1)\by(i,:)';
        a3 = M(0,1)\bz(i,:)';
        
        xdot = [0    1   2*t    3*t^2  4*t^3  5*t^4]* a1; % 1 by 6 with a 6 by 1
        ydot = [0    1   2*t    3*t^2  4*t^3  5*t^4]* a2;
        zdot = [0    1   2*t    3*t^2  4*t^3  5*t^4]* a3;
        
        qdot = pinv(J(q(1),q(2),q(3)))*[xdot; ydot; zdot; 0;0; 0];
        q = q + qdot*dt;
        
        T1 = A_1(q(1));
        T2 = A_1(q(1))*A_2(q(2));
        T3 = A_1(q(1))*A_2(q(2))*A_3(q(3));
        X = [0,T1(1,4),T2(1,4),T3(1,4)];
        Y = [0,T1(2,4),T2(2,4),T3(2,4)];
        Z = [0,T1(3,4),T2(3,4),T3(3,4)];
        
        xpoints = [0, 1, 1, 0, 0];
        ypoints= [3, 3, 3, 3, 3];
        zpoints = [1, 1, 3, 3, 1];

        scatter3(X,Y,Z,200,'blue','filled');
        line(X,Y,Z,'color','black','linewidth',5)
        line(xpoints,ypoints,zpoints,'color', 'red', 'linewidth', 7)
        hold on
        grid on
        axis ([-1 5 -5 3 0 10])
        xlabel('x')
        ylabel('y')
        zlabel('z')
        view(25,25)
        pause(dt)
        hold off
        title('RRP Quintic trajectory')
        
    end
    
end

