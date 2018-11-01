% Lucas Gallegos
% Intro to Robot Modeling and Control
% HW2
% 7 DOF Robot

clear
clc
close all

num_links = 7;
n_steps = 100;
q_theta = linspace(3*pi/2,2*pi,n_steps);
L1 = 5;
L2 = 5;

theta = sym('theta', [1 num_links]);
alpha = sym('alpha', [1 num_links]);
d = sym('d', [1 num_links]);
a = sym('a', [1 num_links]);
L = sym('L', [1 num_links]);

theta(2)=theta(2)-(pi/2);
theta(3)=theta(3)-(pi/2);
theta(6)=theta(6)-(pi/2);

d(1)=0;
d(2)=0;
d(3)=0;
d(4)=L(1);
d(5)=L(2);
d(6)=0;
d(7)=0;

a(1)=0;
a(2)=0;
a(3)=0;
a(4)=0;
a(5)=0;
a(6)=0;
a(7)=0;

dofs = 7; %There are only 7 dofs, the +1 is for the end effector transformation matrix

A = A_matrices_sym(dofs,theta,alpha,d,a);
 

    A1 = A(1:4,[1 2 3 4]);
    A2 = A(5:8,[1 2 3 4]);
    A3 = A(9:12,[1 2 3 4]);
    A4 = A(13:16,[1 2 3 4]);
    A5 = A(17:20,[1 2 3 4]);
    A6 = A(21:24,[1 2 3 4]);
    A7 = A(25:28,[1 2 3 4]);

    T_10 = A1; %Transformation Matrices
    T_20 = A1*A2;
    T_30 = A1*A2*A3;
    T_40 = A1*A2*A3*A4;
    T_50 = A1*A2*A3*A4*A5;
    T_60 = A1*A2*A3*A4*A5*A6;
    T_70 = A1*A2*A3*A4*A5*A6*A7;
    
    J = sym(zeros(6,num_links)); %initialize Jacobian matrix

O_0 = [0;0;0];
O_1 = T_10(1:3, 4);
O_2 = T_20(1:3, 4);
O_3 = T_30(1:3, 4);
O_4 = T_40(1:3, 4);
O_5 = T_50(1:3, 4);
O_6 = T_60(1:3, 4);
O_7 = T_70(1:3, 4);

z_0 = [0;0;1];
z_1 = T_10(1:3,3);
z_2 = T_20(1:3,3);
z_3 = T_30(1:3,3);
z_4 = T_40(1:3,3);
z_5 = T_50(1:3,3);
z_6 = T_60(1:3,3);
z_7 = T_70(1:3,3);

%First column of Jacobian
J(1:3,1)= cross(z_0,(O_7-O_0));
J(4:6,1) = z_0;

%second column
J(1:3,2)= cross(z_1,(O_7-O_1));
J(4:6,2) = z_1;

%third column
J(1:3,3)= cross(z_2,(O_7-O_2));
J(4:6,3) = z_2;

%fourth column
J(1:3,4)= cross(z_3,(O_7-O_3));
J(4:6,4) = z_3;

%fifth column
J(1:3,5)= cross(z_4,(O_7-O_4));
J(4:6,5) = z_4;

%sixth column
J(1:3,6)= cross(z_5,(O_7-O_5));
J(4:6,6) = z_5;

%seventh column
J(1:3,7)= cross(z_6,(O_7-O_6));
J(4:6,7) = z_6;


J


