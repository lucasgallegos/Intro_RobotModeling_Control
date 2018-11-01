
% Lucas Gallegos
% Intro to Robot Modeling and Control
% HW3
% 3 DOF Robot

clear

close all
clc
num_links = 3;
dofs = 3;

theta = sym('theta', [1 num_links]);
alpha = sym('alpha', [1 num_links]);
d = sym('d', [1 num_links]);
a = sym('a', [1 num_links]);
L = sym('L', [1 num_links]);

theta(2) = theta(2)+(pi/2);
theta(3) = 0;
d(1)= L(1); %This is just some arbitrary constant length
d(2) = 0;
a(1:3) = 0;
alpha(1) = pi/2;
alpha(2) = pi/2;
alpha(3) = 0;

A = A_matrices_sym(dofs,theta,alpha,d,a); %function to calculate A matrices

A1 = A(1:4,[1 2 3 4]);
A2 = A(5:8,[1 2 3 4]);
A3 = A(9:12,[1 2 3 4]);

T_10 = A1; %Transformation Matrices
T_20 = A1*A2;
T_30 = A1*A2*A3;

J = sym(zeros(6,num_links)); %initialize Jacobian matrix

O_0 = [0;0;0];
O_1 = T_10(1:3, 4);
O_2 = T_20(1:3, 4);
O_3 = T_30(1:3, 4);

z_0 = [0;0;1];
z_1 = T_10(1:3,3);
z_2 = T_20(1:3,3);
z_3 = T_30(1:3,3);

%First column of Jacobian
J(1:3,1)= cross(z_0,(O_3-O_0));
J(4:6,1) = z_0;

cross(z_1,(O_3-O_1));

%second column
J(1:3,2)= cross(z_1,(O_3-O_1));
J(4:6,2) = z_1;

%third column
J(1:3,3)= z_2;
J(4:6,3) = 0;


J_simp = simplify(J)
J_latex = latex(J_simp)



