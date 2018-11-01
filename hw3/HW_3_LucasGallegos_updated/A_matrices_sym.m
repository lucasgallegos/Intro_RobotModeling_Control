function [A] = A_matrices_sym(dofs,theta,alpha,d,a)

A_i = [];
A = [];

    for i = 1:dofs %This gets an A matrix for each DOF in symbolic form

        A_i = sym([cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
            sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
            0 sin(alpha(i)) cos(alpha(i)) d(i);
            0 0 0 1]);
        A = [A; A_i];
    end