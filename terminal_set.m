clc;clear all;close all;

t_acc = 1.8;
ts  = 0.1;

Ac = [0 1 ;0 -t_acc];
Bc = [0;t_acc];
sysc = ss(Ac,Bc,[],[]);
sysd = c2d(sysc,ts,'zoh');
A = sysd.A; B = sysd.B;

H_u = [1;-1];
b_u = [0.95;3.95];
K = [0.0693 0.4151];
H_x = [1 0;0 1;0 -1;1 1;-2 -1];
b_x = [5/3.6;1;4;1.4;32];
H = [H_x;H_u*K];
b = [b_x;b_u];

P_e = Polyhedron(H,b);
plot(P_e,'alpha',0.1)

