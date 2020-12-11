clear all;
close all;
clc;

Ka = 2;
Rm = 2.6;
Km = 0.00767;
Kg = 70;
Jmot = 3.87e-07;
Ks = 1.2;
b = 0.004;
Jmod = 3.944e-04;
Jbr = 0.0037;

%% 4.1
a1 = (b + Kg^2 * Km ^ 2 / Rm)/(Kg^2 * Jmot + Jmod);
A = [-a1, 0, 0, Ks/(Kg^2 * Jmot + Jmod);
    1, 0, 0, 0;
    a1, 0, 0, (-Ks/Jbr + 1/ (Kg^2 * Jmot + Jmod));
    0, 0, 1, 0];

B = [Kg * Km * Ka / Rm;
    0;
    0;
    0];

C = [0, 1, 0, 1];
D = 0;

%Co = ctrb(A, B);
%Ob = obsv(A, C);

%% 4.2
Q = C.' .* C
R = 1;
K = lqr(A, B, Q, R)

figure()
step(ss(A-B*K, B, C-D*K, D))
title(sprintf('Step for K'))
grid on

