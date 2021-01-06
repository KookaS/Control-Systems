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
Jm = Kg^2 * Jmot + Jmod;

%% 4.1 State-space model of the flexible joint

a1 = (Rm *b + Kg^2 * Km ^ 2)/(Rm*Jm);
A = [-a1, 0, 0, Ks/Jm;
    1, 0, 0, 0;
    a1, 0, 0, -(Ks/Jbr + Ks/Jm);
    0, 0, 1, 0];
b1 = Kg * Km * Ka / (Rm*Jm);
B = [b1;
    0;
    -b1;
    0];

C = [0, 1, 0, 1];
D = 0;
sys = ss(A, B, C, D);

Co = ctrb(A, B);
Ob = obsv(A, C);

%% 4.2 State-space controller design

% LQR
Q = C.' .* C;
R = 1e-2;   %remains with less than 10
K = lqr(A, B, Q, R);
sys_lqr = ss(A-B*K, B, C-D*K, D);
Pc = pole(sys_lqr);
figure
step(sys_lqr)

% Pole placement and estimator
Pe = Pc * 10;
L=acker(A.',C.',Pe).';

% feedforward gain
N = inv(C * inv(-A + B.*K) * B);
Acl = [A, -B.*K;
        L.*C, A - B.*K - L.*C];
Bcl = [B.*N;
        B.*N];
Ccl = [C, zeros(size(C))];

% transfer function y/r
sys_est = ss(Acl, Bcl, Ccl, D);
Tyr = tf(sys_est)
figure
step(Tyr)

% transfer function u/r
Ccl = [zeros(size(C)), -K];
sys_est = ss(Acl, Bcl, Ccl, D);
Tur = tf(sys_est)
figure
step(Tur)

%% 4.3 State-space controller with integrator

% integrator matrices
Abar=[A, zeros(4,1);-C, 0];
Bbar=[B;0];
Cbar = [C, 0];

% integrator controller with pole placement
H = tf([1, 2*0.8*10, 10^2], 1);
Pi = tzero(H);
m = real(max(Pi));   % no imaginary part
Pbar = [Pi; 2*m; 4*m; 6*m];
K= acker(Abar,Bbar,Pbar);    % [K0; K1]
sys_bar = ss(Abar-Bbar*K, Bbar, Cbar-D*K, D);
figure
step(sys_bar)

% pole placement, integrator and state estimator
L= acker(A',C',[10*Pi; 20*m; 40*m]).'
K0 = K(1:4);
K1 = K(5);
Aie = [A, -B*K0, -B*K1;
    L*C, A - B*K0 - L*C, -B*K1;
    -C, zeros(size(C)), 0];
Bie = [zeros(size(B)); zeros(size(B)); 1];
Cie = [C, zeros(size(C)), 0];

% transfer function Y/R
sys_ie = ss(Aie, Bie, Cie, D);
Hyr = tf(sys_ie)
figure
step(Hyr)

% transfer function U/R
Cie = [zeros(size(C)), -K];
sys_ie = ss(Aie, Bie, Cie, D);
Hur = tf(sys_ie)
figure
step(Hur)





