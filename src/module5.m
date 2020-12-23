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
Jm = Jmod + Jmot*Kg^2;

%% 5.1

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
Q = C.' .* C;
R = 1e-2;   %remains with less than 10
K = lqr(A, B, Q, R);
sys_lqr = ss(A-B*K, B, C-D*K, D);
G = tf(sys_lqr);

Ts = 2*pi/10/40;    %CHECK HERE!!!!!!
fcl = 10/2/pi;
% s = tf('s'); 
% G = (Kg*Km*Ks*Ka)/((Jbr*s^2 + Ks)*(Rm*Jm*s^2 + Rm*b*s + Km^2 * Kg^2 *s) + Rm*Ks*Jbr*s^2)

% zoh = c2d(G,Ts,'zoh');
% zpm = c2d(G,Ts,'matched');
% tustin = c2d(G,Ts,'tustin');
% 
% figure()
% step(G, zoh, zpm, tustin)
% legend('continuous', 'zoh', 'zero-pole', 'tustin')
% 
% figure()
% bode(G, zoh, zpm, tustin)
% legend('continuous', 'zoh', 'zero-pole', 'tustin')

%% 5.2

% pole placement
H = tf([1, 2*0.8*10, 10^2], 1);
Pi = tzero(H);
m = real(max(Pi));   % no imaginary part

% Pc = [Pi; 2*m; 4*m];
% K= acker(A,B,Pc);
% sys_lqr = ss(A-B*K, B, C-D*K, D);
% sys_lqr_d = c2d(sys_lqr,Ts,'zoh');
% sys_lqr_d = tf(sys_lqr_d);
sys = ss(A, B, C, D);
sys = c2d(sys_lqr,Ts,'zoh');
sys = tf(sys);
Ts = 200*pi/10/40; %CHECK HERE!!!!!!
[B, A] = tfdata(sys, 'v');
Pol = tfdata(H, 'v')
A = conv(A,[1 -1]);
B = conv(B, [1 1]);

A
nA = length(A)-1
nB = length(B)-1
Bprev = B
B = B(2:end)
d = nA - nB
nR = nA -1
nS = nB + d - 1

% Ifand only ifA(q?1) and q?dB(q?1) are coprime.
% M = [[A'; 0;0;0], [0; A';0;0], [0;0; A';0],[0;0; 0;A'],[0;B'; 0;0; 0], [0;0;B'; 0; 0], [0;0; 0; B'; 0], [0;0;0; 0; B']]
% x = inv(M) * [Pol';0;0;0;0;0]   % [1; s1; s2; ...; r0; r1; ...]

% with integrator
% M = [[A'; 0;0;0;0], [0; A';0;0;0], [0;0; A';0;0],[0;0; 0;A';0],[0;0; 0;0;A'],[0;0;B'; 0;0; 0;0],[0;0;0;B'; 0;0; 0], [0;0;0;0;B'; 0; 0], [0;0;0;0; 0; B'; 0], [0;0;0;0;0; 0; B']]
% x = inv(M) * [Pol';0;0;0;0;0;0;0]   % [1; s1; s2; ...; r0; r1; ...]

% with integrator and regulator
M = [[A'; 0;0;0;0], [0; A';0;0;0], [0;0; A';0;0],[0;0; 0;A';0],[0;0; 0;0;A'],[0;B'; 0;0; 0;0],[0;0;B'; 0;0; 0], [0;0;0;B'; 0; 0], [0;0;0; 0; B'; 0], [0;0;0;0; 0; B']]
x = inv(M) * [Pol';0;0;0;0;0;0;0]   % [1; s1; s2; ...; r0; r1; ...]


% An = conv(A, [1 -1]);
% nA = length(An)-1
% nB = length(B)-1
% d = nA - nB
% nS = nB + d - 1
% nR = nA -1
% M = [[An'; 0;0], [0; An';0], [0;0; An'],[B';0; 0;0; 0],[0;B'; 0;0; 0], [0;0;B'; 0; 0], [0;0; 0; B'; 0], [0;0;0; 0; B']]
% x = inv(M) * [P';0;0;0;0;0]   % [1; s1; s2; ...; r0; r1; ...]

S = [];
T = [];
R = [];
for i = 1:nS+1
    S(i) = x(i);
end

for i = 1:nR+1
    R(i) = x(i + nS+1);
end
% S = conv(S,[1 -1])
% R = conv(R, [1 1])
S
R
T = evalfr(tf(R, 1), 1)   % with integrator
% T = P / evalfr(tf(Bprev, 1), 1)   % different dynamic tracking and regulation
% T = evalfr(tf(P, 1), 1) / evalfr(tf(Bprev, 1), 1)   %same dynamic tracking and regulation

Pcl=conv(A,S)+conv(Bprev,R)
nP = length(Pcl)-1
if nP > nA + nB + d -1
    error('P does not fit the required size')
end


sys_int = tf(conv(B,T),Pcl,Ts,'variable','z^-1')
figure
step(sys_int)








