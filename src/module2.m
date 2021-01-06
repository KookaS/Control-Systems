clear all;
close all;
clc;

Ka=2;
Rm=2.6;
Km=0.00767;
KG=70;
Jmot=3.87e-07;
Ks=1.2;
b=0.004;
Kg = 70;
Jmod = 3.944e-04;
Jm = Jmod + Jmot*Kg^2;
JBr=0.0037;

% Plant
s=tf('s');
G=tf(Ks*KG*Km*Ka/((JBr*s^2+ Ks)*(Rm*Jm*s^2+Rm*b*s+Km^2*KG^2*s)+Rm*Ks*JBr*s^2));

%% 2.1 ZN First method

figure
step(G)
L = 0.1;
R = 1;

Kp = 1.2/R*L;
Ti = 2*L;
Td = 0.5*L;

C = pidstd(Kp,Ti,Td)

T_pi = feedback(C*G, 1);
figure
step(T_pi)

%% 2.2 ZN Second method

% Looking for Pu
Ku = 2.9;
D_pi = feedback(Ku*G, 1);
step(D_pi);

% Generate new model
Pu = 0.35;

Kp = 0.6*Ku;
Ti = 0.5*Pu;
Td = 0.125*Pu;

E = pidstd(Kp,Ti,Td);
E_pid = feedback(E*G, 1);
figure
step(E_pid)

% W/Y
D_pid = feedback(G, E);
figure
step(D_pid)

%% 2.3 Cascade Controller


