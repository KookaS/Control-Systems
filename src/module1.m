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

%% 1.3 Analysis of the feedback systems

% Gain
Dc = 1.2;

% Transfer function, S = E/R, U = U/R, T = Y/R, V = Y/W
S = tf(1/(1+G*Dc));
U = tf(Dc/(1+Dc*G));
T = tf(Dc*G/(1+Dc*G));
V = tf(G/(1+Dc*G));

% Display
zpk(minreal(G))
zpk(minreal(S))
zpk(minreal(U))
zpk(minreal(T))
zpk(minreal(V))

% Step response
subplot(2,2,1)
step(T)
title('Step response')

subplot(2,2,2)
step(U)
title('Control signal')

subplot(2,2,3)
step(S)
title('Error signal')

subplot(2,2,4)
step(V)
title('Disturbance response')

% Poles of T
pole(minreal(T))

%% 1.4 Computing the ultimate gain

% Routh stability criterion
syms k

a1=50.145;
a2=847.945;
a3=16261.45;
a4=58474*k;

A = [1 a2; a1 a3];
b1 = - (det(A)/ a2)

B = [1 a4; a1 0];
b2 = - (det(B) / a2);

C = [a1 a3; b1 b2];
c1 = - (det(C) / b1)

C = [a1 0; b1 0];
c2 = -(det(C)/b1)

%% 1.5 Closed-loop step response analysis

figure
rlocus(G) % Ku = 2.9

% compute T with new Dc = Ku*0.6
Ku = 2.9;
Dc = 0.6*Ku;

T = tf(Dc*G/(1+Dc*G));

% Info of new T = Y/R
figure
step(T)
stepinfo(T)
bandwidth(T)
figure
bodemag(T)