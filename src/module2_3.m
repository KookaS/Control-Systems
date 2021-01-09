clear all;
close all;
clc;

Ks=1.2;
Kg=70;
Km=0.00767;
Ka=2;
Jbr=0.0037;
Rm=2.6;
B=0.004;
Jmod=3.944*10^-4;
Jmot=3.87*10^-7;
Jm=Jmod+Kg^2*Jmot;
k=1.2;

s=tf('s');
G=minreal(Ks*Kg*Km*Ka/((Jbr*s^2+Ks)*(Rm*Jm*s^2+Rm*B*s+Kg^2*Km^2*s)+Rm*Ks*Jbr*s^2));
G1=minreal(G*s);
S=minreal(1/(1+k*G));
U=minreal(k/(1+k*G));
T=minreal(G*k/(1+k*G));
V=minreal(G/(1+k*G));

%% 2.3  Cascade Controller
wn=17.45;   % omega n 
x=3.6;  % gamma 
d =0.32;    % damping
taum=0.02;
Dcp=minreal((s^2+2*wn*d*s+wn^2)/(x*taum*s*wn^2 ));
Dc=5;
zpk(G1);

YR=minreal((Dc*Dcp*G1)/(s*(1+Dcp*G1)+Dc*Dcp*G1));
YW=minreal((G1)/(s+G1*Dcp+G1*Dcp*Dc));
zpk(YR)
minreal(zpk(minreal(YW)))

% plots
figure
step(G1)
title('Step of G1')

figure
step(YR,T)
title('Input step response')
legend('Cascade', 'Ziegler-Nichols')

figure
step(YW,V)
title('Disturbance step response')
legend('Cascade', 'Ziegler-Nichols')