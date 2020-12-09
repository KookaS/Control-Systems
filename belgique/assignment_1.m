clear all
close all
clear global
clc

%This is to open and use the recording files:
%With the motorA in the air
%csvfile = 'C:\Users\Olivier\QRCrecords\log_gpio_08-05-2020_19-40-56.csv';

%With both motors on the ground at home on wood
%csvfile = 'log_gpio_10-05-2020_19-46-20.csv';

%With both motors on the ground in Leuven on slippery floor
csvfile = 'log_gpio_20-08-2020_17-52-01.csv';

labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels

%  n x v: n samples for v elements
[n,v] = size(data);

Ts = 0.01;           
fs = 1/Ts;
f = [0:n-1]'*(fs/n);    %frequency sample for bode like graphs
t = 0:Ts:n*Ts;          %time samples for responses

%{
% In the air measures
voltA = data(:,2);
spdA =  data(:,4);
%}

%{
% On wood measures
voltA = data(:,2);
spdA =  data(:,4);
voltB = data(:,6);
spdB =  data(:,8);
%}

% On slippery floor measures
voltA = data(:,5);
spdA =  data(:,4);
voltB = data(:,9);
spdB =  data(:,8);

%% Physical system
%{
    Manually found TF depending on the physical values of the system
    The values are taken from the exercices for a first usecase.

    PROPERTIES
    t                   time [s]
    f                   frequency [Hz]
    u                   voltage [V]
    w                   angular velocity [rad/s]
    Ra                  electrical resistance to convert u(t) into i(t) [ohm]
    Km                  converter for torque/current  [N*m/A]
    J                   inertia [N*s^2/rad]
    c                   friction coefficient [N*m*s/rad]
    Kb                  feedback converter for the velocity/voltage  [V*s/rad]
%}

Ra = 13.5;
Km = 1;
J = 0.0002;
c = 0;
Kb = 1;

disp('system with back euler:');
sys_beuler = tf([0 Ts], [Ra*J+(Ra*c+Km*Kb)*Ts -Ra*J], Ts)
damp(sys_beuler)
mag_phase(sys_beuler, f)
responses(sys_beuler, t)

fprintf('Closed loop without controller\nPress any key to continue \n'); pause;

%%  Estimated Velocity/Voltage
%{
    Estimated TF found function with data set with use of least-square
    method, LSM
%}

%use the transpose so that it fits the matrix shape for LSM
B = transpose(spdA(3:n));
Atrans = [-spdA(2:(n-1)), -spdA(1:(n-2)), voltA(2:(n-1)), voltA(1:(n-2))];
A = transpose(Atrans);
theta = A/B;
B1 = [0, theta(3), theta(4)];
A1 = [1, theta(1) theta(2)];

disp('system with parameters estimation:');
sys_est = tf(B1, A1, Ts)
damp(sys_est)
mag_phase(sys_est, f)
responses(sys_est, t)

figure('Name','Poles & zeros of sys_e_s_t')
pzmap(sys_est),  %z-plan unit circle
title('Pole-zero map of estimated function')
grid on 

fprintf('Empirical TF without filter\nPress any key to continue \n'); pause;

%%  Estimated Velocity/Voltage with filter
%{ 
    Estimated TF with low-pass filter, butterworth, 6th order
%}

fc = 1; %lower than fs because the rest is noise
wn = fc/(fs/2);

[B_filt,A_filt] = butter(6, wn);   %6th order with cutoff frequency.
spdA_filt = filter(B_filt, A_filt, spdA); 
voltA_filt = filter(B_filt, A_filt, voltA);
B = spdA_filt(3:n);
A = [-spdA_filt(2:n-1), -spdA_filt(1:n-2), voltA_filt(2:n-1), voltA_filt(1:n-2)];
theta = A\B;
B2 = [0, theta(3), theta(4)];
A2 = [1, theta(1) theta(2)];
sys_filtA = tf(B2, A2, Ts);

[B_filt,A_filt] = butter(6, wn);   %6th order with cutoff frequency.
spdB_filt = filter(B_filt, A_filt, spdB); 
voltB_filt = filter(B_filt, A_filt, voltB);
B = spdB_filt(3:n);
A = [-spdB_filt(2:n-1), -spdB_filt(1:n-2), voltB_filt(2:n-1), voltB_filt(1:n-2)];
theta = A\B;
B2 = [0, theta(3), theta(4)];
A2 = [1, theta(1) theta(2)];
sys_filtB = tf(B2, A2, Ts);

disp('system with parameters estimation with a butterwirth filter motorA:');
sys_filtA
damp(sys_filtA)
mag_phase(sys_filtA, f)
responses(sys_filtA, t)

figure('Name','Poles & zeros of sys_f_i_l_t')
pzmap(sys_filtA),  %z-plan unit circle
title('Pole-zero map for estimated filtered function')
grid on

disp('system with parameters estimation with a butterwirth filter motorB:');
sys_filtB
damp(sys_filtB)
mag_phase(sys_filtB, f)
responses(sys_filtB, t)

figure('Name','Poles & zeros of sys_f_i_l_t')
pzmap(sys_filtB),  %z-plan unit circle
title('Pole-zero map for estimated filtered function')
grid on

fprintf('Empirical TF with filter motorA\nPress any key to continue \n'); pause;


%% BODE PLOTS OF THE THREE SYSTEMS

FRF_empirical = squeeze(freqresp(sys_beuler,2*pi*f));
FRF_estimation = squeeze(freqresp(sys_est,2*pi*f));
FRF_filtered = squeeze(freqresp(sys_filtA,2*pi*f));

figure('Name','Magnitude & phase shift')
subplot(2,1,1)
semilogx(f, 20*log10(abs(FRF_empirical)))
hold on
semilogx(f, 20*log10(abs(FRF_estimation)))
semilogx(f, 20*log10(abs(FRF_filtered)))
hold off
title('Bode')
legend('empirical', 'estimation', 'estimation filtered')
grid on
xlim([f(1) f(end)])
xlabel('w  [rad/s]')
ylabel('|FRF|  [dB]')

subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF_empirical)))
hold on
semilogx(f, 180/pi*unwrap(angle(FRF_estimation)))
semilogx(f, 180/pi*unwrap(angle(FRF_filtered)))
hold off
legend('empirical', 'estimation', 'estimation filtered')
grid on
xlim([f(1) f(end)])
xlabel('w  [rad/s]')
ylabel('\phi(FRF)  [^\circ]')
