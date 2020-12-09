clear all
close all
clear global
clc

csvfile = 'log_gpio_10-05-2020_19-46-20.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels

%  n x v: n samples for v elements
[n,v] = size(data); 

Ts = 0.01;           
fs = 1/Ts;
f = [0:n-1]'*(fs/n);    %frequency sample for bode like graphs
t = 0:Ts:n*Ts;          %time samples for responses

%% PLANT 
%motorA
P = tf([0.1843 -0.1805], [1 -1.89 0.8921], Ts);

%motorB
%we will consider motor B as similar as motor A because they vary so litle
%that their difference is negligeable

disp('Plant of assignment 1:');
damp(P)
mag_phase(P, f)

%% CONTROLLER

%open-loop frequency design
K = 7.8;
Ti = 0.2374;

%using Ziegler-Nichols closed-loop experiments
%K=8.55;        %Kr = 19
%Ti=0.17;       %Tr = 0.2

disp('Controller:');
D = tf([K*Ti+K*Ts -K*Ti], [Ti -Ti], Ts)
damp(D)
mag_phase(P*D, f)   %open-loop bode plot

%% CLOSED LOOP

disp('Closed-loop system with a controlelr:');
Kt=1;   % linear feedback
pi_wr = feedback(P*D, Kt)
damp(pi_wr)
mag_phase(pi_wr, f)
responses(pi_wr, t)







