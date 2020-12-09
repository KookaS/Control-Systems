clear all
close all
clear global
clc

%% assignement 3: state feedback with pole placement

Ts= 0.01;

%STATE SPACE MODEL WITH FORWARD EULER
A = 1;
B = Ts;
C = -1;
D = 0;

%% FULL STATE FEEDBACK & LQR

%PLOTS OF DIFFERENT K VALUES
figure
subplot(3,1,1)
rho = 1;
K = dlqr(A, B, rho, 1);
step(ss(A-B*K, B, C-D*K, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
title(sprintf('Step for K = %f', K))
grid on
subplot(3,1,2)
K = -1;
step(ss(A-B*K, B, C-D*K, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
title(sprintf('Step for K = %f', K))
grid on
subplot(3,1,3)
K = 2/Ts + 1;
step(ss(A-B*K, B, C-D*K, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
title(sprintf('Step for K = %f', K))
grid on

%PLOT OF DIFFERENT K VALUES
figure
title(sprintf('Step for K'))
K1 = 0.5;
step(ss(A-B*K1, B, C-D*K1, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
hold on
K2 = 0.995;
step(ss(A-B*K2, B, C-D*K2, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
K3 = 20;
step(ss(A-B*K3, B, C-D*K3, D, Ts, 'StateName', 'f', 'InputName',{'v'},'OutputName','x'))
legend(sprintf('K = %f', K1),sprintf('K = %f', K2),sprintf('K = %f', K3))
grid on
hold off


figure('Name','Poles & zeros of state feedback')
hold on
K = [0:1/(5*Ts):2/Ts];
K_legend = regexp(sprintf('K=%d#', K), '#', 'split');
K_legend(end)=[];
for i = 1:length(K)
    [b,a] = ss2tf(A-B*K(i), B, C-D*K(i), D);
    tff = tf(b, a, Ts);
    pzmap(tff)
end
hold off
legend(K_legend)
title('Pole-zero map for state feedback')
grid on

fprintf('Pause, Full State Feedback\nPress any key to continue \n'); pause;


%% PLOTS FOR DIFFERENT K VALUES FROM ROBOT

Q = [1e-6, 1e-6, 1e-6];
R = [1e-2, 1e-2, 1e-2];
K = [0.5, 0.995, 20];
t = 470;
t_plot = (0:1/100:(t-1)/100);
posA = {};
voltA = {};
xref(1) = 0.0;
xref(2:t) = 0.1;

csvfile = 'log_gpio_21-08-2020_11-51-46.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
posA{1} = data(406:406+t-1,5)./100;
voltA{1} = data(406:406+t-1,2)./100;
%xref = data(406:406+t-1,3)./100;

csvfile = 'log_gpio_21-08-2020_11-52-15.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
posA{2} = data(415:415+t-1,5)./100;
voltA{2} = data(415:415+t-1,2)./100;

csvfile = 'log_gpio_21-08-2020_11-52-38.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
posA{3} = data(402:402+t-1,5)./100;
voltA{3} = data(402:402+t-1,2)./100;

figure
plot(t_plot, posA{1})
hold on
plot(t_plot, posA{2})
plot(t_plot, posA{3})
plot(t_plot, xref)
legend(sprintf('K = %f, Q = %f, R = %f', K(1), Q(1), R(1)), sprintf('K = %f, Q = %f, R = %f', K(2), Q(2), R(2)), sprintf('K = %f, Q = %f, R = %f', K(3), Q(3), R(3)), 'xref')
grid on
title(sprintf('posA for K on robot'))
xlabel('t [s]')
ylabel('posA [m]')
hold off

figure
plot(t_plot, voltA{1})
hold on
plot(t_plot, voltA{2})
plot(t_plot, voltA{3})
plot(t_plot, xref)
legend(sprintf('K = %f, Q = %f, R = %f', K(1), Q(1), R(1)), sprintf('K = %f, Q = %f, R = %f', K(2), Q(2), R(2)), sprintf('K = %f, Q = %f, R = %f', K(3), Q(3), R(3)), 'xref')
grid on
title(sprintf('voltA for K on robot'))
xlabel('t [s]')
ylabel('voltA [V]')
hold off

fprintf('Pause, plot of K values\nPress any key to continue \n'); pause;



%% PLOTS P & L VALUES FROM ROBOT

Q = [1e-4, 1e-5, 1e-6];
R = [1e-4, 1e-3, 1e-2];
K = [0.995, 0.995, 0.995];
t = 600;
t_plot = (0:1/100:(t-1)/100);
P = {};
L = {};
L_manual = {};
xhat = {};
front_dist = {};
des_velocity = {};
nu = {};
S = {};

csvfile = 'log_gpio_21-08-2020_10-22-33.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
P{1} = data(408:408+t-1,11);
xhat{1} = data(408:408+t-1,10);
front_dist{1} = data(408:408+t-1,9);
des_velocity{1} = data(408:408+t-1,4);
nu{1} = data(408:408+t-1,12);
S{1} = data(408:408+t-1,13);

csvfile = 'log_gpio_21-08-2020_10-24-11.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
P{2} = data(307:307+t-1,11);
xhat{2} = data(307:307+t-1,10);
front_dist{2} = data(307:307+t-1,9);
des_velocity{2} = data(307:307+t-1,4);
nu{2} = data(307:307+t-1,12);
S{2} = data(307:307+t-1,13);

csvfile = 'log_gpio_21-08-2020_10-26-41.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
P{3} = data(135:135+t-1,11);
xhat{3} = data(135:135+t-1,10);
front_dist{3} = data(135:135+t-1,9);
des_velocity{3} = data(135:135+t-1,4);
nu{3} = data(135:135+t-1,12);
S{3} = data(135:135+t-1,13);

for i = 1:3
    for k = 1:t
        L{i}(k) = - (P{i}(k) + Q(i)) ./ (P{i}(k) + Q(i) + R(i));
        L_manual{i} = -(Q(i) + sqrt(Q(i)^2+4*Q(i)*R(i)))/(Q(i)+ sqrt(Q(i)^2+4*Q(i)*R(i))+2*R(i));   
    end
end


figure
plot(t_plot, P{1})
hold on
plot(t_plot, P{2})
plot(t_plot, P{3})
legend(sprintf('K = %f, Q = %f, R = %f', K(1), Q(1), R(1)), sprintf('K = %f, Q = %f, R = %f', K(2), Q(2), R(2)), sprintf('K = %f, Q = %f, R = %f', K(3), Q(3), R(3)))
grid on
title(sprintf('P covariance matrix'))
xlabel('t [s]')
ylabel('P')
hold off

figure
plot(t_plot, L{1})
hold on
plot(t_plot, L{2})
plot(t_plot, L{3})
legend(sprintf('K = %f, Q = %f, R = %f', K(1), Q(1), R(1)), sprintf('K = %f, Q = %f, R = %f', K(2), Q(2), R(2)), sprintf('K = %f, Q = %f, R = %f', K(3), Q(3), R(3)))
grid on
title(sprintf('L estimator gain'))
xlabel('t [s]')
ylabel('L')
hold off


L_manual

fprintf('Pause, plot of P & L\nPress any key to continue \n'); pause;

%% KALMAN

R_matrix = {};
f = {};
xref = {};
kalman = {};

for i = 1:3
    R_matrix{i}(1:t) = R(i);
    f{i} = front_dist{i} + des_velocity{i}/100;
    xref{i} = des_velocity{i}./K(i) + xhat{i};

    kalman{i} = KalmanExperiment(t_plot',f{i},P{i},front_dist{i},R_matrix{i}',xref{i},nu{i},S{i});
end

% consistency check of the Kalman filters
analyzeconsistency(kalman{1}); 
analyzeconsistency(kalman{2}); 
analyzeconsistency(kalman{3}); 

fprintf('Pause, plot of Kalman\nPress any key to continue \n'); pause;

%% WRONG INITIAL ESTIMATION

Q = [1e-4, 1e-5, 1e-6];
R = [1e-4, 1e-3, 1e-2];
K = [0.995, 0.995, 0.995];
xhat = {};
front_dist = {};

csvfile = 'log_gpio_23-08-2020_14-46-44.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
xhat{1} = data(:,10);
front_dist{1} = data(:,9);

csvfile = 'log_gpio_23-08-2020_14-48-23.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
xhat{2} = data(:,10);
front_dist{2} = data(:,9);

csvfile = 'log_gpio_23-08-2020_14-49-43.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
xhat{3} = data(:,10);
front_dist{3} = data(:,9);

for i=1:3
    t = length(xhat{i});
    t_plot = (0:1/100:(t-1)/100);

    figure
    hold on
    plot(t_plot, xhat{i})
    plot(t_plot, front_dist{i})
    legend(sprintf('xhat, K = %f, Q = %f, R = %f', K(i), Q(i), R(i)), sprintf('front distance, K = %f, Q = %f, R = %f', K(i), Q(i), R(i)))
    grid on
    title(sprintf('position estimation and front distance'))
    xlabel('t [s]')
    ylabel('xhat [m], front sensor [m]')
    hold off
end

fprintf('Pause, plot of wrong initial values\nPress any key to continue \n'); pause;

%% SLOWER ESTIMATOR POLE

Q = 1e-8;
R = 1e-2;
K = 0.995;
xhat = {};
front_dist = {};

csvfile = 'log_gpio_23-08-2020_14-51-38.csv';
labels = strsplit(fileread(csvfile));
labels = strsplit(labels{:, 2}, ','); % Labels are in line 2 of every record
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels
xhat = data(:,10);
front_dist = data(:,9);

t = length(xhat);
t_plot = (0:1/100:(t-1)/100);

figure
hold on
plot(t_plot, xhat)
plot(t_plot, front_dist)
legend(sprintf('xhat, K = %f, Q = %f, R = %f', K, Q, R), sprintf('front distance, K = %f, Q = %f, R = %f', K, Q, R))
grid on
title(sprintf('position estimation and front distance'))
xlabel('t [s]')
ylabel('xhat [m], front sensor [m]')
hold off
