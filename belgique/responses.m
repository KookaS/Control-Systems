function [] = responses(sys,t)
%RESPONSES Summary of this function goes here
%   Plot the desired responses

figure('Name','impulse, step & ramp response'), 
subplot(3,1,1)
impulse(sys)
title('Impulse response')
grid on
subplot(3,1,2),
step(sys)
title('Step response')
grid on
subplot(3,1,3)
plot(t, lsim(sys, t, t))
title('Ramp response')
grid on
end

