function [] = mag_phase(sys,f)
%MAG_PHASE Summary of this function goes here
%   Plots the magnitude and phase using the frequency response of the sytem

FRF = squeeze(freqresp(sys,2*pi*f));
figure('Name','Magnitude & phase shift')
subplot(2,1,1)
semilogx(f, 20*log10(abs(FRF)))
title('Bode')
grid on
xlim([f(1) f(end)])
xlabel('w  [rad/s]')
ylabel('|FRF|  [dB]')
subplot(2,1,2),semilogx(f, 180/pi*unwrap(angle(FRF)))
grid on
xlim([f(1) f(end)])
xlabel('w  [rad/s]')
ylabel('\phi(FRF)  [^\circ]')

end

