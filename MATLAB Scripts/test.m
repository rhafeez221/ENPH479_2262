clear variables;



%time_array = 0:255 *5;


A = 8.64;         % Final amplitude
freq = 0.001;
phase = -pi/2;


%V_step = step_current_profile(A, t);
%V_ramp = ramp_current_profile(A,t);
%V_sin = sinusoidal_current_profile(A, freq, phase, t);
%V_rampstep = rampstep_voltage_profile(A, 256, 5);
%V_flip = flip(V_rampstep);

%V_triangle = [V_rampstep V_flip(6:end)];
V_saw = saw_voltage_profile(8.64, 256, 2, 2);

tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(V_saw,2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;


%Sm = 0.05133;
%Rm = 1.4011;
%Km = 0.74433;

%Tc = linspace(323,273,2000);

%Th = 323* ones(1, 2000);
%Vi = Sm.*(Th - Tc) + I.*Rm; 

% Plot the result
clf;
%%plot(t, I_step)
%%hold on
%plot(t, I_ramp)
%hold on
plot(t, V_saw)
xlabel('Time')
ylabel('Voltage')
title('Signal Tests')
legend('RampStep')
grid on
