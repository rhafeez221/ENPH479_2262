clear variables;

%% Operating Params
Th = 273.15 + 22; % hotside temp in K
m = 0.117; % mass of aluminum in kg
c = 921.096; % specific heat of Al in J/kg.K

%% Peltier Params
Sm = 0.05133;
Rm = 1.40110;
Km = 0.74433;

%% Profile Params


A = 8.64;
freq = 0.001;
phase = -pi/2;

%I = sinusoidal_current_profile(A, freq, phase,t);
V_1 = triangle_voltage_profile(A,256, 3);
V_2 = flip(V_1);
V_3 = flip(V_2);
V = [V_1 V_2(4:end) V_3(4:end)];

tStart = 0;
dt = 1; % in seconds
% num iter = n
n = size(V,2);
tTotal = n*dt; % in seconds
t = tStart + (0:n-1)*dt;

%% Generate Profiles
[Qc, Tc, delT, I] = calculate_profiles_V(n, Th, Sm, Rm, Km, m, c, V);

%Ta = 273.15 + 23;
%res_alpha = 0.6;
%k_amb = 0.17;

%[Qc_mod, Tc_mod, delT_mod, I_mod] = calculate_profiles_Imod(n, Th, Ta, Sm, Rm, Km, m, c, V, res_alpha, k_amb);

tiles = tiledlayout(2,2);
xlow = -50;

nexttile
plot(t, Tc)
%hold on
%plot(t, Tc_mod)
xlabel('Time')
xlim([xlow n])
ylabel('Tc')
%ylim([0 1.1*Tc(n)])
title('Tc Profile')
%legend("Base Sim", "Mod Sim")
grid on

nexttile
plot(t, I)
xlabel('Time')
xlim([xlow n])
ylabel('Current')
%ylim([0 1.1*I(n)])
title('I Sim')
grid on

nexttile
plot(t, V)
xlabel('Time')
xlim([xlow n])
ylabel('Voltage')
%ylim([0 1.1*V(n)])
title('V Profile')
grid on

nexttile
plot(t, Qc)
xlabel('Time')
xlim([xlow n])
ylabel('Heat Pumped')
%ylim([0 1.1*Qc(n)])
title('Qc Profile')
grid on

