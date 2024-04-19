clear variables;

testFilename = "Arduino Recorded Data\step_test_190324_1345_trim.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:4];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

% parameters to adjust simulation to real.
res_alpha = 0.51;  % 0.6
k_ambient = 0.58;   % 0.17
beta = 1.222;

T_amb_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
T_hot_data = transpose(data_Matrix(:,3));
i_data = transpose(data_Matrix(:,4));

i_avg = mean(i_data)
% operating params
Th = 273.15 + mean(T_hot_data); % hotside temp in K
m = 0.117; % mass of aluminum in kg
c = 921.096; % specific heat of Al in J/kg.K
Ta = 273.15 + mean(T_amb_data);

% % peltier params
Sm = 0.05133;
Rm = 1.40110;
Km = 0.74433;


tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(i_data, 2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;
% 
V_data = step_current_profile(8.2, t);
% I = current_profile(4.0, 0.05, t, 0);
I = i_data;
% % initialize arrays
Qc = zeros(1, n);
Tc = zeros(1, n);
delT = zeros(1, n);
V = zeros(1, n);

% % Initial conditions
Tc(1) = Tc_data(1,1) + 273.15;

Qc(1) = Sm*Tc(1)*I(1) - 0.5*Rm*(I(1))^2 - Km*(Th - Tc(1)) -k_ambient*(Ta - Tc(1));
% 
delT(1) = Qc(1)/(m*c);
% 
V(1) = beta*(Sm*(Th - Tc(1)) + I(1)*(Rm)) ;
% 
% % loop
for x = 2:n  
    Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
    Qc(x) = Sm*Tc(x)*I(x) - 0.5*Rm*(I(x))^2 - Km*(Th - Tc(x)) -k_ambient*(Ta - Tc(x));
    delT(x) = Qc(x)/(m*c);
    V(x) = beta*(Sm*(Th - Tc(x)) + I(x)*(Rm));
end

V_sim_avg = mean(V(600:end))

tiles = tiledlayout(2,2);

nexttile
plot(t, Tc , Color='red')
hold on
plot(t, Tc_data + 273.15, Color='blue')
hold on
xlabel('Time (s)')
ylabel('Tc')
title('Tc sim vs real')
legend('Sim', 'Real')

nexttile
plot(t, V)
hold on
plot(t, V_data)
hold on
xlabel('Time (s)')
ylabel('Voltage')
title('V sim vs real?')
legend('Sim', 'Real')

nexttile
plot(t, I)
xlabel('Time (s)')
ylabel('Current (A)')
title('I sensed')
legend('Sensed')

nexttile
plot(t, T_hot_data+273.15, Color='red')
hold on
plot(t, Tc_data+273.15, Color='blue')
hold on
plot(t, T_amb_data+273.15, Color='green')
xlabel('Time (s)')
ylabel('Temp (K)')
title('Temp Measured')
legend('Hot', 'Cold', 'Ambient')
% 
% tiles = tiledlayout(2,2);
% xlow = -50;
% 
% 
% nexttile
% plot(t, Tc)
% xlabel('Time')
% xlim([xlow n])
% ylabel('Tc')
% %ylim([0 1.1*Tc(n)])
% title('Tc Profile')
% grid on
% 
% nexttile
% plot(t, I)
% xlabel('Time')
% xlim([xlow n])
% ylabel('Current')
% ylim([0 1.1*I(n)])
% title('I Profile')
% grid on
% 
% nexttile
% plot(t, V)
% xlabel('Time')
% xlim([xlow n])
% ylabel('Voltage')
% ylim([0 1.1*V(n)])
% title('V Profile')
% grid on
% 
% nexttile
% plot(t, Qc)
% xlabel('Time')
% xlim([xlow n])
% ylabel('Heat Pumped')
% %ylim([0 1.1*Qc(n)])
% title('Qc Profile')
% grid on
