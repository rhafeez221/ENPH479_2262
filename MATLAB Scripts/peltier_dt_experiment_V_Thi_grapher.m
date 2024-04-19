clear variables;

testFilename = "Arduino Recorded Data\triangle_test_260324_1317_trim.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:5];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

k_ambient = 0.0;   % 0.76
res_alpha = 1.0;  % 0.38
A = 8.76; %voltage step value

% parameters to adjust simulation to real.
Rm_mod = 1.41;
Sm_mod = 0.495;
Km_mod = 0.43;
gamma = 0.0;

Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
Twater_data = transpose(data_Matrix(:,4));
i_data = transpose(data_Matrix(:,5));

%i_avg = mean(i_data)
% operating params
Th = 273.15 + mean(Th_data); % hotside temp in K
Th_arr = Th_data + 273.15;
m = 0.117; % mass of aluminum in kg
c = 921.096; % specific heat of Al in J/kg.K
Ta = 273.15 + mean(Ta_data);

% % peltier params
Sm = 0.05133 * Sm_mod;
Rm = 1.40110 * Rm_mod;
Km = 0.74433 * Km_mod;


tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(i_data, 2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;
% 


%V = step_current_profile(A, t);
V = triangle_voltage_profile(A, 256, 5);

% % initialize arrays
Qc = zeros(1, n);
Tc = zeros(1, n);
delT = zeros(1, n);
I = zeros(1, n);

% % Heat loss
d = 0.0035052;
area = pi*(d/2)^2;
k_steel =  16.3;
x_conduct = 0.003;
K_steel = 1.0*2*(k_steel*area/x_conduct); 

% % Initial conditions
Tc(1) = Tc_data(1,1) + 273.15;

Qc(1) = Sm*Tc(1)*((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm) - 0.5*Rm*(((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm))^2 - Km*(Th_arr(1) - Tc(1)) - K_steel*(Th_arr(1)-Tc(1));
% 
delT(1) = Qc(1)/(m*c);
% 
I(1) = ((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm);
% 
% % loop
for x = 2:n  
    Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
    Qc(x) = Sm*Tc(x)*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))  - 0.5*Rm*((((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm)))^2 - Km*(Th_arr(x) - Tc(x)) - K_steel*(Th_arr(x)-Tc(x)) - gamma*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))^3;
    delT(x) = Qc(x)/(m*c);
    I(x) = ((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm);
end

f = figure;

% yyaxis left
% plot(t, Tc - 273.15 , Color='red', LineWidth=1.0)
% hold on
% plot(t, Tc_data, Color='blue', LineWidth=1.0)
% hold on
% xlabel('Time (s)', FontSize=24)
% 
% ylabel('Tc [C]', FontSize=24)
% title('tuned sim vs real', FontSize=24)
% legend('Sim', 'Real', FontSize=24)
% 
% yyaxis right
% plot(t, I, Color='red', LineWidth=1.0)
% hold on
% plot(t, i_data, Color='blue', LineWidth=1.0)
% hold on
% xlabel('Time (s)', FontSize=24)
% ylabel('Current (A)', FontSize=24)
% title('tuned sim vs real', FontSize=24)
% legend('Sim', 'Real', FontSize=24)

plot(t, Th_data, Color='red', LineWidth=1.0)
hold on
plot(t, Tc_data, Color='blue', LineWidth=1.0)
hold on
plot(t, Ta_data, Color='green', LineWidth=1.0)
hold on
plot(t, Twater_data, Color='black', LineWidth=1.0)
xlabel('Time (s)', FontSize=24)
ylabel('Temperature (C)', FontSize=24)
title('Temperature Measured', FontSize=24)
legend('Hot', 'Cold', 'Ambient', 'Resevoir', FontSize=16)
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
