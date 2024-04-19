clear variables;

testFilename = "Arduino Recorded Data\step_test_080424_1852_trim.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:7];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

k_ambient = 0.0;   % 0.76
res_alpha = 1.0;  % 0.38
A = 8.76; %voltage step value

Rm_mod = 1.41;
Sm_mod = 0.495;
Km_mod = 0.43;
m_mod = 1.29;
% Rm_mod = 1.41;
% Sm_mod = 0.495;
% Km_mod = 0.43;

Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
switch_on = transpose(data_Matrix(:,5));
i_data = transpose(data_Matrix(:,4));
%v_data = transpose(data_Matrix(:,8));

%i_avg = mean(i_data)
% operating params
Th = 273.15 + mean(Th_data); % hotside temp in K
Th_arr = Th_data + 273.15;
m = 0.117 * m_mod; % mass of aluminum in kg
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


%V = v_data * (8.76/255);
V =step_current_profile(A, t);
%V = triangle_voltage_profile(A, 256, 5);

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
K_steel = 1.6*2*(k_steel*area/x_conduct); 

% % Initial conditions
Tc(1) = Tc_data(1,1) + 273.15;

Qc(1) = Sm*Tc(1)*((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm) - 0.5*Rm*(((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm))^2 - Km*(Th_arr(1) - Tc(1)) - K_steel*(Th_arr(1)-Tc(1));
% 
delT(1) = Qc(1)/(m*c);
% 
I(1) = ((V(1)-Sm*(Th_arr(1)-Tc(1)))/Rm);
% 
% % loop

d_start = 800;
d_end = 860;
D = 0.11*2.4*24;

for x = 2:d_start  
    Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
    Qc(x) = Sm*Tc(x)*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))  - 0.5*Rm*((((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm)))^2 - Km*(Th_arr(x) - Tc(x)) - K_steel*(Th_arr(x)-Tc(x));
    delT(x) = Qc(x)/(m*c);
    I(x) = ((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm);
end



for x = d_start:d_end  
    Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
    Qc(x) = Sm*Tc(x)*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))  - 0.5*Rm*((((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm)))^2 - Km*(Th_arr(x) - Tc(x)) - K_steel*(Th_arr(x)-Tc(x)) - D;
    delT(x) = Qc(x)/(m*c);
    I(x) = ((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm);
end
% 
for x = d_end:n  
    Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
    Qc(x) = Sm*Tc(x)*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))  - 0.5*Rm*((((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm)))^2 - Km*(Th_arr(x) - Tc(x)) - K_steel*(Th_arr(x)-Tc(x));
    delT(x) = Qc(x)/(m*c);
    I(x) = ((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm);
end

tiles = tiledlayout(2,2);

nexttile
yyaxis left
plot(t, Tc , Color='red')
hold on
plot(t, Tc_data + 273.15, Color='blue')
hold on
xlabel('Time (s)')
ylabel('Tc')
% 
yyaxis right
plot(t, switch_on, Color='magenta')
ylabel('switch')
title('Tc sim vs real')
legend('Sim', 'Real')

nexttile
yyaxis left
plot(t, I, Color='red')
hold on
plot(t, i_data, Color='blue', LineStyle='--', LineWidth=0.5)
hold on
xlabel('Time (s)')
ylabel('Current (A)')
 
yyaxis right
plot(t, switch_on, Color='Magenta')
ylabel('switch')
title('I sim vs real')
legend('Sim', 'Real')

nexttile
plot(t, V)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('V Step')
%legend('Sensed')

nexttile
yyaxis left
plot(t, Th_data+273.15, Color='red')
hold on
plot(t, Tc_data+273.15, Color='blue')
hold on
plot(t, Ta_data+273.15, Color='green', LineStyle='-')
hold on
xlabel('Time (s)')
ylabel('Temp (K)')

yyaxis right
plot(t, delT, Color='black', LineStyle='-')
xlabel('Time (s)')
ylabel('delT')
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
