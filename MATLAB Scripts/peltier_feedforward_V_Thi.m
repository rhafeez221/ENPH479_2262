clear variables;

testFilename = "Arduino Recorded Data\pid_test_070424_1624.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:10];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

k_ambient = 0.0;   % 0.76
res_alpha = 1.0;  % 0.38
A = 8.76; %voltage step value

% parameters to adjust simulation to real.
Rm_mod = 1.55;
Sm_mod = 0.495;
Km_mod = 0.34;

% Rm_mod = 1.41;
% Sm_mod = 0.495;
% Km_mod = 0.43;


Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
i_data = transpose(data_Matrix(:,4));
v_data = transpose(data_Matrix(:,8));

%i_avg = mean(i_data)
% operating params
Th = 273.15 + mean(Th_data); % hotside temp in K
Th_arr = Th_data + 273.15;
Tc_arr = Tc_data +273.15;
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

V = v_data * (8.76/255);
%V = step_current_profile(A, t);
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

% Disturbance Term [W]
D = 0;

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
    Qc(x) = Sm*Tc(x)*(((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm))  - 0.5*Rm*((((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm)))^2 - Km*(Th_arr(x) - Tc(x)) - K_steel*(Th_arr(x)-Tc(x));
    delT(x) = Qc(x)/(m*c);
    I(x) = ((V(x)-Sm*(Th_arr(x)-Tc(x)))/Rm);
end


% Temp Profile
setpoint = 15;
setpoint_arr = 15*ones(1,n);
tc_ref = 20.33;
tc_init = 22.88;


% slope function
max_slope = (15.29-tc_ref)/(51-0);
rise_time = (setpoint - tc_init)/max_slope;
Tc_profile = ones(1,n);

% this is the problem  child. 
delT_inv = 0;

Tc_profile(1) = tc_init;
for x = 2:(ceil(rise_time)+1)
    Tc_profile(x) = Tc_profile(x-1) + max_slope;
end

for x = ceil(rise_time)+1:n
    Tc_profile(x) = setpoint;
end

Tc_profile_sim = Tc_profile + 273.15;

% Inverter
I_inv_neg = zeros(1,n);
I_inv_pos = zeros(1,n);
V_inv_neg = zeros(1,n);
V_inv_pos = zeros(1,n);

for x = 1:n
%     if x==1
%         delT_test = 0;
%     else
%         delT_test = Tc_data(x) - Tc_data(x-1);
%     end
    I_inv_neg(x) = (Sm*Tc_arr(x))/Rm - sqrt((Sm^2*Tc_arr(x).^2)/Rm^2 - 2*(Km/Rm)*(Th_arr(x)-Tc_arr(x)) - 2*m*c*delT(x)/Rm - 2*(K_steel/Rm)*(Th_arr(x)-Tc_arr(x)) - 2*D/Rm);
    I_inv_pos(x) = (Sm*Tc_arr(x))/Rm + sqrt((Sm^2*Tc_arr(x).^2)/Rm^2 - 2*(Km/Rm)*(Th_arr(x)-Tc_arr(x)) - 2*m*c*delT(x)/Rm - 2*(K_steel/Rm)*(Th_arr(x)-Tc_arr(x)) - 2*D/Rm);
    V_inv_neg(x) = Sm*(Th_arr(x)-Tc_arr(x)) + I_inv_neg(x)*Rm;
    V_inv_pos(x) = Sm*(Th_arr(x)-Tc_arr(x)) + I_inv_pos(x)*Rm;
end

clear figure;
f = figure;

nexttile
yyaxis left
plot(t, Tc - 273.15, Color='red', LineWidth=1.0, LineStyle='-')
hold on
plot(t, Tc_data, Color='blue', LineWidth=1.0, LineStyle='-')
hold on
xlabel('Time (s)', FontSize=24)
ylabel('Tc [C]', FontSize=24)
%title('tuned sim vs real', FontSize=24)
%legend('Sim', 'Real', FontSize=24)

yyaxis right
plot(t, V, Color='black', LineWidth=0.5, LineStyle='-')
%hold on
plot(t, V_inv_neg, Color='green', LineWidth=1.0, LineStyle='-.')
hold on
plot(t, V_inv_pos, Color='magenta', LineWidth=1.0, LineStyle='--')
hold on
xlabel('Time (s)', FontSize=24)
ylabel('Voltage (V)', FontSize=24)
title('applied vs inverted', FontSize=24)
legend('Tc_sim', 'Tc_real','Vapplied', 'neg', 'pos', FontSize=24)

% plot(t, Th_data, Color='red', LineWidth=1.0)
% hold on
% plot(t, Tc_data, Color='blue', LineWidth=1.0)
% hold on
% plot(t, Ta_data, Color='green', LineWidth=1.0)
% hold on
% plot(t, Twater_data, Color='black', LineWidth=1.0)
% xlabel('Time (s)', FontSize=24)
% ylabel('Temperature (C)', FontSize=24)
% title('Temperature Measured', FontSize=24)
% legend('Hot', 'Cold', 'Ambient', 'Resevoir', FontSize=16)
