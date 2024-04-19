clear variables;

testFilename = "Arduino Recorded Data\saw_test_260324_1610_trim.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:5];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

k_ambient = 0;   % 0.76
res_alpha = 1.0;  % 0.38
A = 8.68; %voltage step value

% parameters to adjust simulation to real.
Rm_mod = 1.0;
Sm_mod = 1.0;
Km_mod = 1.0;

Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
Twater_data = transpose(data_Matrix(:,4));
i_data = transpose(data_Matrix(:,5));

%i_avg = mean(i_data)
% operating params
Th = 273.15 + mean(Th_data); % hotside temp in K
m = 0.117; % mass of aluminum in kg
c = 921.096; % specific heat of Al in J/kg.K
Ta = 273.15 + mean(Ta_data);

% % peltier params
Sm = 0.05133 ;
Rm = 1.40110 ;
Km = 0.74433 ;


tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(i_data, 2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;
% 
%V = step_current_profile(A, t);
V = saw_voltage_profile(A, 256, 2,2);

[Qc, Tc, delT, I] = calculate_profiles_V(n, Th, Sm*Sm_mod, Rm*Rm_mod, Km*Km_mod, m, c, V);

f = figure;

tiles = tiledlayout(2,2);

% Tc tile
ax1 = nexttile;
h1 = plot(t, Tc , Color='red');
hold on
plot(t, Tc_data + 273.15, Color='blue')
hold on
xlabel('Time (s)')
ylabel('Tc')
title('Tc sim vs real')
legend('Sim', 'Real')

ax2 = nexttile;
h2 = plot(t, I, Color='red');
hold on
plot(t, i_data, Color='blue')
hold on
xlabel('Time (s)')
ylabel('Current (A)')
title('I sim vs real')
legend('Sim', 'Real')

ax3 = nexttile;
plot(t, V)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('V Step')
%legend('Sensed')

ax4 = nexttile;
plot(t, Th_data+273.15, Color='red')
hold on
plot(t, Tc_data+273.15, Color='blue')
hold on
plot(t, Ta_data+273.15, Color='green')
hold on
plot(t, Twater_data+273.15, Color='black')
xlabel('Time (s)')
ylabel('Temp (K)')
title('Temp Measured')
legend('Hot', 'Cold', 'Ambient', 'Resevoir')

b = uicontrol('Parent',f,'Style','slider','Position',[81,10,419,23],...
              'value',Rm_mod, 'min',0.0, 'max',5.0);
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Position',[50,10,23,23],...
                'String','0.0','BackgroundColor',bgcolor);
bl2 = uicontrol('Parent',f,'Style','text','Position',[500,10,23,23],...
                'String','5.0','BackgroundColor',bgcolor);
bl3 = uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
                'String','Rm Modifier','BackgroundColor',bgcolor);


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
