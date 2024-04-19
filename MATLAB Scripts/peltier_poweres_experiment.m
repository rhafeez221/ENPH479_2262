clear variables;

testFilename = "Arduino Recorded Data\power_res_test_090424_1056.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:7];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
i_data = transpose(data_Matrix(:,4));
switch_on = transpose(data_Matrix(:,5));
%%pwm_data = transpose(data_Matrix(:,8));



tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(i_data, 2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;
% 


f = figure;

yyaxis left
plot(t, i_data, Color='blue', LineWidth=1.0)
hold on
xlabel('Time (s)', FontSize=24)

ylabel('Current[A]', FontSize=24)
%title('tuned sim vs real', FontSize=24)
%legend('Recorded', 'Setpoint', FontSize=24)

yyaxis right
plot(t, switch_on, Color='red', LineWidth=1.0)
hold on
xlabel('Time (s)', FontSize=24)
ylabel('Switch Val', FontSize=24)
title('Pres Switch', FontSize=24)
legend('Current', 'Switch', FontSize=24)

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
