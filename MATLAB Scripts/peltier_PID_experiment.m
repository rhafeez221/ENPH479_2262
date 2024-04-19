clear variables;

testFilename = "Arduino Recorded Data\pid_test_070424_1624.csv";
opts = detectImportOptions(testFilename);
preview(testFilename, opts);

opts.SelectedVariableNames = [1:10];
%opts.DataRange = '2:-1';
data_Matrix = readmatrix(testFilename, opts);

Ta_data = transpose(data_Matrix(:,1));
Tc_data = transpose(data_Matrix(:,2));
Th_data = transpose(data_Matrix(:,3));
i_data = transpose(data_Matrix(:,4));
%switch_on = transpose(data_Matrix(:,6));
pwm_data = transpose(data_Matrix(:,8));


V = pwm_data * (8.76/255);

tStart = 0;
dt = 1; % in seconds
% % num iter = n
n = size(i_data, 2);
tTotal = n*dt; % in seconds
% 
t = tStart + (0:n-1)*dt;
% 

setpoint = 15;
setpoint_arr = 15*ones(1,n);
tc_ref = 20.33;
tc_init = 22.88;

% slope function
max_slope = (15.29-tc_ref)/(51-0);
rise_time = (setpoint - tc_init)/max_slope;
temp_profile = ones(1,n);


temp_profile(1) = tc_init;
for x = 2:(ceil(rise_time)+1)
    temp_profile(x) = temp_profile(x-1) + max_slope;
end

for x = ceil(rise_time)+1:n
    temp_profile(x) = setpoint;
end
f = figure;

yyaxis left
plot(t, Tc_data, Color='blue', LineWidth=1.0)
hold on
plot(t, setpoint_arr, Color='magenta', LineWidth=0.5, LineStyle='-')
hold on
%
%temp_profile
hold on
xlabel('Time (s)', FontSize=24)

ylabel('Tc [C]', FontSize=24)
title('tuned sim vs real', FontSize=24)
%legend('Recorded', 'Setpoint', FontSize=24)

yyaxis right
plot(t, V, Color='black', LineWidth=1.0)
hold on
xlabel('Time (s)', FontSize=24)%
ylabel('Voltage (V)', FontSize=24)
title('PID Response', FontSize=24)
%legend('Sim', 'Real', FontSize=24)

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
