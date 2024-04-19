function V = triangle_voltage_profile(max_voltage, num_steps, step_length)
    % 0:4 -> number of steps is 5.
    % 0:255 -> num steps = 256
    % Calculate step size
    V_1 = rampstep_voltage_profile(max_voltage, num_steps, step_length);
    V_2 = flip(V_1);
    V = [V_1 V_2(step_length+1:end)];
end
