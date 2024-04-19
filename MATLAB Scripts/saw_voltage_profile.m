function V = saw_voltage_profile(max_voltage, num_steps, step_length, num_cycles)
    % 0:4 -> number of steps is 5.
    % 0:255 -> num steps = 256
    % Calculate step size
    V_1 = triangle_voltage_profile(max_voltage, num_steps, step_length);
    V = [V_1];
    for i = 1:num_cycles
        V_i = flip(V_1);
        V = [V V_i(step_length+1:end)];
    end
end
