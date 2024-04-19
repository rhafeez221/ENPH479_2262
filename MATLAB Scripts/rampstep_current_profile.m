function V = rampstep_voltage_profile(max_voltage, num_steps, step_length)
    % 0:4 -> number of steps is 5.
    % 0:255 -> num steps = 256
    % Calculate step size
    step_size = max_voltage / (num_steps-1);

    % Initialize current profile array
    V = zeros(1, num_steps*step_length);

    % Generate current profile
    for i = 1:num_steps
       % Calculate voltage value for each step
        current_value = step_size * (i - 1);
        
        % Identify time indices for each step
        step_indices = (i - 1) * 5 <= time_array & time_array < i * 5;
        
        % Assign voltage value to corresponding time indices
        V(step_indices) = current_value; 
    end
end
