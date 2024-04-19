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
        volt_value = step_size * (i - 1);
        
        % Identify time indices for each step
        %step_indices = (i - 1) * step_length &  i * step_length;
        if ( (i-1) == 0)
            %start_dex = 1;
            V(1:5) = volt_value; 
        else
            start_dex = (i-1)*step_length + 1;
            end_dex = i*(step_length);
            V(start_dex:end_dex) = volt_value; 
        end

        % Assign voltage value to corresponding time indices
        
    end
end
