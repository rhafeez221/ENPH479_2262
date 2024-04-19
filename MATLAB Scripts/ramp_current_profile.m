function I = ramp_current_profile(final_current, time_array)
    % Generate a ramp current profile
    % Inputs:
    %   final_current: Final or maximal current value
    %   time_array: Time array
    % Output:
    %   I: Current profile

    % Calculate ramp slope
    slope = final_current / max(time_array);

    % Generate ramp current profile
    I = slope * time_array;
    I(I > final_current) = final_current; % Ensure current does not exceed final value
end