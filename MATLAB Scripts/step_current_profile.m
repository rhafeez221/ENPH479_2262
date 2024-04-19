function I = step_current_profile(final_current, time_array)
    % Generate a step current profile
    % Inputs:
    %   final_current: Final or maximal current value
    %   time_array: Time array
    % Output:
    %   I: Current profile

    % Initialize current profile
    I = zeros(size(time_array));

    % Apply step current
    I(time_array >= 0) = final_current;
end