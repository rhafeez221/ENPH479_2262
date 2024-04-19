function I = sinusoidal_current_profile(amplitude, frequency, phase_shift, time_array)
    % Generate a sinusoidal current profile
    % Inputs:
    %   amplitude: Amplitude of the sinusoidal current
    %   frequency: Frequency of the sinusoidal current (in Hz)
    %   phase_shift: Phase shift of the sinusoidal current (in radians)
    %   time_array: Time array
    % Output:
    %   I: Current profile

    % Generate sinusoidal current profile
    I = amplitude * (sin(2 * pi * frequency * time_array + phase_shift) + 1)/2;
end