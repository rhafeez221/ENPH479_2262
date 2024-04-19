function [Qc, Tc, delT, V] = calculate_profiles_Imod(n, Th, Ta, Sm, Rm, Km, m, c, I, res_alpha, k_amb)
    % Initialize arrays
    Qc = zeros(1, n);
    Tc = zeros(1, n);
    delT = zeros(1, n);
    V = zeros(1, n);

    % Initial conditions
    Tc(1) = Th;
    Qc(1) = Sm * Tc(1) * I(1) - 0.5 * Rm * (I(1))^2 - Km * (Th - Tc(1)) - k_amb*(Ta - Tc(1));
    delT(1) = Qc(1) / (m * c);
    V(1) = Sm * (Th - Tc(1)) + I(1) * Rm;

    % Loop
    for x = 2:n  
       Tc(x) = Tc(x-1) - res_alpha*delT(x-1);
       Qc(x) = Sm * Tc(x) * I(x) - 0.5 * Rm * (I(x))^2 - Km * (Th - Tc(x)) - k_amb*(Ta - Tc(x));
       delT(x) = Qc(x) / (m * c);
       V(x) = Sm * (Th - Tc(x)) + I(x) * Rm;
    end
end