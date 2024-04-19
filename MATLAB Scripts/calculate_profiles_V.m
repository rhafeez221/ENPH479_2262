function [Qc, Tc, delT, I] = calculate_profiles_V(n, Th, Sm, Rm, Km, m, c, V)
    % Initialize arrays
    Qc = zeros(1, n);
    Tc = zeros(1, n);
    delT = zeros(1, n);
    I = zeros(1, n);
   

    % Initial conditions
    Tc(1) = Th;
    Qc(1) = Sm * Tc(1) * ((V(1)-Sm*(Th-Tc(1)))/Rm) - 0.5 * Rm * (((V(1)-Sm*(Th-Tc(1)))/Rm))^2 - Km * (Th - Tc(1));
    delT(1) = Qc(1) / (m * c);
    I(1) = ((V(1)-Sm*(Th-Tc(1)))/Rm);

    % Loop
    for x = 2:n  
       Tc(x) = Tc(x-1) - delT(x-1);
       Qc(x) = Sm * Tc(x) * (((V(x)-Sm*(Th-Tc(x)))/Rm)) - 0.5 * Rm * (((V(x)-Sm*(Th-Tc(x)))/Rm))^2 - Km * (Th - Tc(x));
       delT(x) = Qc(x) / (m * c);
       I(x) = ((V(x)-Sm*(Th-Tc(x)))/Rm);
    end
end
