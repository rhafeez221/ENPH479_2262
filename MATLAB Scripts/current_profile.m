function I = current_profile(A, k, t, t0)
    %t: time
    %A: initial Amplitude
    %k: decay constant 
    %C: asymptote value
    I = A - A*(exp(-k*(t-t0)));
end
