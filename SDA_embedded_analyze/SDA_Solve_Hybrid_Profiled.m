function [result,trace,UembeddedBabai] = SDA_Solve_Hybrid_Profiled(explicitInput,embeddedInput)
%SDA_SOLVE_HYBRID_PROFILED Embedded Babai upper bound, explicit SDA search.
UembeddedBabai=embedded_babai_candidate(embeddedInput);
[result,trace]=SDA_Solve_Profiled_Core(explicitInput, ...
    double(explicitInput.lambdaZeroLogged),"hybrid",UembeddedBabai);
end
