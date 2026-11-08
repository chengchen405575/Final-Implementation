function [result,trace,UembeddedBabai] = SDA_Solve_Hybrid_Profiled(explicitInput,embeddedInput)
%SDA_SOLVE_HYBRID_PROFILED Embedded Babai upper bound, explicit SDA search.
UembeddedBabai=embedded_babai_candidate(embeddedInput);
[result,trace]=SDA_Solve_Profiled_Core(explicitInput, ...
    double(explicitInput.lambdaZeroLogged),"hybrid",UembeddedBabai);
% In the hybrid comparison, Round and Shift are the explicit-geometry
% candidates, whereas Babai is constructed in embedded geometry and then
% reevaluated with the complete explicit objective.  The latter reevaluation
% is exactly the initial radius used by the hybrid solver.
result.candidateTotalCosts(3)=result.initialRadius;
result.candidateSequences(:,3)=UembeddedBabai;
end
