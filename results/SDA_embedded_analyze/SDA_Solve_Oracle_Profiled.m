function [result,trace] = SDA_Solve_Oracle_Profiled(input,formulation,Uoptimal)
%SDA_SOLVE_ORACLE_PROFILED Diagnostic search with known optimum as incumbent.
% This is not implementable online; it isolates proof-stage search effort.
if strcmp(string(formulation),"embedded")
    lambdaZero=0.0;
else
    lambdaZero=double(input.lambdaZeroLogged);
end
[result,trace]=SDA_Solve_Profiled_Core(input,lambdaZero,"oracle_"+string(formulation),Uoptimal);
end
