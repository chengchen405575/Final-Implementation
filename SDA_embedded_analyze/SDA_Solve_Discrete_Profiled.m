function [result,trace] = SDA_Solve_Discrete_Profiled(input)
% Explicit zero-vector penalty at completed three-switch block boundaries.
lambdaZero = double(input.lambdaZeroLogged);
[result,trace] = SDA_Solve_Profiled_Core(input,lambdaZero,"discrete");
end
