function [result,trace] = SDA_Solve_Embedded_Profiled(input)
% Zero-vector penalty is already embedded in H; no explicit branch term.
[result,trace] = SDA_Solve_Profiled_Core(input,0.0,"embedded");
end
