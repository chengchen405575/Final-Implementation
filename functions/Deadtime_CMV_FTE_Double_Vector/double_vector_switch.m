function U_apply = double_vector_switch(UapplyFirst, UapplySecond)
%DOUBLE_VECTOR_SWITCH Apply two MPC vectors in consecutive half-periods.
% UapplyFirst and UapplySecond are the two 3x1 switching vectors obtained
% from Uapply(1:3) and Uapply(4:6), respectively.
% Configure this MATLAB Function block with sample time pmsm_MPC_Ts/2 and
% hold both inputs for the complete external MPC period. After reset, the
% first invocation outputs Uapply; the next outputs UapplySecond.

persistent useSecondVector
if isempty(useSecondVector)
    useSecondVector = false;
end

if useSecondVector
    U_apply = UapplySecond;
else
    U_apply = UapplyFirst;
end

useSecondVector = ~useSecondVector;

end
