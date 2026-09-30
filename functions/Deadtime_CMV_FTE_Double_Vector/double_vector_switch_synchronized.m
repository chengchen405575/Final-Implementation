function [U_apply, selectedHalf, newSolutionDetected] = ...
    double_vector_switch_synchronized( ...
    UapplyFirst, UapplySecond, solutionToggle)
%DOUBLE_VECTOR_SWITCH_SYNCHRONIZED Apply a new vector pair in fixed order.
% solutionToggle changes once whenever SDA_Solve_Synchronized finishes.
% Its edge resets the sequence to UapplyFirst. The next invocation applies
% UapplySecond. Further invocations hold UapplySecond until a new solution.
%
% Diagnostic outputs:
%   selectedHalf        uint8(1): first vector; uint8(2): second vector
%   newSolutionDetected true on the call accepting a new SDA result

persistent lastSolutionToggle
persistent secondVectorPending

currentSolutionToggle = logical(solutionToggle);
if isempty(lastSolutionToggle)
    % Treat the first invocation as the arrival of a new SDA solution.
    lastSolutionToggle = ~currentSolutionToggle;
    secondVectorPending = false;
end

newSolutionDetected = (currentSolutionToggle ~= lastSolutionToggle);

if newSolutionDetected
    U_apply = UapplyFirst;
    selectedHalf = uint8(1);
    secondVectorPending = true;
    lastSolutionToggle = currentSolutionToggle;
elseif secondVectorPending
    U_apply = UapplySecond;
    selectedHalf = uint8(2);
    secondVectorPending = false;
else
    % Do not restart an old vector pair if this task runs unexpectedly.
    U_apply = UapplySecond;
    selectedHalf = uint8(2);
end

end
