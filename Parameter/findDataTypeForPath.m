function dataType = findDataTypeForPath(path, rules)
%FINDDATATYPEFORPATH Return the first matching DataType from ScalingRules.

dataType = '';
path = char(string(path));

for k = 1:numel(rules)
    pattern = char(string(rules(k).PathPattern));

    if matchesPathPattern(path, pattern)
        dataType = char(string(rules(k).DataType));
        return
    end
end
end

function isMatch = matchesPathPattern(path, pattern)
expression = ['^' regexptranslate('wildcard', pattern) '$'];
isMatch = ~isempty(regexp(path, expression, 'once'));
end
