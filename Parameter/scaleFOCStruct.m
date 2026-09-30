function out = scaleFOCStruct(in, path, rules, mode)
if nargin < 4 || isempty(mode)
    mode = 'fixpoint';
end
out = in;
path = string(path);
fields = fieldnames(in);
for k = 1:numel(fields)
    name = fields{k};
    value = in.(name);
    fullPath = path + "." + name;

    if isa(value, 'Simulink.Parameter')
        dataType = findDataTypeForPath(fullPath, rules);
        if ~isempty(dataType)
            value.DataType = resolveDataType(dataType, mode);
            out.(name) = value;
        end

    elseif isstruct(value)
        out.(name) = scaleFOCStruct(value, fullPath, rules, mode);

    elseif isnumeric(value) || islogical(value)
        dataType = findDataTypeForPath(fullPath, rules);
        if ~isempty(dataType)
            p = Simulink.Parameter;
            p.Value = value;
            p.Complexity = 'real';
            p.HasCoderInfo = false;
            p.DataType = resolveDataType(dataType, mode);
            p.Min = [];
            p.Max = [];
            p.DocUnits = '';
            p.CoderInfo.StorageClass = "ExportedGlobal";
            out.(name) = p;
        end
    end
end
end

function dt = resolveDataType(dataType, mode)
    dataType = char(dataType);
    if strcmp(mode,'double') && startsWith(strtrim(dataType),'fixdt')
        dt = 'double';        % nur Festkomma floaten
    else
        dt = dataType;        % fixpoint-Modus, boolean, uintN -> unverändert
    end
end