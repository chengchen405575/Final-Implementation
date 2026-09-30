function createFOC_SLDD(modelName, slddPath, FOC)
%CREATEFOC_SLDD Create/update an SLDD from the calculated FOC struct.
%
% Usage:
%   createFOC_SLDD()
%   createFOC_SLDD("SVPWMSimulation")
%   createFOC_SLDD("SVPWMSimulation", "FOC_params.sldd")
%   createFOC_SLDD("SVPWMSimulation", "FOC_params.sldd", FOC)

if nargin < 1 || isempty(modelName)
    modelName = "";
end

if nargin < 2 || isempty(slddPath)
    slddPath = fullfile(pwd, "FOC_params.sldd");
end

slddPath = char(string(slddPath));
if isempty(fileparts(slddPath))
    slddPath = fullfile(pwd, slddPath);
end

if nargin < 3 || isempty(FOC)
    try
        FOC = evalin("base", "FOC");
    catch
        error("createFOC_SLDD:MissingFOC", ...
            "FOC is not available. Run FOC_Ini first or pass FOC as third argument.");
    end
end

% rules = loadScalingRules();

% If this SLDD is still open from a previous run, create/open will fail.
closeDictionaryConnections(slddPath);

if isfile(slddPath)
    fprintf("Existing dictionary will be overwritten: %s\n", slddPath);
    delete(slddPath);
end

dd = Simulink.data.dictionary.create(slddPath);
cleanupObj = onCleanup(@() closeDictionary(dd)); %#ok<NASGU>
ddsec = getSection(dd, "Design Data");

fprintf("Dictionary created: %s\n", slddPath);
fprintf("\n=== Adding parameters ===\n");

entries = getFOCEntries();
nLoaded = 0;

for i = 1:size(entries, 1)
    varName = entries{i, 1};
    structPath = entries{i, 2};

    try
        value = getStructPathValue(FOC, structPath);
        % dataType = findRuleDataType(structPath, rules);
        param = makeParam(value);

        addEntry(ddsec, varName, param);
        fprintf("  OK  %-45s  %s\n", varName, structPath);
        nLoaded = nLoaded + 1;
    catch ME
        fprintf("  ERR %-45s  %s\n", varName, ME.message);
    end
end

saveChanges(dd);
fprintf("\n=== Dictionary saved: %d entries ===\n", nLoaded);
fprintf("    -> %s\n\n", slddPath);

if strlength(string(modelName)) > 0
    linkDictionaryToModel(string(modelName), slddPath);
else
    fprintf("Link manually if needed:\n");
    fprintf("  set_param('DeinModell', 'DataDictionary', '%s')\n", slddPath);
    fprintf("  save_system('DeinModell')\n\n");
end
end

function entries = getFOCEntries()
% "FOC_SampleTime_TsCur",                 "FOC.SampleTime.TsCur";
% "FOC_SampleTime_TsSpeed",               "FOC.SampleTime.TsSpeed";
% "FOC_SampleTime_BSW_PulseGenFrequency", "FOC.SampleTime.BSW.PulseGenFrequency";
% "FOC_SampleTime_BSW_DeadTime",          "FOC.SampleTime.BSW.DeadTime";
% "FOC_SampleTime_BSW_DeadTimeSample",    "FOC.SampleTime.BSW.DeadTimeSample";
entries = {
    "FOC_PWM_frequency",                    "FOC.PWM_frequency";
    "FOC_LUT_NumOfPoints",                  "FOC.LUT.NumOfPoints";
    "FOC_Control_MTPA_MTPV_w_vec",          "FOC.Control.MTPA_MTPV.w_vec";
    "FOC_Control_MTPA_MTPV_T_vec",          "FOC.Control.MTPA_MTPV.T_vec";
    "FOC_Control_MTPA_MTPV_n_Tmin_grid",    "FOC.Control.MTPA_MTPV.n_Tmin_grid";
    "FOC_Control_MTPA_MTPV_n_Tmax_grid",    "FOC.Control.MTPA_MTPV.n_Tmax_grid";
    "FOC_Control_Current_PILUT_Ld_lut",     "FOC.Control.Current.PILUT.Ld_lut";
    "FOC_Control_Current_PILUT_Lq_lut",     "FOC.Control.Current.PILUT.Lq_lut";
    "FOC_Control_Current_PILUT_id_bp",      "FOC.Control.Current.PILUT.id_bp";
    "FOC_Control_Current_PILUT_iq_bp",      "FOC.Control.Current.PILUT.iq_bp";
    "FOC_Control_Current_TclCur",           "FOC.Control.Current.TclCur";
    "FOC_Control_Current_KPiq",             "FOC.Control.Current.KPiq";
    "FOC_Control_Current_KPiqMat",          "FOC.Control.Current.KPiqMat";
    "FOC_Control_Current_KIiq",             "FOC.Control.Current.KIiq";
    "FOC_Control_Current_KPid",             "FOC.Control.Current.KPid";
    "FOC_Control_Current_KPidMat",          "FOC.Control.Current.KPidMat";
    "FOC_Control_Current_KIid",             "FOC.Control.Current.KIid";
    "FOC_Control_Current_Krq",              "FOC.Control.Current.Krq";
    "FOC_Control_Current_Tnq",              "FOC.Control.Current.Tnq";
    "FOC_Control_Current_Krd",              "FOC.Control.Current.Krd";
    "FOC_Control_Current_Tnd",              "FOC.Control.Current.Tnd";
    "FOC_Control_Speed_PhiR",               "FOC.Control.Speed.PhiR";
    "FOC_Control_Speed_alpha",              "FOC.Control.Speed.alpha";
    "FOC_Control_Speed_Te",                 "FOC.Control.Speed.Te";
    "FOC_Control_Speed_Tns",                "FOC.Control.Speed.Tns";
    "FOC_Control_Speed_KPs",                "FOC.Control.Speed.KPs";
    "FOC_Control_Speed_KIs",                "FOC.Control.Speed.KIs";
    "FOC_SampleTime_TsSpeed",               "FOC.SampleTime.TsSpeed";
    "FOC_Control_OpenLoop_Uq_OpenLoop_V",   "FOC.Control.OpenLoop.Uq_OpenLoop_V";
    "FOC_Control_OpenLoop_Ud_OpenLoop_V",   "FOC.Control.OpenLoop.Ud_OpenLoop_V";
    "FOC_Control_OpenLoop_Iq_Ref_A",        "FOC.Control.OpenLoop.Iq_Ref_A";
    "FOC_Control_OpenLoop_Id_Ref_A",        "FOC.Control.OpenLoop.Id_Ref_A";
    "FOC_Control_TorqueRef_Nm",             "FOC.Control.TorqueRef_Nm";
    "FOC_Control_SpeedRef_rpm",             "FOC.Control.SpeedRef_rpm";
    "FOC_Control_RF_RateTrq_Nm",            "FOC.Control.RF_RateTrq_Nm";
    "FOC_Control_RF_RateSpeed_rpm",         "FOC.Control.RF_RateSpeed_rpm";
    "FOC_Control_RF_RateCurrent_A",         "FOC.Control.RF_RateCurrent_A";
    "FOC_Control_InverterVdc_V",            "FOC.Control.InverterVdc_V";
    "FOC_Control_MotorImax_A",              "FOC.Control.MotorImax_A";
    "FOC_Control_Umax_V",                   "FOC.Control.Umax_V";
    "FOC_Control_U_fb",                     "FOC.Control.U_fb";
    "FOC_Control_MTPA_MTPV_nT_id_grid",     "FOC.Control.MTPA_MTPV.nT_id_grid";
    "FOC_Control_MTPA_MTPV_nT_iq_grid",     "FOC.Control.MTPA_MTPV.nT_iq_grid";
    "FOC_Flag_ClosedLoopControl",           "FOC.Flag.ClosedLoopControl";
    "FOC_Flag_TorqueControl",               "FOC.Flag.TorqueControl";
    "FOC_Flag_PWMEnable",                   "FOC.Flag.PWMEnable";
    "FOC_Flag_ResetControl",                "FOC.Flag.ResetControl";
    "FOC_Flag_ControlStrategy",             "FOC.Flag.ControlStrategy";
    "FOC_LUT_ParkClake_theta_Vec",          "FOC.LUT.ParkClake.theta_Vec";
    "FOC_LUT_ParkClake_SinLut",             "FOC.LUT.ParkClake.SinLut";
    "FOC_LUT_ParkClake_CosLut",             "FOC.LUT.ParkClake.CosLut";
    "FOC_LUT_DQLimiter_SqrtLutx",           "FOC.LUT.DQLimiter.SqrtLutx";
    "FOC_LUT_DQLimiter_SqrtLuty",           "FOC.LUT.DQLimiter.SqrtLuty";
    "FOC_LUT_DQLimiter_DQ_LUTScaling",      "FOC.LUT.DQLimiter.DQ_LUTScaling";
    "FOC_Control_RF_omegamech_rad_s",       "FOC.Control.RF_omegamech_rad_s";
    "FOC_SampleTime_TsCur",                 "FOC.SampleTime.TsCur";
    "FOC_Control_theta_mechOffsetFix",      "FOC.Control.theta_mechOffsetFix";
    "FOC_Control_theta_mechOffsetTime",     "FOC.Control.theta_mechOffsetTime";
    "FOC_PWM_HybridMode",                   "FOC.PWM.HybridMode";
    "FOC_PWM_PWMManuel",                    "FOC.PWM.PWMManuel";
    "FOC_PWM_AZSPWMMethod",                 "FOC.PWM.AZSPWMMethod";
    "FOC_PWM_RSPWMMethod",                  "FOC.PWM.RSPWMMethod";
    "FOC_PWM_LB1",                          "FOC.PWM.LB1";
    "FOC_PWM_UB1",                          "FOC.PWM.UB1";
    "FOC_PWM_LB2",                          "FOC.PWM.LB2";
    "FOC_PWM_UB2",                          "FOC.PWM.UB2";
    "FOC_LUT_ModIndex_SqrtLutx",            "FOC.LUT.ModIndex.SqrtLutx";
    "FOC_LUT_ModIndex_SqrtLuty",            "FOC.LUT.ModIndex.SqrtLuty";
    "FOC_Flag_SpeedControl",                "FOC.Flag.SpeedControl"
    "FOC_Plant_Motor_Ld",                   "FOC.Plant.Motor.Ld";
    "FOC_Plant_Motor_Lq",                   "FOC.Plant.Motor.Lq";
    "FOC_LUT_ParkClake_AtanLut",            "FOC.LUT.ParkClake.AtanLut";
    "Filter_Current_SensorTimeConst",       "FOC.Filter.Current.SensorTimeConst";
    "Filter_Current_FilterTimeConst",       "FOC.Filter.Current.FilterTimeConst";
    };
end

function rules = loadScalingRules()
rules = struct("PathPattern", {}, "DataType", {});

if exist("ScalingRules", "file") ~= 2
    return
end

try
    ScalingRules;
catch ME
    warning("createFOC_SLDD:ScalingRulesFailed", ...
        "ScalingRules could not be loaded: %s", ME.message);
end
end

function value = getStructPathValue(rootStruct, structPath)
parts = split(string(structPath), ".");
value = rootStruct;

for i = 2:numel(parts)
    fieldName = char(parts(i));
    if isstruct(value)
        value = value.(fieldName);
    elseif isa(value, "Simulink.Parameter")
        value = value.Value.(fieldName);
    else
        error("createFOC_SLDD:InvalidPath", ...
            "Cannot resolve path '%s'.", structPath);
    end
end
end

function dataType = findRuleDataType(structPath, rules)
dataType = "";

if isempty(rules)
    return
end

if exist("findDataTypeForPath", "file") == 2
    dataType = string(findDataTypeForPath(structPath, rules));
    return
end

path = char(string(structPath));
for i = 1:numel(rules)
    expression = ["^" regexptranslate("wildcard", rules(i).PathPattern) "$"];
    if ~isempty(regexp(path, expression, "once"))
        dataType = string(rules(i).DataType);
        return
    end
end
end

function param = makeParam(value)
if isa(value, "Simulink.Parameter")
    param = value;
else
    param = Simulink.Parameter;
    param.Value = value;
    param.CoderInfo.StorageClass = "ExportedGlobal";
end
end

function closeDictionaryConnections(slddPath)
try
    [~, dictName, dictExt] = fileparts(slddPath);
    Simulink.data.dictionary.closeAll([dictName dictExt], "-discard");
catch ME
    warning("createFOC_SLDD:DictionaryCloseFailed", ...
        "Dictionary could not be closed before creation: %s", ME.message);
end
end

function closeDictionary(dd)
try
    if dd.isOpen
        dd.close;
    end
catch
end
end

function linkDictionaryToModel(modelName, slddPath)
try
    load_system(modelName);
    set_param(modelName, "DataDictionary", slddPath);
    save_system(modelName);
    fprintf("Model '%s' linked to dictionary.\n", modelName);
catch ME
    fprintf("Model could not be linked: %s\n", ME.message);
    fprintf("Manual link:\n");
    fprintf("  set_param('%s', 'DataDictionary', '%s')\n", modelName, slddPath);
end
end
