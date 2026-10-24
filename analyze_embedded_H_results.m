function stats = analyze_embedded_H_results(fileName)
%ANALYZE_EMBEDDED_H_RESULTS Compare embedded-H and branch-penalty SDI runs.
if nargin < 1
    fileName = fullfile(pwd,'embedded_H.mldatx');
end

Simulink.sdi.clear;
Simulink.sdi.load(fileName);
ids = Simulink.sdi.getAllRunIDs;
names = strings(numel(ids),1);
wanted = {'Jopt','nodes','optimal','Uopt','Jround','Jshift','Jbabai', ...
    'SDA_input.signal8','SDA_input.signal9'};
D = cell(numel(ids),numel(wanted));
T = cell(numel(ids),numel(wanted));
stats = struct;

for k = 1:numel(ids)
    run = Simulink.sdi.getRun(ids(k));
    names(k) = string(run.Name);
    signals = run.getAllSignals;
    signalNames = string({signals.Name});
    for q = 1:numel(wanted)
        target = string(wanted{q});
        if startsWith(target,'SDA_')
            index = find(signalNames == target,1);
        else
            index = find(signalNames == "SDA_reference_output."+target,1);
        end
        if ~isempty(index)
            values = signals(index).Values;
            D{k,q} = squeeze(values.Data);
            T{k,q} = values.Time;
        end
    end

    mask = T{k,1} >= 0.05;
    lambdaZero = median(double(D{k,8}(T{k,8} >= 0.05)));
    lambdaDeadtime = median(double(D{k,9}(T{k,9} >= 0.05)));
    fprintf('\nRUN=%s samples=%d lambdaZero=%g lambdaDeadtime=%g\n', ...
        names(k),sum(mask),lambdaZero,lambdaDeadtime);
    for q = [1 2 3 5 6 7]
        x = double(D{k,q}(mask,:));
        fprintf(' %s mean=%.6g median=%.6g min=%.6g max=%.6g\n', ...
            wanted{q},mean(x(:)),median(x(:)),min(x(:)),max(x(:)));
    end
    candidates = [double(D{k,5}(mask)) double(D{k,6}(mask)) ...
        double(D{k,7}(mask))];
    [~,winner] = min(candidates,[],2);
    fprintf(' WIN round=%.2f%% shift=%.2f%% babai=%.2f%%\n', ...
        100*mean(winner==1),100*mean(winner==2),100*mean(winner==3));
    stats.runs(k).name = names(k);
    stats.runs(k).lambdaZero = lambdaZero;
    stats.runs(k).lambdaDeadtime = lambdaDeadtime;
    stats.runs(k).winnerPercent = 100*[mean(winner==1) ...
        mean(winner==2) mean(winner==3)];
end

pairs = {
    'embedded_H_10NM','original_10NM';
    'embedded_H_50NM','original_50NM';
    'embedded_H_90NM','original_90NM'};
for p = 1:size(pairs,1)
    a = find(names == pairs{p,1});
    b = find(names == pairs{p,2});
    ta = T{a,1};
    tb = T{b,1};
    ia = find(ta >= 0.05);
    ib = find(tb >= 0.05);
    n = min(numel(ia),numel(ib));
    ia = ia(1:n);
    ib = ib(1:n);

    fprintf('\nPAIR %s vs %s n=%d maxDt=%.3g\n', ...
        names(a),names(b),n,max(abs(ta(ia)-tb(ib))));
    jDifference = double(D{a,1}(ia))-double(D{b,1}(ib));
    nodeDifference = double(D{a,2}(ia))-double(D{b,2}(ib));
    fprintf([' JDIFF mean=%.6g median=%.6g std=%.6g ' ...
        'min=%.6g max=%.6g\n'],mean(jDifference),median(jDifference), ...
        std(jDifference),min(jDifference),max(jDifference));
    fprintf([' NODEDIFF mean=%.6g median=%.6g embLess=%.2f%% ' ...
        'same=%.2f%% embMore=%.2f%%\n'],mean(nodeDifference), ...
        median(nodeDifference),100*mean(nodeDifference<0), ...
        100*mean(nodeDifference==0),100*mean(nodeDifference>0));

    ua = D{a,4};
    ub = D{b,4};
    if size(ua,1) == numel(ta)
        ua = ua(ia,:);
        ub = ub(ib,:);
    else
        ua = ua(:,ia).';
        ub = ub(:,ib).';
    end
    identical = all(double(ua)==double(ub),2);
    fprintf(' UOPT_IDENTICAL=%.4f%% elementMismatch=%d\n', ...
        100*mean(identical),nnz(double(ua)~=double(ub)));

    stats.pairs(p).embedded = names(a);
    stats.pairs(p).original = names(b);
    stats.pairs(p).jDifference = jDifference;
    stats.pairs(p).nodeDifference = nodeDifference;
    stats.pairs(p).identicalPercent = 100*mean(identical);
end
end
