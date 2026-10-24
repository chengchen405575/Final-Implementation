function plot_embedded_H_analysis(dataFile,outputDirectory,torqueLabel,snapshotTime)
%PLOT_EMBEDDED_H_ANALYSIS Create reproducible embedded-H comparison figures.
if nargin < 1
    dataFile = fullfile(pwd,'embedded_H.mldatx');
end
if nargin < 2
    outputDirectory = fullfile(pwd,'figures_embedded_H');
end
if nargin < 3
    torqueLabel = '90NM';
end
if nargin < 4
    snapshotTime = 0.5;
end
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

Simulink.sdi.clear;
Simulink.sdi.load(dataFile);
runIds = Simulink.sdi.getAllRunIDs;
runNames = strings(numel(runIds),1);
for k = 1:numel(runIds)
    runNames(k) = string(Simulink.sdi.getRun(runIds(k)).Name);
end
embeddedRun = Simulink.sdi.getRun(runIds(runNames == "embedded_H_"+torqueLabel));
originalRun = Simulink.sdi.getRun(runIds(runNames == "original_"+torqueLabel));

[Hembedded,tHembedded] = signal_data(embeddedRun,'SDA_input.H');
[Horiginal,tHoriginal] = signal_data(originalRun,'SDA_input.H');
Hembedded = matrix_sample(Hembedded,tHembedded,snapshotTime);
Horiginal = matrix_sample(Horiginal,tHoriginal,snapshotTime);

rowOriginal = vecnorm(Horiginal,2,2);
rowEmbedded = vecnorm(Hembedded,2,2);
energyOriginal = cumsum(rowOriginal.^2);
energyEmbedded = cumsum(rowEmbedded.^2);
depth = (1:size(Horiginal,1)).';

figure('Color','w','Position',[100 100 1180 430]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot(depth,rowOriginal,'-o','LineWidth',1.5,'MarkerSize',4); hold on;
plot(depth,rowEmbedded,'-s','LineWidth',1.5,'MarkerSize',4);
xlabel('SDA depth i'); ylabel('$\|H(i,:)\|_2$','Interpreter','latex');
grid on; legend('Branch penalty','Embedded H','Location','best');
title(sprintf('Row norms at t = %.3f s (%s)',snapshotTime,torqueLabel));
nexttile;
plot(depth,energyOriginal,'-o','LineWidth',1.5,'MarkerSize',4); hold on;
plot(depth,energyEmbedded,'-s','LineWidth',1.5,'MarkerSize',4);
xlabel('SDA depth i'); ylabel('$\sum_{\ell=1}^{i}\|H(\ell,:)\|_2^2$', ...
    'Interpreter','latex');
grid on; legend('Branch penalty','Embedded H','Location','best');
title('Cumulative row energy');
exportgraphics(gcf,fullfile(outputDirectory, ...
    'H_row_norm_and_cumulative_energy_90Nm.png'),'Resolution',300);
close(gcf);

[UuncEmbedded,tEmbedded] = signal_data(embeddedRun,'SDA_input.U_unc');
[UuncOriginal,tOriginal] = signal_data(originalRun,'SDA_input.U_unc');
[UoptEmbedded,tUoptEmbedded] = signal_data(embeddedRun, ...
    'SDA_reference_output.Uopt');
[UoptOriginal,tUoptOriginal] = signal_data(originalRun, ...
    'SDA_reference_output.Uopt');
UuncEmbedded = vector_sample(UuncEmbedded,tEmbedded,snapshotTime);
UuncOriginal = vector_sample(UuncOriginal,tOriginal,snapshotTime);
UoptEmbedded = vector_sample(UoptEmbedded,tUoptEmbedded,snapshotTime);
UoptOriginal = vector_sample(UoptOriginal,tUoptOriginal,snapshotTime);

candidateNames = {'Optimal','Babai','First block 000','First block 111'};
candidatesOriginal = candidate_set(Horiginal,UuncOriginal,UoptOriginal);
candidatesEmbedded = candidate_set(Hembedded,UuncEmbedded,UoptEmbedded);
lambdaZero = 1.0e4;
partialOriginal = zeros(15,4);
partialEmbedded = zeros(15,4);
for c = 1:4
    partialOriginal(:,c) = partial_metric(Horiginal,UuncOriginal, ...
        candidatesOriginal(:,c),lambdaZero);
    partialEmbedded(:,c) = partial_metric(Hembedded,UuncEmbedded, ...
        candidatesEmbedded(:,c),0.0);
end
[JroundOriginal,tj] = signal_data(originalRun,'SDA_reference_output.Jround');
[JshiftOriginal,~] = signal_data(originalRun,'SDA_reference_output.Jshift');
[JbabaiOriginal,~] = signal_data(originalRun,'SDA_reference_output.Jbabai');
[JroundEmbedded,te] = signal_data(embeddedRun,'SDA_reference_output.Jround');
[JshiftEmbedded,~] = signal_data(embeddedRun,'SDA_reference_output.Jshift');
[JbabaiEmbedded,~] = signal_data(embeddedRun,'SDA_reference_output.Jbabai');
radiusOriginal = min([scalar_sample(JroundOriginal,tj,snapshotTime), ...
    scalar_sample(JshiftOriginal,tj,snapshotTime), ...
    scalar_sample(JbabaiOriginal,tj,snapshotTime)]);
radiusEmbedded = min([scalar_sample(JroundEmbedded,te,snapshotTime), ...
    scalar_sample(JshiftEmbedded,te,snapshotTime), ...
    scalar_sample(JbabaiEmbedded,te,snapshotTime)]);

figure('Color','w','Position',[100 100 1180 470]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot_partial_panel(depth,partialOriginal,radiusOriginal,candidateNames, ...
    'Explicit branch penalty');
nexttile;
plot_partial_panel(depth,partialEmbedded,radiusEmbedded,candidateNames, ...
    'Embedded quadratic penalty');
exportgraphics(gcf,fullfile(outputDirectory, ...
    'partial_metric_and_initial_radius_90Nm.png'),'Resolution',300);
close(gcf);
end

function [data,time] = signal_data(run,signalName)
signals = run.getAllSignals;
names = string({signals.Name});
index = find(names == string(signalName),1);
assert(~isempty(index),'Signal not found: %s',signalName);
values = signals(index).Values;
data = squeeze(values.Data);
time = values.Time;
end

function value = matrix_sample(data,time,targetTime)
[~,index] = min(abs(time-targetTime));
if ndims(data) == 3 && size(data,3) == numel(time)
    value = double(data(:,:,index));
elseif ndims(data) == 3 && size(data,1) == numel(time)
    value = double(squeeze(data(index,:,:)));
else
    error('Unsupported matrix signal dimensions.');
end
end

function value = vector_sample(data,time,targetTime)
[~,index] = min(abs(time-targetTime));
if size(data,1) == numel(time)
    value = double(data(index,:)).';
else
    value = double(data(:,index));
end
value = value(:);
end

function value = scalar_sample(data,time,targetTime)
[~,index] = min(abs(time-targetTime));
value = double(data(index));
end

function candidates = candidate_set(H,Uunc,Uopt)
Ubabai = zeros(size(Uunc));
z = H*Uunc;
for i = 1:numel(Uunc)
    head = H(i,1:i-1)*Ubabai(1:i-1);
    center = (z(i)-head)/H(i,i);
    Ubabai(i) = double(center > 0.5);
end
Uzero = Uopt;
Uzero(1:3) = 0.0;
Uone = Uopt;
Uone(1:3) = 1.0;
candidates = [Uopt Ubabai Uzero Uone];
end

function metric = partial_metric(H,Uunc,U,lambdaZero)
z = H*Uunc;
metric = zeros(numel(U),1);
accumulator = 0.0;
for i = 1:numel(U)
    residual = H(i,1:i)*U(1:i)-z(i);
    accumulator = accumulator+residual^2;
    if mod(i,3) == 0
        block = U(i-2:i);
        if all(block==0.0) || all(block==1.0)
            accumulator = accumulator+lambdaZero;
        end
    end
    metric(i) = accumulator;
end
end

function plot_partial_panel(depth,metrics,radius,names,panelTitle)
plot(depth,metrics,'LineWidth',1.4); hold on;
yline(radius,'k--','Initial quadratic radius','LineWidth',1.2);
for boundary = 3:3:15
    xline(boundary,':','Color',[0.5 0.5 0.5]);
end
xlabel('SDA depth i'); ylabel('Accumulated partial metric');
title(panelTitle); grid on;
legend([names {'Initial radius'}],'Location','northwest');
end
