function runFaultSweep(numCases, options)
% runFaultSweep  Simulate labeled faults on the IEEE 9-bus model and save waveforms.
%
%   runFaultSweep(numCases) plans numCases fault cases with balanced classes
%   (AG, BG, CG, AB, BC, CA, ABG, BCG, CAG, ABC, NF), simulates each one on
%   IEEE9BusFaults.slx (built by buildFaultModel), and writes the dataset to
%   the output folder:
%
%     cases.csv              Full case plan (written once, reused on resume)
%     sweep_settings.json    Settings the plan was drawn with
%     index.csv              One row per finished case, with its label and settings
%     failures.csv           Cases whose simulation raised an error
%     waveforms/case_NNNNNN.csv
%                            Relay window of the faulted line: time_s (relative to
%                            fault inception), Va Vb Vc in volts, Ia Ib Ic in amperes
%
%   Each case draws a line, a fault position from FaultPositions, a load level
%   from LoadLevels, an inception angle in [0, 360) degrees, a per-phase fault
%   resistance, and (for grounded faults) a ground resistance. NF cases save a
%   window from a random line.
%
%   Rerunning with the same arguments skips cases already in index.csv, so an
%   interrupted sweep continues where it stopped. Rerunning into a folder that
%   was planned with different settings is an error.
%
%   To stop a running sweep, press Ctrl+C, or create a file named STOP in the
%   output folder; the sweep then exits after the current case and deletes it.
%
%   Name-value options:
%     ModelFile             Fault model (default "IEEE9BusFaults.slx")
%     OutputFolder          Dataset folder (default "dataset")
%     Seed                  Random seed for the case plan (default 360)
%     FaultPositions        Allowed fault positions along a line, as fractions
%                           from the sending bus (default 0.1:0.2:0.9)
%     LoadLevels            Allowed multipliers on loads A, B, and C
%                           (default [0.85 0.925 1 1.075 1.15])
%     FaultResistanceRange  Per-phase fault resistance range in ohms, sampled
%                           log-uniformly (default [0.01 20])
%     GroundResistanceRange Ground resistance range in ohms for grounded faults,
%                           sampled log-uniformly (default [0.01 300])
%     PreFaultCycles        Cycles kept before inception (default 1)
%     PostFaultCycles       Cycles kept after inception (default 2)
%
%   Example:
%     buildFaultModel("IEEE9BusSystem.slx", "IEEE9BusFaults.slx")
%     runFaultSweep(22)      % two cases per class as a smoke test
%     runFaultSweep(3300)    % full dataset: 300 cases per class

    arguments
        numCases (1,1) double {mustBeInteger, mustBePositive}
        options.ModelFile (1,1) string {mustBeFile} = "IEEE9BusFaults.slx"
        options.OutputFolder (1,1) string = "dataset"
        options.Seed (1,1) double {mustBeInteger, mustBeNonnegative} = 360
        options.FaultPositions (1,:) double {mustBeInRange(options.FaultPositions, 0, 1, "exclusive")} = 0.1:0.2:0.9
        options.LoadLevels (1,:) double {mustBePositive} = [0.85 0.925 1 1.075 1.15]
        options.FaultResistanceRange (1,2) double {mustBePositive} = [0.01 20]
        options.GroundResistanceRange (1,2) double {mustBePositive} = [0.01 300]
        options.PreFaultCycles (1,1) double {mustBeInteger, mustBePositive} = 1
        options.PostFaultCycles (1,1) double {mustBeInteger, mustBePositive} = 2
    end

    [~, model] = fileparts(options.ModelFile);
    load_system(options.ModelFile);
    restoreModel = onCleanup(@() closeModel(model));
    modelWorkspace = get_param(model, "ModelWorkspace");
    sampleTime = evalin(modelWorkspace, "Ts");
    systemFrequency = 60;
    samplesPerCycle = round(1/(systemFrequency*sampleTime));

    waveformFolder = fullfile(options.OutputFolder, "waveforms");
    if ~isfolder(waveformFolder)
        mkdir(waveformFolder);
    end

    cases = loadOrPlanCases(numCases, options, samplesPerCycle, sampleTime);
    indexFile = fullfile(options.OutputFolder, "index.csv");
    failureFile = fullfile(options.OutputFolder, "failures.csv");
    stopFile = fullfile(options.OutputFolder, "STOP");
    remaining = cases(~ismember(cases.caseId, finishedCaseIds(indexFile)), :);
    fprintf("%d of %d cases left to simulate.\n", height(remaining), height(cases));
    if height(remaining) == 0
        return
    end

    preSamples = options.PreFaultCycles*samplesPerCycle;
    postSamples = options.PostFaultCycles*samplesPerCycle;
    stopTime = (max(cases.startSample) + postSamples + 1)*sampleTime;
    sites = faultSiteNames();
    sweepTimer = tic();
    numDone = 0;

    groups = unique(remaining(:, ["loadScale", "faultPosition"]), "rows");
    for g = 1:height(groups)
        inGroup = abs(remaining.loadScale - groups.loadScale(g)) < 1e-9 & ...
            abs(remaining.faultPosition - groups.faultPosition(g)) < 1e-9;
        group = remaining(inGroup, :);
        setCompileTimeSettings(model, modelWorkspace, sites, groups(g, :));
        fprintf("Load %.3f, fault position %.2f: %d cases (compiling on the first one).\n", ...
            groups.loadScale(g), groups.faultPosition(g), height(group));

        for k = 1:height(group)
            caseRow = group(k, :);
            simulationInput = buildSimulationInput(model, caseRow, sites, sampleTime, stopTime);
            try
                simulationOutput = sim(simulationInput);
                if ~isempty(simulationOutput.ErrorMessage)
                    error("runFaultSweep:simulationFailed", "%s", simulationOutput.ErrorMessage);
                end
                waveform = extractWindow(simulationOutput, caseRow, preSamples, postSamples, sampleTime);
                waveformFile = fullfile("waveforms", sprintf("case_%06d.csv", caseRow.caseId));
                writetable(waveform, fullfile(options.OutputFolder, waveformFile));
                appendRow(indexFile, indexRow(caseRow, waveformFile));
            catch simulationError
                if isUserInterrupt(simulationError)
                    fprintf("Sweep stopped by Ctrl+C. Rerun the same command to continue.\n");
                    return
                end
                warning("runFaultSweep:caseSkipped", "Case %d failed and was skipped: %s", ...
                    caseRow.caseId, simulationError.message);
                appendRow(failureFile, table(caseRow.caseId, string(simulationError.message), ...
                    VariableNames=["caseId", "message"]));
            end

            numDone = numDone + 1;
            secondsPerCase = toc(sweepTimer)/numDone;
            fprintf("  case %6d  %-3s %s  (%d/%d, %.1f s/case, about %.1f h left)\n", ...
                caseRow.caseId, caseRow.label, caseRow.faultLine, numDone, height(remaining), ...
                secondsPerCase, secondsPerCase*(height(remaining) - numDone)/3600);

            if isfile(stopFile)
                delete(stopFile);
                fprintf("Sweep stopped by the STOP file. Rerun the same command to continue.\n");
                return
            end
        end
    end
    fprintf("Sweep finished in %.1f min. Index: %s\n", toc(sweepTimer)/60, indexFile);
end

function cases = loadOrPlanCases(numCases, options, samplesPerCycle, sampleTime)
% loadOrPlanCases  Reuse an existing case plan or draw a new one.
    planFile = fullfile(options.OutputFolder, "cases.csv");
    settingsFile = fullfile(options.OutputFolder, "sweep_settings.json");
    settings = options;
    settings.NumCases = numCases;
    settings = rmfield(settings, ["ModelFile", "OutputFolder"]);
    settingsText = jsonencode(settings, PrettyPrint=true);
    if isfile(planFile)
        if ~isfile(settingsFile) || ~isequal(jsondecode(fileread(settingsFile)), jsondecode(settingsText))
            error("runFaultSweep:planMismatch", ...
                "%s was planned with different settings. " + ...
                "Use a new OutputFolder, or delete that folder to start over.", ...
                options.OutputFolder);
        end
        cases = readtable(planFile, TextType="string");
        return
    end

    randomStream = RandStream("mt19937ar", Seed=options.Seed);
    classes = faultClasses();
    classIndex = mod(randperm(randomStream, numCases) - 1, height(classes)) + 1;
    sites = faultSiteNames();

    caseId = (1:numCases)';
    label = classes.label(classIndex);
    faultLine = reshape(sites(randi(randomStream, numel(sites), numCases, 1)), [], 1);
    faultPosition = reshape(options.FaultPositions(randi(randomStream, ...
        numel(options.FaultPositions), numCases, 1)), [], 1);
    loadScale = reshape(options.LoadLevels(randi(randomStream, ...
        numel(options.LoadLevels), numCases, 1)), [], 1);
    inceptionSample = randi(randomStream, samplesPerCycle, numCases, 1) - 1;
    inceptionAngleDeg = 360*inceptionSample/samplesPerCycle;
    % Fault inception lands two cycles after t = 0 plus the inception offset.
    startSample = 2*samplesPerCycle + inceptionSample;
    faultStartTime = startSample*sampleTime;
    faultResistance = logUniform(randomStream, options.FaultResistanceRange, numCases);
    groundResistance = logUniform(randomStream, options.GroundResistanceRange, numCases);
    isGrounded = classes.flags(classIndex, 4) == 1;
    groundResistance(~isGrounded) = NaN;
    isFaultFree = label == "NF";
    faultResistance(isFaultFree) = NaN;

    cases = table(caseId, label, faultLine, faultPosition, loadScale, inceptionAngleDeg, ...
        faultStartTime, startSample, faultResistance, groundResistance);
    writetable(cases, planFile);
    writelines(settingsText, settingsFile);
end

function values = logUniform(randomStream, range, count)
% logUniform  Draw count values spread evenly in log scale across range.
    logRange = log10(range);
    values = 10.^(logRange(1) + (logRange(2) - logRange(1))*rand(randomStream, count, 1));
end

function classes = faultClasses()
% faultClasses  Fault labels and which of [a b c ground] each one involves.
    label = ["AG"; "BG"; "CG"; "AB"; "BC"; "CA"; "ABG"; "BCG"; "CAG"; "ABC"; "NF"];
    flags = [1 0 0 1; 0 1 0 1; 0 0 1 1; 1 1 0 0; 0 1 1 0; 1 0 1 0; ...
        1 1 0 1; 0 1 1 1; 1 0 1 1; 1 1 1 0; 0 0 0 0];
    classes = table(label, flags);
end

function sites = faultSiteNames()
% faultSiteNames  Site tags created by buildFaultModel, one per line.
    sites = ["L45", "L46", "L57", "L69", "L78", "L89"];
end

function setCompileTimeSettings(model, modelWorkspace, sites, group)
% setCompileTimeSettings  Set load level and fault position; this needs a recompile.
    set_param(model, FastRestart="off");
    assignin(modelWorkspace, "loadScale", group.loadScale);
    for site = sites
        assignin(modelWorkspace, "faultPosition_" + site, group.faultPosition);
    end
    set_param(model, FastRestart="on");
end

function simulationInput = buildSimulationInput(model, caseRow, sites, sampleTime, stopTime)
% buildSimulationInput  Set the fault variables for one case.
    classes = faultClasses();
    flags = classes.flags(classes.label == caseRow.label, :);
    faultResistance = fillMissing(caseRow.faultResistance, 1);
    groundResistance = fillMissing(caseRow.groundResistance, 1);

    simulationInput = Simulink.SimulationInput(model);
    simulationInput = simulationInput.setModelParameter(StopTime=string(stopTime));
    for site = sites
        siteFlags = zeros(1, 4);
        if site == caseRow.faultLine
            siteFlags = flags;
        end
        simulationInput = simulationInput.setVariable("faultFlags_" + site, siteFlags, Workspace=model);
    end
    % Half a step early so the first faulted sample is exactly startSample.
    simulationInput = simulationInput.setVariable("faultStartTime", ...
        (caseRow.startSample - 0.5)*sampleTime, Workspace=model);
    simulationInput = simulationInput.setVariable("faultResistance", faultResistance, Workspace=model);
    simulationInput = simulationInput.setVariable("groundResistance", groundResistance, Workspace=model);
end

function isInterrupt = isUserInterrupt(simulationError)
% isUserInterrupt  True when an error came from Ctrl+C rather than the model.
    message = string(simulationError.message);
    isInterrupt = simulationError.identifier == "MATLAB:interrupt" || ...
        contains(message, "Ctrl-C") || contains(message, "Simulation aborted");
end

function value = fillMissing(value, defaultValue)
% fillMissing  Replace NaN (setting unused by this class) with a placeholder.
    if isnan(value)
        value = defaultValue;
    end
end

function waveform = extractWindow(simulationOutput, caseRow, preSamples, postSamples, sampleTime)
% extractWindow  Cut the relay window around inception for the case's line.
    voltage = squeeze(simulationOutput.get("v_" + caseRow.faultLine))';
    current = squeeze(simulationOutput.get("i_" + caseRow.faultLine))';
    % Row r holds time (r - 1)*Ts, so sample s is row s + 1.
    rows = (caseRow.startSample - preSamples + 1):(caseRow.startSample + postSamples);
    time_s = ((rows - 1)' - caseRow.startSample)*sampleTime;
    waveform = table(time_s, voltage(rows, 1), voltage(rows, 2), voltage(rows, 3), ...
        current(rows, 1), current(rows, 2), current(rows, 3), ...
        VariableNames=["time_s", "Va", "Vb", "Vc", "Ia", "Ib", "Ic"]);
end

function row = indexRow(caseRow, waveformFile)
% indexRow  Index entry for one finished case.
    row = removevars(caseRow, "startSample");
    row.file = waveformFile;
end

function ids = finishedCaseIds(indexFile)
% finishedCaseIds  Case IDs already written to the index.
    ids = zeros(0, 1);
    if isfile(indexFile)
        index = readtable(indexFile, TextType="string");
        ids = index.caseId;
    end
end

function appendRow(fileName, row)
% appendRow  Append one table row to a CSV file, writing the header if new.
    writetable(row, fileName, WriteMode="append");
end

function closeModel(model)
% closeModel  Leave the fault model unchanged on disk.
    if bdIsLoaded(model)
        set_param(model, FastRestart="off");
        close_system(model, 0);
    end
end
