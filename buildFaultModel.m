function buildFaultModel(sourceModelFile, targetModelFile, options)
% buildFaultModel  Create a fault-ready copy of the IEEE 9-bus example model.
%
%   buildFaultModel(sourceModelFile, targetModelFile) copies the Simscape
%   Electrical IEEE 9-bus example and edits the copy so that each of the six
%   transmission lines can carry a timed fault at an adjustable location.
%
%   Each line is replaced by a subsystem in the same place, connected to the
%   same two buses and labeled with the line name plus "Fault site <site>".
%   Inside, the line is split into two
%   series sections joined at a fault node. The
%   split position is the model-workspace variable faultPosition_<site>
%   (fraction of line length measured from the sending bus). It is a
%   compile-time value, so changing it needs Fast Restart turned off.
%
%   The fault on each node is four variable resistors: phases a, b, and c to
%   a star point, and the star point to ground. Before faultStartTime every
%   resistor is openResistance. From faultStartTime on, a resistor whose
%   entry in faultFlags_<site> ([a b c g]) is 1 drops to faultResistance
%   (phases) or groundResistance (star to ground). Every fault setting is a
%   Simulink signal, so those settings change freely under Fast Restart.
%
%   A relay is added at the sending end of every line: a three-phase current
%   sensor in series with the line and a phase-to-ground voltage sensor. The
%   relay signals are logged with To Workspace blocks named v_<site> and
%   i_<site>, in volts and amperes.
%
%   The three loads (A, B, C) are scaled together by the model-workspace
%   variable loadScale (1 = the example's 125/90/100 MW). It is compile-time
%   like faultPosition_<site>.
%
%   All flags start at zero, so the saved model has no fault.
%
%   Name-value options:
%     SamplesPerCycle - Samples per 60 Hz cycle used for Ts (default 64).
%
%   Example:
%     buildFaultModel("IEEE9BusSystem.slx", "IEEE9BusFaults.slx")

    arguments
        sourceModelFile (1,1) string {mustBeFile}
        targetModelFile (1,1) string
        options.SamplesPerCycle (1,1) double {mustBeInteger, mustBePositive} = 64
    end

    [targetFolder, targetModel] = fileparts(targetModelFile);
    if targetFolder == ""
        targetFolder = pwd();
    end
    targetPath = fullfile(targetFolder, targetModel + ".slx");

    if bdIsLoaded(targetModel)
        close_system(targetModel, 0);
    end
    copyfile(sourceModelFile, targetPath, "f");
    load_system(targetPath);

    systemFrequency = 60;
    sampleTime = 1/(systemFrequency*options.SamplesPerCycle);
    set_param(targetModel, PreLoadFcn=sprintf("Ts = %.12g;", sampleTime));
    modelWorkspace = get_param(targetModel, "ModelWorkspace");
    assignin(modelWorkspace, "Ts", sampleTime);
    openResistance = 1e6;
    assignin(modelWorkspace, "faultStartTime", 0.05);
    assignin(modelWorkspace, "openResistance", openResistance);
    assignin(modelWorkspace, "faultResistance", 1e-3);
    assignin(modelWorkspace, "groundResistance", 1e-3);

    addLoadScaling(targetModel, modelWorkspace);
    addFaultClock(targetModel);
    sites = faultSiteTable();
    for k = 1:height(sites)
        addFaultSite(targetModel, sites(k, :), modelWorkspace);
    end

    set_param(targetModel, StopTime="0.1", ReturnWorkspaceOutputs="on", ...
        SaveTime="on", SaveOutput="off", SaveState="off", SignalLogging="off");
    save_system(targetModel, targetPath);
    fprintf("Saved %s with %d fault sites (Ts = 1/%d s).\n", targetPath, ...
        height(sites), round(1/sampleTime));
end

function sites = faultSiteTable()
% faultSiteTable  Line block names and site tags used throughout the sweep.
    siteName = ["L45"; "L46"; "L57"; "L69"; "L78"; "L89"];
    lineName = ["B4 to B5"; "B4 to B6"; "B5 to B7"; "B6 to B9"; "B7 to B8"; "B8 to B9"];
    sites = table(siteName, lineName);
end

function addLoadScaling(model, modelWorkspace)
% addLoadScaling  Multiply the P and Q of loads A, B, and C by loadScale.
    assignin(modelWorkspace, "loadScale", 1);
    loadBlocks = find_system(model, SearchDepth=1, Regexp="on", Name="^Load [ABC]\n");
    if numel(loadBlocks) ~= 3
        error("buildFaultModel:loadsNotFound", ...
            "Expected loads A, B, and C in %s but found %d. " + ...
            "Use the unmodified IEEE9BusSystem example as the source model.", ...
            model, numel(loadBlocks));
    end
    for k = 1:numel(loadBlocks)
        for parameter = ["P", "Qpos"]
            nominalValue = get_param(loadBlocks{k}, parameter);
            set_param(loadBlocks{k}, parameter, "(" + nominalValue + ")*loadScale");
        end
    end
end

function addFaultClock(model)
% addFaultClock  Add the shared faultActive signal, 1 once faultStartTime is reached.
    origin = [-1400, -400];
    clockBlock = add_block("simulink/Sources/Clock", model + "/Fault clock", ...
        Position=offsetPosition(origin, [0, 0], [30, 30]));
    startBlock = add_block("simulink/Sources/Constant", model + "/Fault start time", ...
        Position=offsetPosition(origin, [0, 60], [80, 30]));
    set_param(startBlock, Value="faultStartTime");
    compareBlock = add_block("simulink/Logic and Bit Operations/Relational Operator", ...
        model + "/Fault started", Position=offsetPosition(origin, [130, 20], [40, 40]));
    set_param(compareBlock, Operator=">=");
    convertBlock = add_block("simulink/Signal Attributes/Data Type Conversion", ...
        model + "/Fault active", Position=offsetPosition(origin, [210, 25], [60, 30]));
    set_param(convertBlock, OutDataTypeStr="double");

    gotoBlock = add_block("simulink/Signal Routing/Goto", model + "/Fault active tag", ...
        Position=offsetPosition(origin, [300, 25], [80, 30]));
    set_param(gotoBlock, GotoTag=faultActiveTag(), TagVisibility="global");

    add_line(model, portOf(clockBlock, "Outport", 1), portOf(compareBlock, "Inport", 1));
    add_line(model, portOf(startBlock, "Outport", 1), portOf(compareBlock, "Inport", 2));
    add_line(model, portOf(compareBlock, "Outport", 1), portOf(convertBlock, "Inport", 1));
    add_line(model, portOf(convertBlock, "Outport", 1), portOf(gotoBlock, "Inport", 1));
end

function tag = faultActiveTag()
% faultActiveTag  Goto/From tag that carries the shared fault-active signal.
    tag = "faultActive";
end

function addFaultSite(model, site, modelWorkspace)
% addFaultSite  Replace one line with a subsystem holding the split line, fault, and relay.
    lineBlock = findLineBlock(model, site.lineName);
    tag = site.siteName;

    lineLength = str2double(get_param(lineBlock, "length"));
    lengthVariable = "lineLength_" + tag;
    positionVariable = "faultPosition_" + tag;
    assignin(modelWorkspace, lengthVariable, lineLength);
    assignin(modelWorkspace, positionVariable, 0.5);
    assignin(modelWorkspace, "faultFlags_" + tag, zeros(1, 4));

    linePorts = get_param(lineBlock, "PortHandles");
    sendingBusPort = disconnectPort(model, linePorts.LConn(1));
    receivingBusPort = disconnectPort(model, linePorts.RConn(1));
    deleteGroundReference(linePorts.LConn(2));
    deleteGroundReference(linePorts.RConn(2));

    lineLabel = string(get_param(lineBlock, "Name"));
    sitePath = model + "/" + lineLabel + newline + "Fault site " + tag;
    siteBlock = add_block("simulink/Ports & Subsystems/Subsystem", sitePath, ...
        Position=get_param(lineBlock, "Position"));
    Simulink.SubSystem.deleteContents(siteBlock);
    set_param(siteBlock, Orientation=get_param(lineBlock, "Orientation"), ...
        NameLocation=get_param(lineBlock, "NameLocation"));

    origin = [340, 172];
    sendingPort = addConnectionPort(sitePath, "~1", "Left", origin + [-300, 20]);
    receivingPort = addConnectionPort(sitePath, "~2", "Right", origin + [380, 20]);

    % Line lengths must stay compile-time: with run-time lengths the load-flow
    % initialization no longer starts the machines in steady state.
    nearSection = add_block(lineBlock, sitePath + "/Near section", ...
        Position=offsetPosition(origin, [0, 0], [55, 55]), BlockRotation=0, BlockMirror="off");
    set_param(nearSection, length=lengthVariable + "*" + positionVariable);
    farSection = add_block(lineBlock, sitePath + "/Far section", ...
        Position=offsetPosition(origin, [200, 0], [55, 55]), BlockRotation=0, BlockMirror="off");
    set_param(farSection, length=lengthVariable + "*(1 - " + positionVariable + ")");
    delete_block(lineBlock);

    nearPorts = get_param(nearSection, "PortHandles");
    farPorts = get_param(farSection, "PortHandles");
    groundPort(sitePath, nearPorts.LConn(2), origin + [-20, 90], "Near g1");
    groundPort(sitePath, nearPorts.RConn(2), origin + [60, 90], "Near g2");
    groundPort(sitePath, farPorts.LConn(2), origin + [180, 90], "Far g1");
    groundPort(sitePath, farPorts.RConn(2), origin + [260, 90], "Far g2");
    add_line(sitePath, nearPorts.RConn(1), farPorts.LConn(1), autorouting="on");
    add_line(sitePath, farPorts.RConn(1), receivingPort, autorouting="on");

    addFaultResistors(sitePath, tag, origin, nearPorts.RConn(1));
    addRelay(sitePath, tag, origin, sendingPort, nearPorts.LConn(1));

    sitePorts = get_param(siteBlock, "PortHandles");
    add_line(model, sendingBusPort, sitePorts.LConn(1), autorouting="smart");
    add_line(model, sitePorts.RConn(1), receivingBusPort, autorouting="smart");
end

function port = addConnectionPort(system, name, side, position)
% addConnectionPort  Add a physical subsystem port and return its inner port handle.
    portBlock = add_block("nesl_utility/Connection Port", system + "/" + name, ...
        Position=[position, position + [30, 16]]);
    set_param(portBlock, Side=side);
    ports = get_param(portBlock, "PortHandles");
    port = [ports.LConn, ports.RConn];
end

function deleteGroundReference(portHandle)
% deleteGroundReference  Remove the Electrical Reference block wired to a line ground port.
    lineHandle = get_param(portHandle, "Line");
    endpoints = [get_param(lineHandle, "SrcPortHandle"), get_param(lineHandle, "DstPortHandle")];
    referenceBlock = get_param(endpoints(endpoints ~= portHandle), "Parent");
    delete_line(lineHandle);
    delete_block(referenceBlock);
end

function addFaultResistors(model, tag, origin, faultNodePort)
% addFaultResistors  Attach a-b-c-star and star-ground variable resistors to a node.
    splitter = add_block("ee_lib/Connectors &" + newline + "References/Phase Splitter", ...
        model + "/Fault split", Position=offsetPosition(origin, [100, 150], [30, 60]));
    add_line(model, portOf(splitter, "LConn", 1), faultNodePort, autorouting="on");

    activeBlock = add_block("simulink/Signal Routing/From", model + "/Fault active", ...
        Position=offsetPosition(origin, [-290, 230], [100, 30]));
    set_param(activeBlock, GotoTag=faultActiveTag());
    flagsBlock = add_block("simulink/Sources/Constant", model + "/Fault flags", ...
        Position=offsetPosition(origin, [-290, 280], [100, 30]));
    set_param(flagsBlock, Value="faultFlags_" + tag);
    involvedBlock = add_block("simulink/Math Operations/Product", ...
        model + "/Fault involved", Position=offsetPosition(origin, [-150, 250], [30, 40]));
    resistanceBlock = add_block("simulink/Sources/Constant", model + "/Fault resistances", ...
        Position=offsetPosition(origin, [-290, 340], [200, 30]));
    set_param(resistanceBlock, Value="[faultResistance faultResistance faultResistance groundResistance]");
    openBlock = add_block("simulink/Sources/Constant", model + "/Fault open", ...
        Position=offsetPosition(origin, [-290, 400], [100, 30]));
    set_param(openBlock, Value="openResistance");
    switchBlock = add_block("simulink/Signal Routing/Switch", model + "/Fault select", ...
        Position=offsetPosition(origin, [-60, 330], [40, 70]));
    set_param(switchBlock, Criteria="u2 > Threshold", Threshold="0.5");
    splitBlock = add_block("simulink/Signal Routing/Demux", model + "/Fault demux", ...
        Position=offsetPosition(origin, [20, 300], [5, 130]));
    set_param(splitBlock, Outputs="4");

    add_line(model, portOf(activeBlock, "Outport", 1), portOf(involvedBlock, "Inport", 1), autorouting="on");
    add_line(model, portOf(flagsBlock, "Outport", 1), portOf(involvedBlock, "Inport", 2), autorouting="on");
    add_line(model, portOf(resistanceBlock, "Outport", 1), portOf(switchBlock, "Inport", 1), autorouting="on");
    add_line(model, portOf(involvedBlock, "Outport", 1), portOf(switchBlock, "Inport", 2), autorouting="on");
    add_line(model, portOf(openBlock, "Outport", 1), portOf(switchBlock, "Inport", 3), autorouting="on");
    add_line(model, portOf(switchBlock, "Outport", 1), portOf(splitBlock, "Inport", 1), autorouting="on");

    starPort = [];
    resistorNames = ["a", "b", "c", "g"];
    for k = 1:numel(resistorNames)
        resistor = add_block("fl_lib/Electrical/Electrical Elements/Variable Resistor", ...
            model + "/Fault R" + resistorNames(k), ...
            Position=offsetPosition(origin, [250, 100 + 60*k], [40, 30]));
        converter = add_block("nesl_utility/Simulink-PS Converter", ...
            model + "/Fault PS R" + resistorNames(k), ...
            Position=offsetPosition(origin, [175, 75 + 60*k], [20, 16]));
        set_param(converter, Unit="Ohm");
        add_line(model, portOf(splitBlock, "Outport", k), portOf(converter, "Inport", 1), autorouting="on");
        add_line(model, portOf(converter, "RConn", 1), portOf(resistor, "LConn", 1), autorouting="on");

        if k <= 3
            add_line(model, portOf(splitter, "RConn", k), portOf(resistor, "LConn", 2), autorouting="on");
            if isempty(starPort)
                starPort = portOf(resistor, "RConn", 1);
            else
                add_line(model, portOf(resistor, "RConn", 1), starPort, autorouting="on");
            end
        else
            add_line(model, starPort, portOf(resistor, "LConn", 2), autorouting="on");
            groundPort(model, portOf(resistor, "RConn", 1), origin + [330, 380], "Fault ground");
        end
    end
end

function port = portOf(block, portType, index)
% portOf  Return one port handle of a block.
    ports = get_param(block, "PortHandles");
    port = ports.(portType)(index);
end

function lineBlock = findLineBlock(model, lineName)
% findLineBlock  Return the handle of the transmission line with this name.
    candidates = find_system(model, SearchDepth=1, Regexp="on", ...
        Name="^" + lineName + "\n");
    if numel(candidates) ~= 1
        error("buildFaultModel:lineNotFound", ...
            "Expected one block named ""%s"" in %s but found %d. " + ...
            "Use the unmodified IEEE9BusSystem example as the source model.", ...
            lineName, model, numel(candidates));
    end
    lineBlock = get_param(candidates{1}, "Handle");
end

function otherPort = disconnectPort(model, portHandle)
% disconnectPort  Delete the wire on a physical port and return the far port.
    lineHandle = get_param(portHandle, "Line");
    if lineHandle == -1
        error("buildFaultModel:portNotConnected", ...
            "A transmission line port in %s is not connected. " + ...
            "Use the unmodified IEEE9BusSystem example as the source model.", model);
    end
    endpoints = [get_param(lineHandle, "SrcPortHandle"), get_param(lineHandle, "DstPortHandle")];
    otherPort = endpoints(endpoints ~= portHandle);
    if numel(otherPort) ~= 1
        error("buildFaultModel:branchedConnection", ...
            "A transmission line port in %s has a branched wire, which this script cannot rewire.", model);
    end
    delete_line(lineHandle);
end

function addRelay(model, tag, origin, busPort, lineSendingPort)
% addRelay  Insert current and voltage sensors between a bus and a line.
    sensorLibrary = "ee_lib/Sensors &" + newline + "Transducers/";
    currentSensor = add_block(sensorLibrary + "Current Sensor" + newline + "(Three-Phase)", ...
        model + "/Relay I", Position=offsetPosition(origin, [-200, 8], [40, 40]));
    voltageSensor = add_block(sensorLibrary + "Phase Voltage Sensor" + newline + "(Three-Phase)", ...
        model + "/Relay V", Position=offsetPosition(origin, [-90, -80], [40, 40]));
    neutralBlock = add_block("ee_lib/Connectors &" + newline + "References/Grounded Neutral" + ...
        newline + "(Three-Phase)", model + "/Relay N", ...
        Position=offsetPosition(origin, [20, -70], [20, 20]));

    currentPorts = get_param(currentSensor, "PortHandles");
    voltagePorts = get_param(voltageSensor, "PortHandles");
    neutralPorts = get_param(neutralBlock, "PortHandles");

    % Right-side ports follow the library order: signal output, then ~2.
    add_line(model, busPort, currentPorts.LConn(1), autorouting="on");
    add_line(model, currentPorts.RConn(2), lineSendingPort, autorouting="on");
    add_line(model, voltagePorts.LConn(1), lineSendingPort, autorouting="on");
    add_line(model, voltagePorts.RConn(2), neutralPorts.LConn(1), autorouting="on");

    logSignal(model, currentPorts.RConn(1), "i_" + tag, "A", origin + [-230, -140]);
    logSignal(model, voltagePorts.RConn(1), "v_" + tag, "V", origin + [-90, -140]);
end

function logSignal(model, physicalPort, variableName, unitName, position)
% logSignal  Convert a physical signal and log it to the simulation output.
    converter = add_block("nesl_utility/PS-Simulink Converter", ...
        model + "/PS " + variableName, Position=[position, position + [30, 20]]);
    set_param(converter, Unit=unitName);
    sink = add_block("simulink/Sinks/To Workspace", model + "/Log " + variableName, ...
        Position=[position + [50, 0], position + [110, 20]]);
    set_param(sink, VariableName=variableName, SaveFormat="Array", SampleTime="Ts");
    converterPorts = get_param(converter, "PortHandles");
    sinkPorts = get_param(sink, "PortHandles");
    add_line(model, physicalPort, converterPorts.LConn(1), autorouting="on");
    add_line(model, converterPorts.Outport(1), sinkPorts.Inport(1), autorouting="on");
end

function groundPort(model, portHandle, position, name)
% groundPort  Tie an electrical port to an Electrical Reference block.
    reference = add_block("fl_lib/Electrical/Electrical Elements/Electrical Reference", ...
        model + "/" + name, Position=[position, position + [20, 16]]);
    referencePorts = get_param(reference, "PortHandles");
    add_line(model, portHandle, referencePorts.LConn(1), autorouting="on");
end

function position = offsetPosition(origin, offset, blockSize)
% offsetPosition  Build a [left top right bottom] position near an origin.
    topLeft = origin + offset;
    position = [topLeft, topLeft + blockSize];
end
