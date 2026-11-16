function mdl = build_AFS_FinTabSim()
%BUILD_AFS_FINTABSIM Generate the AFS_FinTabSim.slx Simulink model.
%   Builds a 6DOF flight simulation of a rocket whose attitude and roll are
%   controlled by servo-driven tabs on its four fins:
%
%     Roll Program -> Flight Computer (100 Hz) -> Fin Tab Servos -> Rocket Plant
%          ^               ^                                      |
%          |            Sensors <---------------------------------+
%                                    Wind Model ------------------^
%
%   The heavy lifting lives in plain .m functions (afs_aero, afs_eom,
%   afs_controller, ...) that the MATLAB Function blocks call, so the physics
%   can be read, unit-tested and edited without touching the diagram.
%   Parameters come from afs_params() through the base-workspace struct P.

mdl  = 'AFS_FinTabSim';
here = fileparts(mfilename('fullpath'));
addpath(here);
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
f = fullfile(here, [mdl '.slx']);
if isfile(f)
    delete(f);
end

P = afs_params(); %#ok<NASGU>  (lets the diagram compile during the build)
assignin('base', 'P', afs_params());

new_system(mdl);
load_system(mdl);
set_param(mdl, ...
    'Solver', 'ode45', ...
    'StopTime', 'P.sim.tStop', ...
    'MaxStep', 'P.sim.maxStep', ...
    'RelTol', '1e-5', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'ReturnWorkspaceOutputsName', 'out', ...
    'SignalLogging', 'off');
set_param(mdl, 'PostLoadFcn', sprintf([ ...
    'addpath(fileparts(get_param(''%s'',''FileName'')));\n' ...
    'if ~evalin(''base'',''exist(''''P'''',''''var'''')''), assignin(''base'',''P'',afs_params()); end'], mdl));
set_param(mdl, 'InitFcn', ...
    'if ~exist(''P'',''var''), P = afs_params(); end');

%% ---------------------------------------------------------------- top level
add_block('simulink/Sources/Clock', [mdl '/Clock']);

add_block('simulink/Lookup Tables/1-D Lookup Table', [mdl '/Roll Program']);
set_param([mdl '/Roll Program'], ...
    'BreakpointsForDimension1', 'P.ctrl.rollProg.t', ...
    'Table', 'P.ctrl.rollProg.phi', ...
    'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip');

build_wind([mdl '/Wind Model']);
build_sensors([mdl '/Sensors']);
build_flight_computer([mdl '/Flight Computer']);
build_servos([mdl '/Fin Tab Servos']);
build_plant([mdl '/Rocket Plant']);
build_apogee([mdl '/Apogee Stop']);

c = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');
c('Clock/1',            'Roll Program/1');
c('Clock/1',            'Flight Computer/1');
c('Sensors/1',          'Flight Computer/2');
c('Sensors/2',          'Flight Computer/3');
c('Sensors/3',          'Flight Computer/4');
c('Roll Program/1',     'Flight Computer/5');
c('Flight Computer/1',  'Fin Tab Servos/1');
c('Fin Tab Servos/1',   'Rocket Plant/1');
c('Wind Model/1',       'Rocket Plant/2');
c('Clock/1',            'Rocket Plant/3');
c('Rocket Plant/1',     'Sensors/1');
c('Rocket Plant/2',     'Sensors/2');
c('Rocket Plant/1',     'Apogee Stop/1');
c('Clock/1',            'Apogee Stop/2');

% Logging (timeseries in out.<name>)
logs = {'x',        'Rocket Plant/1'
        'aux',      'Rocket Plant/2'
        'onRail',   'Rocket Plant/3'
        'delta',    'Fin Tab Servos/1'
        'deltaCmd', 'Flight Computer/1'
        'ctrl',     'Flight Computer/2'
        'phiCmd',   'Roll Program/1'
        'wind',     'Wind Model/1'};
for k = 1:size(logs, 1)
    b = [mdl '/log_' logs{k, 1}];
    add_block('simulink/Sinks/To Workspace', b, ...
        'VariableName', logs{k, 1}, 'SaveFormat', 'Timeseries');
    c(logs{k, 2}, ['log_' logs{k, 1} '/1']);
end

% Quick-look scopes
add_scope(mdl, 'Tab Deflections', 'Fin Tab Servos/1');
add_scope(mdl, 'Flight Computer Out', 'Flight Computer/2');

Simulink.BlockDiagram.arrangeSystem(mdl);
add_block('built-in/Note', [mdl '/' sprintf(['AFS fin-tab rocket 6DOF. Parameters: afs_params.m (struct P). ' ...
    'Run run_AFS_FinTabSim.m for controlled vs. passive flights + plots.'])], ...
    'Position', [20 -60 520 -40]);

save_system(mdl, f);
fprintf('Saved %s\n', f);
end

%% ===================================================================== wind
function build_wind(sys)
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
add_block('simulink/Sources/Constant', [sys '/Steady Wind'], 'Value', 'P.env.windENU');
add_block('simulink/Sources/Band-Limited White Noise', [sys '/Gust Noise'], ...
    'Cov', 'P.env.gustPower*[1 1 0.3]', 'Ts', '0.05', 'seed', 'P.env.gustSeed');
add_block('simulink/Continuous/State-Space', [sys '/Gust Filter'], ...
    'A', '-eye(3)/P.env.gustTau', 'B', 'eye(3)/P.env.gustTau', ...
    'C', 'eye(3)', 'D', 'zeros(3)');
add_block('simulink/Math Operations/Sum', [sys '/Sum'], 'Inputs', '++');
add_block('simulink/Sinks/Out1', [sys '/wind_ENU']);
c = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
c('Gust Noise/1', 'Gust Filter/1');
c('Steady Wind/1', 'Sum/1');
c('Gust Filter/1', 'Sum/2');
c('Sum/1', 'wind_ENU/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% ================================================================== sensors
function build_sensors(sys)
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
add_block('simulink/Sources/In1', [sys '/x']);
add_block('simulink/Sources/In1', [sys '/aux']);
sel(sys, 'Body Rates', '[11 12 13]', '13');
sel(sys, 'Quaternion', '[7 8 9 10]', '13');
sel(sys, 'Dynamic Pressure', '4', '8');
add_block('simulink/Sources/Constant', [sys '/Gyro Bias'], 'Value', 'P.sens.gyroBias');
add_block('simulink/Sources/Band-Limited White Noise', [sys '/Gyro Noise'], ...
    'Cov', 'P.sens.gyroNoise*[1 1 1]', 'Ts', 'P.sens.Ts', 'seed', 'P.sens.gyroSeed');
add_block('simulink/Sources/Random Number', [sys '/qbar Noise'], ...
    'Variance', 'P.sens.qbarNoise^2', 'SampleTime', 'P.sens.Ts', 'Seed', '31');
add_block('simulink/Math Operations/Sum', [sys '/Gyro Sum'], 'Inputs', '+++');
add_block('simulink/Math Operations/Sum', [sys '/qbar Sum'], 'Inputs', '++');
add_block('simulink/Sinks/Out1', [sys '/gyro']);
add_block('simulink/Sinks/Out1', [sys '/quat']);
add_block('simulink/Sinks/Out1', [sys '/qbar']);
c = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
c('x/1', 'Body Rates/1');
c('x/1', 'Quaternion/1');
c('aux/1', 'Dynamic Pressure/1');
c('Body Rates/1', 'Gyro Sum/1');
c('Gyro Bias/1', 'Gyro Sum/2');
c('Gyro Noise/1', 'Gyro Sum/3');
c('Dynamic Pressure/1', 'qbar Sum/1');
c('qbar Noise/1', 'qbar Sum/2');
c('Gyro Sum/1', 'gyro/1');
c('Quaternion/1', 'quat/1');
c('qbar Sum/1', 'qbar/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% ========================================================== flight computer
function build_flight_computer(sys)
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
names = {'t', 'gyro', 'quat', 'qbar', 'phiCmd'};
for k = 1:numel(names)
    add_block('simulink/Sources/In1', [sys '/' names{k}]);
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/ZOH ' names{k}], ...
        'SampleTime', 'P.sens.Ts');
    add_line(sys, [names{k} '/1'], ['ZOH ' names{k} '/1'], 'autorouting', 'smart');
end
mlfcn([sys '/Controller'], ...
    {'function [deltaCmd, ctrlOut] = fcn(t, gyro, quat, qbar, phiCmd, P)'
     '% Fin-tab flight control law (see afs_controller.m). Runs at P.sens.Ts.'
     '[deltaCmd, ctrlOut] = afs_controller(P, t, gyro, quat, qbar, phiCmd);'});
for k = 1:numel(names)
    add_line(sys, ['ZOH ' names{k} '/1'], sprintf('Controller/%d', k), 'autorouting', 'smart');
end
add_block('simulink/Sinks/Out1', [sys '/deltaCmd']);
add_block('simulink/Sinks/Out1', [sys '/ctrlOut']);
add_line(sys, 'Controller/1', 'deltaCmd/1', 'autorouting', 'smart');
add_line(sys, 'Controller/2', 'ctrlOut/1', 'autorouting', 'smart');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% =================================================================== servos
function build_servos(sys)
% First-order servo with slew-rate and travel limits, one per tab (vector of 4).
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
add_block('simulink/Sources/In1', [sys '/deltaCmd']);
add_block('simulink/Math Operations/Sum', [sys '/Error'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [sys '/1 over tau'], 'Gain', '1/P.servo.tau');
add_block('simulink/Discontinuities/Saturation', [sys '/Slew Limit'], ...
    'UpperLimit', 'P.servo.rateMax', 'LowerLimit', '-P.servo.rateMax');
add_block('simulink/Continuous/Integrator', [sys '/Tab Angle'], ...
    'InitialCondition', 'zeros(P.geo.nFins,1)', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.servo.defMax', 'LowerSaturationLimit', '-P.servo.defMax');
add_block('simulink/Sinks/Out1', [sys '/delta']);
c = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
c('deltaCmd/1', 'Error/1');
c('Error/1', '1 over tau/1');
c('1 over tau/1', 'Slew Limit/1');
c('Slew Limit/1', 'Tab Angle/1');
c('Tab Angle/1', 'delta/1');
c('Tab Angle/1', 'Error/2');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% ==================================================================== plant
function build_plant(sys)
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
add_block('simulink/Sources/In1', [sys '/delta']);
add_block('simulink/Sources/In1', [sys '/wind']);
add_block('simulink/Sources/In1', [sys '/t']);
mlfcn([sys '/Aerodynamics & Propulsion'], ...
    {'function [F_B, M_B, aux] = fcn(x, delta, wind, t, P)'
     '% Body-frame forces/moments: thrust, drag, body lift, fins, fin tabs.'
     '% aux = [alpha; beta; mach; qbar; aoaTotal; thrust; mass; xCg]'
     '[F_B, M_B, aux] = afs_aero(P, x, delta, wind, t);'});
mlfcn([sys '/6DOF Equations of Motion'], ...
    {'function [xdot, onRail] = fcn(x, F_B, M_B, t, P)'
     '% Translational + rotational dynamics, quaternion kinematics, rail constraint.'
     '[xdot, onRail] = afs_eom(P, x, F_B, M_B, t);'});
add_block('simulink/Continuous/Integrator', [sys '/State'], 'InitialCondition', 'P.x0');
add_block('simulink/Sinks/Out1', [sys '/x']);
add_block('simulink/Sinks/Out1', [sys '/aux']);
add_block('simulink/Sinks/Out1', [sys '/onRail']);
c = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
c('State/1', 'Aerodynamics & Propulsion/1');
c('delta/1', 'Aerodynamics & Propulsion/2');
c('wind/1',  'Aerodynamics & Propulsion/3');
c('t/1',     'Aerodynamics & Propulsion/4');
c('State/1', '6DOF Equations of Motion/1');
c('Aerodynamics & Propulsion/1', '6DOF Equations of Motion/2');
c('Aerodynamics & Propulsion/2', '6DOF Equations of Motion/3');
c('t/1',     '6DOF Equations of Motion/4');
c('6DOF Equations of Motion/1', 'State/1');
c('State/1', 'x/1');
c('Aerodynamics & Propulsion/3', 'aux/1');
c('6DOF Equations of Motion/2', 'onRail/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% =================================================================== apogee
function build_apogee(sys)
add_block('simulink/Ports & Subsystems/Subsystem', sys);
clear_subsystem(sys);
add_block('simulink/Sources/In1', [sys '/x']);
add_block('simulink/Sources/In1', [sys '/t']);
sel(sys, 'Vertical Velocity', '6', '13');
add_block('simulink/Logic and Bit Operations/Compare To Constant', [sys '/Descending'], ...
    'relop', '<', 'const', '0');
add_block('simulink/Logic and Bit Operations/Compare To Constant', [sys '/After Boost'], ...
    'relop', '>', 'const', 'P.prop.tBurn');
add_block('simulink/Sources/Constant', [sys '/Stop Enabled'], ...
    'Value', 'P.sim.stopAtApogee', 'OutDataTypeStr', 'boolean');
add_block('simulink/Logic and Bit Operations/Logical Operator', [sys '/AND'], ...
    'Operator', 'AND', 'Inputs', '3');
add_block('simulink/Sinks/Stop Simulation', [sys '/Stop']);
c = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
c('x/1', 'Vertical Velocity/1');
c('Vertical Velocity/1', 'Descending/1');
c('t/1', 'After Boost/1');
c('Descending/1', 'AND/1');
c('After Boost/1', 'AND/2');
c('Stop Enabled/1', 'AND/3');
c('AND/1', 'Stop/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end

%% ================================================================== helpers
function clear_subsystem(sys)
% Remove the default In1 -> Out1 contents of a fresh subsystem.
lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
delete_line(lines);
delete_block([sys '/In1']);
delete_block([sys '/Out1']);
end

function sel(sys, name, idx, width)
add_block('simulink/Signal Routing/Selector', [sys '/' name], ...
    'NumberOfDimensions', '1', 'IndexMode', 'One-based', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', idx, ...
    'InputPortWidth', width);
end

function mlfcn(blk, code)
% MATLAB Function block whose last argument P is a non-tunable parameter.
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = strjoin(code, newline);
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'P');
d.Scope = 'Parameter';
d.Tunable = false;
end

function add_scope(mdl, name, src)
add_block('simulink/Sinks/Scope', [mdl '/' name]);
add_line(mdl, src, [name '/1'], 'autorouting', 'smart');
end
