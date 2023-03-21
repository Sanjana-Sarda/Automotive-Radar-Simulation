% Set random seed for reproducible results
rndState = rng('default');

% Create scenario using helper
[scenario, egoVehicle] = helperSimpleHighwayScenarioDSD();

% Automotive radar system parameters
freq = 77e9; % Hz
rgMax = 250; % m
spMax = 30; % m/s
rcs = 10;    % dBsm

azRes = 4;   % deg
rgRes = 2.5; % m
rrRes = 0.5; % m/s

% Create a forward-looking automotive radar
rdg = radarDataGenerator(1, 'No scanning', ...
    'UpdateRate', 10, ...
    'MountingLocation', [3.4 0 0.2], ...
    'CenterFrequency', freq, ...
    'HasRangeRate', true, ...
    'FieldOfView', [70 5], ...
    'RangeLimits', [0 rgMax], ...
    'RangeRateLimits', [-spMax spMax], ...
    'HasRangeAmbiguities',true, ...
    'MaxUnambiguousRange', rgMax, ...
    'HasRangeRateAmbiguities',true, ...
    'MaxUnambiguousRadialSpeed', spMax, ...
    'ReferenceRange', rgMax, ...
    'ReferenceRCS',rcs, ...
    'AzimuthResolution',azRes, ...
    'RangeResolution',rgRes, ...
    'RangeRateResolution',rrRes, ...
    'TargetReportFormat', 'Detections', ...
    'Profiles',actorProfiles(scenario));

% Create bird's eye plot and detection plotter function
[~,detPlotterFcn] = helperSetupBEP(egoVehicle,rdg);

% Enable ghost target model
release(rdg);
rdg.HasGhosts = true;

% Generate raw detections
time = scenario.SimulationTime;
tposes = targetPoses(egoVehicle);
[dets,~,config] = rdg(tposes,time);

% Plot detections
detPlotterFcn(dets,config);
title('Simple Multipath Environment');


%IQ Data Generation
rgMax = rdg.RangeLimits(2)       % m
spMax = rdg.RangeRateLimits(2)   % m/s

lambda = freq2wavelen(rdg.CenterFrequency);
prf = 2*speed2dop(2*spMax,lambda);

rrRes = rdg.RangeRateResolution
dopRes = 2*speed2dop(rrRes,lambda);
numPulses = 2^nextpow2(prf/dopRes)

prf = dopRes*numPulses

rgUmb = time2range(1/prf)

release(rdg);

% Set the range and range-rate ambiguities according to desired PRF and
% number of pulses
rdg.MaxUnambiguousRange = rgUmb;
rdg.MaxUnambiguousRadialSpeed = spMax;

% Set the statistical radar to report clustered detections to compare to
% the IQ video from the radar transceiver.
rdg.TargetReportFormat = 'Clustered detections';
rdg.DetectionCoordinates = 'Body';

azRes = rdg.AzimuthResolution;
rdg.AzimuthResolution = rdg.FieldOfView(1);

% Construct the radar transceiver from the radar data generator
rtxrx = radarTransceiver(rdg)

% Restore the desired azimuth resolution for the statistical radar
rdg.AzimuthResolution = azRes;

numRxElmt = ceil(beamwidth2ap(rdg.AzimuthResolution,lambda,0.8859)/(lambda/2))

elmt = rtxrx.ReceiveAntenna.Sensor;
rxarray = phased.ULA(numRxElmt,lambda/2,'Element',elmt);
rtxrx.ReceiveAntenna.Sensor = rxarray;

restart(scenario);
tposes = targetPoses(egoVehicle);

% Generate 3-bounce propagation paths for the targets in the scenario
paths = helper3BounceGhostPaths(tposes,rdg);

time = scenario.SimulationTime; % Current simulation time
Xcube = rtxrx(paths,time);      % Generate IQ data for transceiver from the 3-bounce path model

size(Xcube)

rngdopproc = phased.RangeDopplerResponse( ...
    'RangeMethod','Matched filter', ...
    'DopplerOutput','Speed', ...
    'PropagationSpeed',rtxrx.ReceiveAntenna.PropagationSpeed, ...
    'OperatingFrequency',rtxrx.ReceiveAntenna.OperatingFrequency, ...
    'SampleRate',rtxrx.Receiver.SampleRate);
mfcoeff = getMatchedFilter(rtxrx.Waveform);
[Xrngdop,rggrid,rrgrid] = rngdopproc(Xcube,mfcoeff);

azFov = rdg.FieldOfView(1);
anggrid = -azFov/2:azFov/2;
bmfwin = @(N)normmax(taylorwin(N,5,-60));
beamformer = phased.PhaseShiftBeamformer( ...
        'Direction',[anggrid;0*anggrid],...
        'SensorArray',rtxrx.ReceiveAntenna.Sensor, ...
        'OperatingFrequency',rtxrx.ReceiveAntenna.OperatingFrequency);
Xbfmrngdop = Xrngdop;
[Nr,Ne,Nd] = size(Xbfmrngdop);
Xbfmrngdop = permute(Xbfmrngdop,[1 3 2]); % Nr x Nd x Ne
Xbfmrngdop = reshape(Xbfmrngdop,[],Ne);
Xbfmrngdop = beamformer(Xbfmrngdop.*bmfwin(Ne)');
Xbfmrngdop = reshape(Xbfmrngdop,Nr,Nd,[]); % Nr x Nd x Nb
Xbfmrngdop = permute(Xbfmrngdop,[1 3 2]); % Nr x Nb x Nd

%%Spoofing Attack
[scenario, egoVehicle] = helperDroneSpoofScenario;

viewLoc = [scenario.Actors(2).Position(1)-10 -10];

release(rdg);
rdg.RangeRateResolution = 0.5;
rdg.FieldOfView(2) = 10;
rdg.TargetReportFormat = 'Detections';

tprofiles = actorProfiles(scenario);
rdprofiles = helperRoadProfiles(scenario);
rdg.Profiles = [tprofiles;rdprofiles];

% Create bird's eye plot and detection plotter function
[bep,detPlotterFcn] = helperSetupBEP(egoVehicle,rdg);
[ax3d,detChasePlotterFcn] = helperRadarChasePlot(egoVehicle,rdg);
camup(ax3d,[0 0 1]);
pos = egoVehicle.Position+[-5 -5 0];
campos(ax3d,pos);
camtarget(ax3d,[15 0 0]);

% Generate clustered detections
time = scenario.SimulationTime;
tposes = targetPoses(egoVehicle);
rdposes = helperRoadPoses(egoVehicle);
poses = [tposes rdposes];

[dets,~,config] = rdg(poses,time);

% Plot detections
dyndets = helperKeepDynamicObjects(dets,egoVehicle);
detPlotterFcn(dyndets,config);



function y = normmax(x)
if all(abs(x(:))==0)
    y = ones(size(x),'like',x);
else
    y = x(:)/max(abs(x(:)));
end
end