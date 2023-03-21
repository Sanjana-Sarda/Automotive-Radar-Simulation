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

% Output tracks instead of detections
release(rdg);
rdg.TargetReportFormat = 'Tracks';
rdg.ConfirmationThreshold = [2 3];
rdg.DeletionThreshold = [5 5];
FilterInitializationFcn = 'initcvekf'; % constant-velocity EKF

% Create a new bird's eye plot to plot the tracks
[bep,trkPlotterFcn] = helperSetupBEP(egoVehicle,rdg);
title('Simple Multipath Environment');

% Run simulation
restart(scenario);
scenario.StopTime = 7.5;
while advance(scenario)
    time = scenario.SimulationTime;
    tposes = targetPoses(egoVehicle);

    % Generate tracks
    [trks,~,config] = rdg(tposes,time);

    % Filter out tracks corresponding to static objects (e.g. barrier)
    dyntrks = helperKeepDynamicObjects(trks, egoVehicle);

    % Visualize dynamic tracks
    helperPlotScenario(bep,egoVehicle);
    trkPlotterFcn(dyntrks,config);
end

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

helperPlotBeamformedRangeDoppler(Xbfmrngdop,rggrid,anggrid,rtxrx);


function y = normmax(x)
    if all(abs(x(:))==0)
        y = ones(size(x),'like',x);
    else
        y = x(:)/max(abs(x(:)));
    end
    end