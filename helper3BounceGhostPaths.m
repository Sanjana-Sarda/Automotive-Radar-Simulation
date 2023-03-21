function [paths,gposes,idxTgts,grazang] = helper3BounceGhostPaths(tposes,rdg)
    % This is a helper function and may be removed or modified in a future
    % release.
    
    %   Copyright 2021-2023 The MathWorks, Inc.
    
    % Add profile info to pose info
    tgts = matlabshared.tracking.internal.fusion.truthPose(tposes);
    ids = [rdg.Profiles.ActorID];
    for m = 1:numel(tgts)
        iFnd = find(tgts(m).TruthID==ids,1);
    
        dims = struct( ...
            'Length',rdg.Profiles(iFnd).Length, ...
            'Width',rdg.Profiles(iFnd).Width, ...
            'Height',rdg.Profiles(iFnd).Height, ...
            'OriginOffset',rdg.Profiles(iFnd).OriginOffset);
        sigs = {rcsSignature( ...
            'Azimuth',rdg.Profiles(iFnd).RCSAzimuthAngles, ...
            'Elevation',rdg.Profiles(iFnd).RCSElevationAngles, ...
            'Pattern',rdg.Profiles(iFnd).RCSPattern)};
    
        tgts(m).Dimensions = dims;
        tgts(m).Signatures = sigs;
    end
    
    % Get target poses in the sensor frame
    mntPt = matlabshared.tracking.internal.fusion.MountPoint( ...
        'MountingLocation',rdg.MountingLocation, ...
        'MountingAngles',rdg.MountingAngles, ...
        'FloatToUse',1);
    tgts = toMountedFrame(mntPt,tgts);
    
    rcsdBsm = arrayfun(@(t)t.Signatures{1}.Pattern(1),tgts);
    
    % Get ghost target information
    [gposes,idxTgts] = radar.internal.scenario.ghostPoses(tgts,rdg.RangeLimits(2),rdg.FieldOfView);
    
    % Generate path information for input to transceiver
    tpos = cell2mat(arrayfun(@(p)p.Position(:),tgts(idxTgts)','UniformOutput',false));
    tvel = cell2mat(arrayfun(@(p)p.Velocity(:),tgts(idxTgts)','UniformOutput',false));
    gpos = cell2mat(arrayfun(@(p)p.Position(:),gposes(:)','UniformOutput',false));
    gvel = cell2mat(arrayfun(@(p)p.Velocity(:),gposes(:)','UniformOutput',false));
    
    rcs = db2pow(rcsdBsm);
    lambda = freq2wavelen(rdg.CenterFrequency);
    paths = radar.internal.propagation.threeBouncePaths(tpos,tvel,gpos,gvel,[],lambda,rcs(idxTgts));
    
    % Remove direct path (we'll add it back next for all of the targets)
    paths = paths(:,2:4);
    paths = paths(:).';
    idxTgts = repmat(idxTgts(:),[1 3]);
    idxTgts = idxTgts(:).';
    
    % Add direct path returns
    coeff = matlabshared.tracking.internal.fusion.db2mag(aperture2gain(rcs(:)',lambda(:)'));
    tpos = cell2mat(arrayfun(@(t)t.Position(:),tgts(:)','UniformOutput',false));
    [th,ph,rg] = cart2sph(tpos(1,:),tpos(2,:),tpos(3,:));
    angs = rad2deg([th(:) ph(:)]');
    aoa = angs;
    aod = angs;
    
    len = 2*rg;
    pathLossdB = 2*fspl(rg,lambda);
    
    tvel = cell2mat(arrayfun(@(t)t.Velocity(:),tgts(:)','UniformOutput',false));
    
    ur = bsxfun(@rdivide,tpos,rg);
    rr = dot(ur,tvel,1);
    dop = -2*speed2dop(rr,lambda);
    
    dly = range2time(rg);
    
    dp = repmat(struct( ...
        'PathLength',zeros(1,1,'like',rg), ...
        'PathDelay',zeros(1,1,'like',rg), ...
        'PathLoss',zeros(1,1,'like',rg), ...
        'ReflectionCoefficient',zeros(1,1,'like',rg), ...
        'AngleOfDeparture',zeros(2,1,'like',rg), ...
        'AngleOfArrival',zeros(2,1,'like',rg), ...
        'DopplerShift',zeros(1,1,'like',rg)),size(rg,2),1);
    for m = 1:numel(dp)
        dp(m).PathLength = len(m);
        dp(m).PathDelay = dly(m);
        dp(m).PathLoss = pathLossdB(m);
        dp(m).ReflectionCoefficient(:) = coeff(m);
        dp(m).AngleOfArrival(:) = aoa(:,m);
        dp(m).AngleOfDeparture(:) = aod(:,m);
        dp(m).DopplerShift(:) = dop(m);
    end
    
    % Make sure path structures match. Assumes dp structure fields match with
    % some of those in the paths structure.
    dpPathFields = fields(dp); 
    rdrPathFields = fields(paths); 
    rmFieldNames = setdiff(rdrPathFields,dpPathFields); 
    paths = rmfield(paths,rmFieldNames);
    
    % Concatenate outputs
    paths = [dp(:);paths(:)]';
    idxTgts = [1:numel(tgts) idxTgts(:)'];
    end
    