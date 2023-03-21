function profiles = helperRoadProfiles(egoVehicle)

    persistent lastscene gids
    
    isEgo = isa(egoVehicle,'driving.scenario.Actor');
    
    if isEgo
        scenario = egoVehicle.Scenario;
    else
        scenario = egoVehicle;
    end
    rt = scenario.RoadTiles;
    
    if isempty(lastscene) ...
            || scenario~=lastscene ...
            || numel(rt)~=numel(gids)
        lastscene = scenario;
    %     gids = helperGroupRoadTiles(rt);
        gids = 1:numel(rt);
    end
    
    uids = unique(gids);
    num = numel(uids);
    
    % Default profiles for the road tiles
    profiles = actorProfiles(scenario);
    profiles = profiles(1);
    profiles.ClassID = 0;
    profiles.Height = 1e-1;
    profiles.OriginOffset = [0 0 profiles.Height];
    
    % Compute approximate peak RCS for road illuminated by beam at normal incidence
    ht = 0.2;  % m
    bw = 10; % deg
    radius = ht*tand(bw/2);
    rcs0 = pow2db(rcsdisc(radius,3e8,77e9,0,90));
    
    % Compute grazing angle dependent variation in RCS
    lambda = freq2wavelen(77e9,3e8);
    sigh = 5e-3; % approximate road surface roughness inside of beam (m)
    el = -90:90;
    arg = 2*(2*pi*sigh*sind(el)/lambda).^2;
    ps = exp(-arg); % Ament's formula
    ps = sqrt(1-ps.^2);
    psdB = mag2db(ps);
    psdB = max(psdB,-400);
    rcs = rcs0+psdB;
    
    profiles.RCSPattern = [rcs(:) rcs(:)]; % dBsm
    profiles.RCSAzimuthAngles = [-180 180];
    profiles.RCSElevationAngles = el;
    profiles = repmat(profiles,num,1);
    
    rtIDOffset = 100e3;
    for m = 1:num
    
        % All of the tiles that are part of this straight road segment
        iTiles = find(uids(m)==gids);
    
        thisTile = rt(iTiles(1));
        verts = thisTile.Vertices;
    
        v1 = diff(verts(1:2,:),1,1);
        len = norm(v1);
        u1 = v1(:)/len;
        u3 = thisTile.SurfaceNormal(:);
        u2 = cross(u3,u1);
        rotmat = [u1 u2 u3];
        
        off = verts(1,:);
        verts = bsxfun(@minus,verts,off)*rotmat;
        for k = 2:numel(iTiles)
            theseVerts = rt(iTiles(k)).Vertices;
            theseVerts = bsxfun(@minus,theseVerts,off)*rotmat;
            thisDist = theseVerts(:,2);
            d = verts(:,2);
            if any(thisDist>max(d))
                iNew = isZero(thisDist-max(thisDist));
                iOld = isZero(d-max(d));
                verts(iOld,:) = theseVerts(iNew,:);
            end
            if any(thisDist<min(d))
                iNew = isZero(thisDist-min(thisDist));
                iOld = isZero(d-min(d));
                verts(iOld,:) = theseVerts(iNew,:);
            end
        end
        verts = bsxfun(@plus,verts*rotmat',off);
        
        v1 = diff(verts(1:2,:),1);
        len = norm(v1);
        v2 = diff(verts(2:3,:),1);
        wid = norm(v2);
        
        profiles(m).ActorID = thisTile.TileID+rtIDOffset;
        profiles(m).Length = len;
        profiles(m).Width = wid;
    end
    end
    
    
    function tf = isZero(val)
    tf = abs(val)<1e-10;
    end
    