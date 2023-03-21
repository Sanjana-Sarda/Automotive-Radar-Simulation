function rdposes = helperRoadPoses(egoVehicle)

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
    
    % Default pose for the road tiles
    rdposes = struct('ActorID',0,'ClassID',0);
    poses = actorPoses(scenario);
    flds = fieldnames(poses);
    for m = 1:numel(flds)
        thisFld = flds{m};
        rdposes.(thisFld) = poses(1).(thisFld);
    end
    rdposes.Roll = 0;
    rdposes.Pitch = 0;
    rdposes.Yaw = 0;
    rdposes.Velocity(:) = 0;
    rdposes.AngularVelocity(:) = [0 0 0];
    rdposes = repmat(rdposes(1),1,num);
    
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
    
        [roll, pitch, yaw] = rot2euler(rotmat);
    %     yaw = atan2d(u1(2),u1(1));
    
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
        pos = mean(verts,1);
    
        rdposes(m).ActorID = thisTile.TileID+rtIDOffset;
        rdposes(m).Position(:) = pos;
        rdposes(m).Roll = roll;
        rdposes(m).Pitch = pitch;
        rdposes(m).Yaw = yaw;
    end
    
    if isEgo
        rdposes = driving.scenario.targetsToEgo(rdposes,egoVehicle);
    end
    end
    
    function tf = isZero(val)
    tf = abs(val)<1e-10;
    end
    
    function [roll, pitch, yaw] = rot2euler(rotmat)
    if rotmat(1,1) == 0 && rotmat(2,1) == 0
        roll = atan2d(rotmat(1,2),rotmat(2,2));
    else
        roll = atan2d(rotmat(3,2),rotmat(3,3));
    end
    if rotmat(1,1) == 0 && rotmat(2,1) == 0
        pitch = 90;
    else
        pitch = atan2d(-rotmat(3,1),sqrt(sum(rotmat(1:2,1).^2)));
    end
    if rotmat(1,1) == 0 && rotmat(2,1) == 0
        yaw = 0;
    else
        yaw = atan2d(rotmat(2,1),rotmat(1,1));
    end
    end