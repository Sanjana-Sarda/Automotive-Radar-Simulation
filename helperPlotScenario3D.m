function helperPlotScenario3D(ax3d,egoVehicle,rdr)

    % off = egoVehicle.Position+[-10 -30 5];
    % [dx,dy,dz] = sph2cart(deg2rad(40),deg2rad(-5),1);
    % u = [dx dy dz];
    % v = -20*u+off;
    % camtarget(ax3d,v);
    % campos(ax3d,v-u);
    
    % h = findobj(ax3d,'Type','Light');
    % if isempty(h)
    %     h = light(ax3d,'Style','local');
    %     material(ax3d,'metal');
    % end
    % h = h(1);
    % h.Position = egoVehicle.Position+[0 0 5];
    % h.Position = v-10*u;
    
    % camtarget(ax3d,[32.2 1.3 0]);
    % campos(ax3d,[32.2 1.3 310.3]);
    % drawnow
    
    if nargin>2
        h = findobj(ax3d,'DisplayName','Field of view');
        if isempty(h)
            % Add radar's field of view
            fov = deg2rad(rdr.FieldOfView);
            rgMax = rdr.RangeLimits(2);
            [x,y,z] = sph2cart(fov(1)/2*[-1 1 1 -1],fov(2)/2*[-1 -1 1 1],rgMax);
            verts = [[x(:) y(:) z(:)];0 0 0]+rdr.MountingLocation+egoVehicle.Position;
            faces = [ ...
                5 1 2; ...
                5 2 3; ...
                5 3 4; ...
                5 4 1];
            h = patch(ax3d,'Faces',faces,'Vertices',verts,'DisplayName','Field of view');
            h.FaceColor = 'r';
            h.FaceAlpha = 0.05;
        else
            verts = h.Vertices;
            verts = verts-verts(end,:)+rdr.MountingLocation+egoVehicle.Position;
            h.Vertices = verts;
        end
    
        % Show ghosts' images
        scenario = egoVehicle.Scenario;
        tposes = targetPoses(egoVehicle);
        tprofiles = actorProfiles(scenario);
        rdprofiles = helperRoadProfiles(scenario);
        rdposes = helperRoadPoses(egoVehicle);
        poses = [tposes(:);rdposes(:)];
        profiles = [tprofiles(:);rdprofiles(:)];
    
        mntPt = matlabshared.tracking.internal.fusion.MountPoint('FloatToUse',1, ...
            'MountingLocation',rdr.MountingLocation, ...
            'MountingAngles',rdr.MountingAngles);
        posesMnt = toMountedFrame(mntPt,poses);
        gposes = radar.internal.scenario.ghostPoses(posesMnt,profiles);
        gposes = toBodyFrame(mntPt,gposes);
        if isempty(gposes)
            h = findobj(ax3d,'DisplayName','Ghosts');
            if isempty(h)
                wasHeld = ishold(ax3d);
                if ~wasHeld
                    hold(ax3d,'on');
                end
                h = patch(ax3d,'Faces',[],'Vertices',NaN(3,3),'DisplayName','Ghosts');
                h.FaceColor = 'g';
                h.EdgeColor = 'g';
                h.FaceAlpha = 0.1;
                if ~wasHeld
                    hold(ax3d,'off');
                end
            else
                set(h,'Faces',[]);
            end
        else
            gcubes = matlabshared.tracking.internal.fusion.targetTruth.cuboids(gposes,profiles);
            gv = gcubes(1).Vertices;
            gf = gcubes(1).Faces;
            for m = 2:numel(gcubes)
                numVerts = size(gv,1);
                gv = [gv;gcubes(m).Vertices]; %#ok<AGROW>
                gf = [gf;gcubes(m).Faces+numVerts]; %#ok<AGROW>
            end
            
            gv = gv + egoVehicle.Position;
            
            h = findobj(ax3d,'DisplayName','Ghosts');
            if isempty(h)
                wasHeld = ishold(ax3d);
                if ~wasHeld
                    hold(ax3d,'on');
                end
                h = patch(ax3d,'Faces',gf,'Vertices',gv,'DisplayName','Ghosts');
                h.FaceColor = 'g';
                h.EdgeColor = 'g';
                h.FaceAlpha = 0.1;
                if ~wasHeld
                    hold(ax3d,'off');
                end
            else
                set(h,'Faces',gf,'Vertices',gv);
            end
        end
    end
    end