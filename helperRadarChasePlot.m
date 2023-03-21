function [ax3d,tgtRptPlotterFcn] = helperRadarChasePlot(egoVehicle,varargin)

    name = 'Simulating Radar Ghosts due to Multipath';
    figname = strtrim([name ' Chase Plot']);
    [fig3d,isNew] = helperFigureName(figname);
    if isNew
        fig3d.Visible = 'on';
    end
    clf(fig3d);
    ax3d = axes(fig3d);
    
    chasePlot(egoVehicle,'ViewHeight',4,'ViewPitch',10,'Parent',ax3d);
    cameratoolbar(fig3d,'Show','SetMode','dollyhv','SetCoordSys','z');
    
    scenario = egoVehicle.Scenario;
    for m = 1:numel(scenario.Actors)
        h = findobj(ax3d,'Tag',['ActorPatch' num2str(m)]);
        if ~isempty(h)
            h.FaceAlpha = 0.3;
        end
    end
    
    helperPlotScenario3D(ax3d,egoVehicle,varargin{:});
    
    tgtRptPlotterFcn = '';
    if nargin>2
        rdr = varargin{1};
        if strcmpi(rdr.TargetReportFormat,'Tracks')
            tgtRptPlotter = findobj(ax3d,'DisplayName','Tracks');
            if isempty(tgtRptPlotter)
                wasHeld = ishold(ax3d);
                if ~wasHeld
                    hold(ax3d,'on');
                end
                tgtRptPlotter = plot3(ax3d,NaN,NaN,NaN,'ks','MarkerFaceColor','k','DisplayName','Tracks');
                tgtRptPlotter(2) = plot3(ax3d,NaN,NaN,NaN,'k-','Tag','Track velocity');
                if ~wasHeld
                    hold(ax3d,'off');
                end
            else
                tgtRptPlotter(1).XData(:) = NaN;
                tgtRptPlotter(2).XData(:) = NaN;
            end
            tgtRptPlotterFcn = @(trks,config)helperPlotTracks(tgtRptPlotter,egoVehicle,trks,config);
        else
            tgtRptPlotter = findobj(ax3d,'DisplayName','Detections');
            if isempty(tgtRptPlotter)
                wasHeld = ishold(ax3d);
                if ~wasHeld
                    hold(ax3d,'on');
                end
                tgtRptPlotter = plot3(ax3d,NaN,NaN,NaN,'ko','MarkerFaceColor','k','DisplayName','Detections');
                tgtRptPlotter(2) = plot3(ax3d,NaN,NaN,NaN,'k-','Tag','Detection velocity');
                if ~wasHeld
                    hold(ax3d,'off');
                end
            else
                tgtRptPlotter(1).XData(:) = NaN;
                tgtRptPlotter(2).XData(:) = NaN;
            end
            tgtRptPlotterFcn = @(dets,config)helperPlotDetections(tgtRptPlotter,egoVehicle,dets,config);
        end
    end
    end
    
    function helperPlotDetections(detPlotter,egoVehicle,dets,config)
    velScale = 0.1;
    ax = detPlotter(1).Parent;
    if numel(dets)
        
        pos = cell2mat(cellfun(@(d)d.Measurement(1:3),dets(:)','UniformOutput',false));
        vel = cell2mat(cellfun(@(d)d.Measurement(4:6),dets(:)','UniformOutput',false));
        spd = sqrt(sum(abs(vel).^2,1));
        isGd = true|spd<5;
    
        if any(isGd)
            pos = pos(:,isGd)+egoVehicle.Position(:);
            set(detPlotter(1),'XData',pos(1,:),'YData',pos(2,:),'ZData',pos(3,:));
    
            vel = vel(:,isGd)+egoVehicle.Velocity(:);
            vel = velScale*vel;
            vel = [pos;pos;NaN*pos]+[0*vel;vel;vel];
            set(detPlotter(2),'XData',vel(1:3:end),'YData',vel(2:3:end),'ZData',vel(3:3:end));
    
            % Annotate 2 and 3 bounce ghosts
            if isfield(dets{1}.ObjectAttributes{1},'BouncePathIndex')
                pltr21 = findobj(ax,'DisplayName','1^{st} 2-bounce ghosts');
                if isempty(pltr21) || ~ishghandle(pltr21)
                    washeld = ishold(ax);
                    if ~washeld
                        hold(ax,'on');
                    end
                    clrs = lines(4);
                    pltr21 = plot3(ax,NaN,NaN,NaN,'ko', ...
                        'DisplayName','1^{st} 2-bounce ghosts', ...
                        'MarkerFaceColor',clrs(2,:));
                    pltr21v = plot3(ax,NaN,NaN,NaN,'k-', ...
                        'Tag','1^{st} 2-bounce ghosts vel');
                    pltr22 = plot3(ax,NaN,NaN,NaN,'ko', ...
                        'DisplayName','2^{nd} 2-bounce ghosts', ...
                        'MarkerFaceColor',clrs(3,:));
                    pltr22v = plot3(ax,NaN,NaN,NaN,'k-', ...
                        'Tag','2^{nd} 2-bounce ghosts vel');
                    pltr3 = plot3(ax,NaN,NaN,NaN,'ko', ...
                        'DisplayName','3-bounce ghosts', ...
                        'MarkerFaceColor',clrs(4,:));
                    pltr3v = plot3(ax,NaN,NaN,NaN,'k-', ...
                        'Tag','3-bounce ghosts vel');
                    if ~washeld
                        hold(ax,'off');
                    end
                else
                    pltr21v = findobj(ax,'Tag','1^{st} 2-bounce ghosts vel');
                    pltr22 = findobj(ax,'DisplayName','2^{nd} 2-bounce ghosts');
                    pltr22v = findobj(ax,'Tag','2^{nd} 2-bounce ghosts vel');
                    pltr3 = findobj(ax,'DisplayName','3-bounce ghosts');
                    pltr3v = findobj(ax,'Tag','3-bounce ghosts vel');
                end
    
                vel = reshape(vel,3,3,[]);
                
                is21 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets(:)')==1;
                is21 = is21(isGd);
                if any(is21)
                    set(pltr21,'XData',pos(1,is21),'YData',pos(2,is21),'ZData',pos(3,is21));
                    tmp = vel(:,:,is21);
                    set(pltr21v,'XData',tmp(1,:),'YData',tmp(2,:),'ZData',tmp(3,:));
                else
                    pltr21.XData(:) = NaN;
                    pltr21v.XData(:) = NaN;
                end
    
                is22 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets(:)')==2;
                is22 = is22(isGd);
                if any(is22)
                    set(pltr22,'XData',pos(1,is22),'YData',pos(2,is22),'ZData',pos(3,is22));
                    tmp = vel(:,:,is22);
                    set(pltr22v,'XData',tmp(1,:),'YData',tmp(2,:),'ZData',tmp(3,:));
                else
                    pltr22.XData(:) = NaN;
                    pltr22v.XData(:) = NaN;
                end
    
                is3 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets(:)')==3;
                is3 = is3(isGd);
                if any(is3)
                    set(pltr3,'XData',pos(1,is3),'YData',pos(2,is3),'ZData',pos(3,is3));
                    tmp = vel(:,:,is3);
                    set(pltr3v,'XData',tmp(1,:),'YData',tmp(2,:),'ZData',tmp(3,:));
                else
                    pltr3.XData(:) = NaN;
                    pltr3v.XData(:) = NaN;
                end
            end
        else
            detPlotter(1).XData(:) = NaN;
            detPlotter(2).XData(:) = NaN;
    
            pltr21 = findobj(ax,'DisplayName','1^{st} 2-bounce ghosts');
            if ~isempty(pltr21)
                pltr21v = findobj(ax,'Tag','1^{st} 2-bounce ghosts vel');
                pltr22 = findobj(ax,'DisplayName','2^{nd} 2-bounce ghosts');
                pltr22v = findobj(ax,'Tag','2^{nd} 2-bounce ghosts vel');
                pltr3 = findobj(ax,'DisplayName','3-bounce ghosts');
                pltr3v = findobj(ax,'Tag','3-bounce ghosts vel');
                
                pltr21.XData(:) = NaN;
                pltr21v.XData(:) = NaN;
                pltr22.XData(:) = NaN;
                pltr22v.XData(:) = NaN;
                pltr3.XData(:) = NaN;
                pltr3v.XData(:) = NaN;
            end
        end
    elseif config.IsValidTime
        detPlotter(1).XData(:) = NaN;
        detPlotter(2).XData(:) = NaN;
    
        pltr21 = findobj(ax,'DisplayName','1^{st} 2-bounce ghosts');
        if ~isempty(pltr21)
            pltr21v = findobj(ax,'Tag','1^{st} 2-bounce ghosts vel');
            pltr22 = findobj(ax,'DisplayName','2^{nd} 2-bounce ghosts');
            pltr22v = findobj(ax,'Tag','2^{nd} 2-bounce ghosts vel');
            pltr3 = findobj(ax,'DisplayName','3-bounce ghosts');
            pltr3v = findobj(ax,'Tag','3-bounce ghosts vel');
            
            pltr21.XData(:) = NaN;
            pltr21v.XData(:) = NaN;
            pltr22.XData(:) = NaN;
            pltr22v.XData(:) = NaN;
            pltr3.XData(:) = NaN;
            pltr3v.XData(:) = NaN;
        end
    end
    end
    
    function helperPlotTracks(trkPlotter,egoVehicle,trks,config)
    velScale = 0.3;
    if numel(trks)
        pos = cell2mat(arrayfun(@(t)t.State(1:2:end),trks(:)','UniformOutput',false));
        vel = cell2mat(arrayfun(@(t)t.State(2:2:end),trks(:)','UniformOutput',false));
        lbls = arrayfun(@(t)['T' num2str(t.TrackID)],trks,'UniformOutput',false);
        
        pos = pos+egoVehicle.Position(:);
        set(trkPlotter(1),'XData',pos(1,:),'YData',pos(2,:),'ZData',pos(3,:));
    
        
        vel = vel+egoVehicle.Velocity(:);
        vel = velScale*vel;
        vel = [pos;pos;NaN*pos]+[0*vel;vel;vel];
        set(trkPlotter(2),'XData',vel(1:3:end),'YData',vel(2:3:end),'ZData',vel(3:3:end));
    elseif config.IsValidTime
        trkPlotter(1).XData(:) = NaN;
        trkPlotter(2).XData(:) = NaN;
    end
    end