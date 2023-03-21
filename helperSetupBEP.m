function [bep,tgtRptPlotterFcn] = helperSetupBEP(egoVehicle,rdr)

    name = 'Simulating Radar Ghosts due to Multipath';
    figname = strtrim([name ' BEP']);
    [fig,isNew] = helperFigureName(figname);
    if isNew
        fig.Visible = 'on';
    end
    clf(fig);
    ax = axes(fig);
    
    bep = birdsEyePlot('XLim',[-5 60],'YLim',20*[-1 1],'Parent',ax);
    legend(ax,'off');
    
    helperPlotScenario(bep,egoVehicle);
    
    
    % Add radar's field of view
    caPlotter = coverageAreaPlotter(bep,'DisplayName','Radar FoV','FaceColor','r');
    pos = rdr.MountingLocation(1:2);
    rgMax = rdr.RangeLimits(2);
    yaw = rdr.MountingAngles(1);
    azFov = rdr.FieldOfView(1);
    
    plotCoverageArea(caPlotter,pos,rgMax,yaw,azFov);
    
    if nargin>1
        if strcmpi(rdr.TargetReportFormat,'Tracks')
            tgtRptPlotter = trackPlotter(bep,'MarkerFaceColor','k','DisplayName','Radar tracks','VelocityScaling',0.3);
            tgtRptPlotterFcn = @(trks,config)helperPlotTracks(tgtRptPlotter,trks,config);
        else
            tgtRptPlotter = detectionPlotter(bep,'DisplayName','Radar detections','VelocityScaling',0.3,'MarkerFaceColor','k');
            tgtRptPlotterFcn = @(dets,config)helperPlotDetections(tgtRptPlotter,dets,config);
        end
        legend(ax,'show');
    end
    end
    
    function helperPlotDetections(detPlotter,dets,config)
    if numel(dets)
        pos = cell2mat(cellfun(@(d)d.Measurement(1:2),dets(:)','UniformOutput',false))';
        vel = cell2mat(cellfun(@(d)d.Measurement(4:5),dets(:)','UniformOutput',false))';
        plotDetection(detPlotter,pos,vel);
        
        % Annotate 2 and 3 bounce ghosts
        if isfield(dets{1}.ObjectAttributes{1},'BouncePathIndex')
            bep = detPlotter.ParentPlot;
            pltr21 = findPlotter(bep,'DisplayName','1^{st} 2-bounce ghosts');
            if isempty(pltr21)
                clrs = lines(4);
                pltr21 = detectionPlotter(bep,'DisplayName','1^{st} 2-bounce ghosts', ...
                    'Marker','o','MarkerFaceColor',clrs(2,:), ...
                    'VelocityScaling',0.3);
                pltr22 = detectionPlotter(bep,'DisplayName','2^{nd} 2-bounce ghosts', ...
                    'Marker','o','MarkerFaceColor',clrs(3,:), ...
                    'VelocityScaling',0.3);
                pltr3 = detectionPlotter(bep,'DisplayName','3-bounce ghosts', ...
                    'Marker','o','MarkerFaceColor',clrs(4,:), ...
                    'VelocityScaling',0.3);
            else
                pltr22 = findPlotter(bep,'DisplayName','2^{nd} 2-bounce ghosts');
                pltr3 = findPlotter(bep,'DisplayName','3-bounce ghosts');
            end
            
            is21 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets)==1;
            plotDetection(pltr21,pos(is21,:),vel(is21,:));
            
            is22 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets)==2;
            plotDetection(pltr22,pos(is22,:),vel(is22,:));
    
            is3 = cellfun(@(d)d.ObjectAttributes{1}.BouncePathIndex,dets)==3;
            plotDetection(pltr3,pos(is3,:),vel(is3,:));
        end
    elseif config.IsValidTime
        clearData(detPlotter);
    end
    end
    function helperPlotTracks(trkPlotter,trks,config)
    if numel(trks)
        pos = cell2mat(arrayfun(@(t)t.State(1:2:end),trks(:)','UniformOutput',false));
        vel = cell2mat(arrayfun(@(t)t.State(2:2:end),trks(:)','UniformOutput',false));
        lbls = arrayfun(@(t)['T' num2str(t.TrackID)],trks,'UniformOutput',false);
        
        isghost = false(1,numel(trks));
        % Annotate ghost tracks
        if isfield(trks(1).ObjectAttributes,'BouncePathIndex')
            bep = trkPlotter.ParentPlot;
            pltrgTrks = findPlotter(bep,'DisplayName','Ghost tracks');
            if isempty(pltrgTrks)
                pltrgTrks = trackPlotter(bep,'MarkerFaceColor','w','DisplayName','Ghost tracks','VelocityScaling',0.3);
            end
            is21 = cellfun(@(d)d.BouncePathIndex,{trks(:).ObjectAttributes})==1;
            is22 = cellfun(@(d)d.BouncePathIndex,{trks(:).ObjectAttributes})==2;
            is3 = cellfun(@(d)d.BouncePathIndex,{trks(:).ObjectAttributes})==3;
            isghost = is21 | is22 | is3;
            plotTrack(pltrgTrks,pos(1:2,isghost)',vel(1:2,isghost)',lbls(isghost));
        end
        plotTrack(trkPlotter,pos(1:2,~isghost)',vel(1:2,~isghost)',lbls(~isghost));
    elseif config.IsValidTime
        clearData(trkPlotter);
    end
    end
    