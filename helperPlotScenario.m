function helperPlotScenario(bep,egoVehicle)

    % Add road boundaries
    rbPlotter = findPlotter(bep,'DisplayName','Road');
    if isempty(rbPlotter)
        rbPlotter = laneBoundaryPlotter(bep,'DisplayName','Road');
    end
    rb = roadBoundaries(egoVehicle);
    plotLaneBoundary(rbPlotter,rb);
    
    % Add lane markings
    lmPlotter = findPlotter(bep,'Tag','Lane Markings');
    if isempty(lmPlotter)
        lmPlotter = laneMarkingPlotter(bep,'Tag','Lane Markings');
    end
    [lmv, lmf] = laneMarkingVertices(egoVehicle);
    plotLaneMarking(lmPlotter,lmv,lmf);
    
    % Create outline plotter for barriers and actors
    olPlotter = findPlotter(bep,'Tag','Outlines');
    if isempty(olPlotter)
        olPlotter = outlinePlotter(bep,'Tag','Outlines');
    end
    
    % Add barriers
    [bPosition,bYaw,bLength,bWidth,bOriginOffset,bColor,bSegs] = targetOutlines(egoVehicle,'Barriers');
    plotBarrierOutline(olPlotter,bSegs,bPosition,bYaw,bLength,bWidth,...
        'OriginOffset',bOriginOffset,'Color',bColor);
    
    % Add actor outlines
    [position,yaw,length,width,originOffset,color] = targetOutlines(egoVehicle);
    plotOutline(olPlotter,position,yaw,length,width, ...
        'OriginOffset',originOffset,'Color',color);
    end
    