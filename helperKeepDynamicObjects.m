function dynrpts = helperKeepDynamicObjects(rpts,egoVehicle)
    % Filter out target reports corresponding to static objects (e.g. guardrail)
    %
    % This is a helper function and may be removed or modified in a future
    % release.
    
    dynrpts = rpts;
    if ~isempty(rpts)
        if iscell(rpts)
            vel = cell2mat(cellfun(@(d)d.Measurement(4:end),rpts(:)','UniformOutput',false));
        else
            vel = cell2mat(arrayfun(@(t)t.State(2:2:end),rpts(:)','UniformOutput',false));
        end
        vel = sign(vel(1,:)).*sqrt(sum(abs(vel(1:2,:)).^2,1));
        egoVel = sign(egoVehicle.Velocity(1))*norm(egoVehicle.Velocity(1:2));
        gndvel = vel+egoVel; % detection speed relative to ground
        isStatic = gndvel > -4 & ... greater than 4 m/s departing and,
            gndvel < 8; % less than 8 m/s closing speed
        dynrpts = rpts(~isStatic);
    end
    end