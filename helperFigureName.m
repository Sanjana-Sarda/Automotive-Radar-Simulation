function [fig,isNew] = helperFigureName(name)
    fig = findobj(groot,'Type','figure','Name',name);
    isNew = isempty(fig);
    if isNew
        fig = figure('Name',name);
    end
    fig = fig(1);
    end