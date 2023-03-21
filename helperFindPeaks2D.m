function idx = helperFindPeaks2D(x,thresh,minPts)
    % This is a helper function and may be removed or modified in a future
    % release.
    
    if nargin<3
        minPts = 2;
    end
    if isscalar(minPts)
        minPts = minPts*ones(size(x),'like',minPts);
    end
    
    iFnd = find(x(:)>thresh);
    idx = zeros(size(iFnd),'like',iFnd);
    
    iWin = cell(numel(minPts),1);
    for m = 1:numel(minPts)
        iWin0 = 1:minPts(m);
        iWin{m} = iWin0-floor(mean(iWin0));
    end
    
    isFnd = idx<1;
    while any(isFnd)
        iPtFnd = find(isFnd,1);
        [iFnd,idx] = findLocalMax(x,iPtFnd,iFnd,idx,iWin);
        isFnd = idx<1;
    end
    
    idx = idx(idx>0);
    idx = unique(idx);
    end
    
    function [iFnd,idx] = findLocalMax(x,iPtFnd,iFnd,idx,iWin)
    
    % Search adjacent cells for max point
    [rPt,cPt] = ind2sub(size(x),iFnd(iPtFnd));
    rWin = rPt+iWin{1};
    cWin = cPt+iWin{2};
    rWin = rWin(rWin>0&rWin<=size(x,1));
    cWin = cWin(cWin>0&cWin<=size(x,2));
    [rSrch,cSrch] = meshgrid(rWin,cWin);
    iSrch = sub2ind(size(x),rSrch(:),cSrch(:));
    
    [~,iMax] = max(x(iSrch));
    iMax = iSrch(iMax);
    iPtMax = find(iFnd==iMax,1);
    if isempty(iPtMax)
        iPtMax = iPtFnd;
    end
    
    % Replace previous max with new max
    if idx(iPtFnd)>0
        isPtPrev = idx==idx(iPtFnd);
        idx(isPtPrev) = iMax;
    end
    
    % Assign max to adjacent cells
    isPtSrch = ismember(iFnd,iSrch);
    idx(isPtSrch) = iMax;
    
    if iPtMax~=iPtFnd
        % Local max not yet found. Keep searching.
        [iFnd,idx] = findLocalMax(x,iPtMax,iFnd,idx,iWin);
    end
    end