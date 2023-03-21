function fig = helperPlotBeamformedRangeDoppler(Xbfmrngdop,rnggrid,anggrid,rtxrx)
    Xpow = abs(Xbfmrngdop).^2;
    noisedB = pow2db(noisepow(rtxrx.Receiver.SampleRate,rtxrx.Receiver.NoiseFigure,rtxrx.Receiver.ReferenceTemperature));
    
    % "Find peaks" across Doppler bins for each beam
    Xpow = max(Xpow,[],3); % Nr x Nb
    
    mfcoeff = getMatchedFilter(rtxrx.Waveform);
    Gft = pow2db(numel(mfcoeff));
    Gst = pow2db(rtxrx.NumRepetitions);
    Gbf = pow2db(rtxrx.ReceiveAntenna.Sensor.NumElements);
    Gproc = Gft+Gst-Gbf;
    noisedB = noisedB+Gproc;
    
    Xdb = pow2db(Xpow);
    Xdb = Xdb-noisedB;
    noisedB = 0;
    
    % Find targets and compute SNR
    Pfa = 1e-6;
    threshdB = npwgnthresh(Pfa);
    threshdB = threshdB+3;
    idx = helperFindPeaks2D(Xdb,threshdB,[3 3]);
    idx = idx(idx>0);
    [rdx,cdx] = ind2sub(size(Xdb),idx);
    idx = sub2ind(size(Xdb),rdx,cdx);
    snrdB = Xdb(idx);
    rg = rnggrid(rdx);
    az = anggrid(cdx);
    
    [fig,isNew] = helperFigureName('Beamformed, Range-Doppler');
    visState = fig.Visible;
    fig.Visible = 'off';
    clf(fig);
    
    ax = axes(fig);
    snrMax = ceil(max(Xdb(:)/10)*10);
    if isvector(Xdb)
        plot(ax,rnggrid,Xdb);
    
        ylim(ax,[floor(noisedB/10)*10 ceil(signaldB/10)*10]+[-10 5]);
    
        hold(ax,'on');
        plot(ax,rnggrid([1 end]),noisedB*[1 1],'k-.','LineWidth',2);
        plot(ax,rnggrid([1 end]),signaldB*[1 1],'r-.');
        hold(ax,'off');
    
        xlabel(ax,'Range (m)');
        ylabel(ax,'SNR (dB)');
        grid(ax,'on'); grid(ax,'minor');
        xlim(ax,[0 2*rg]);
    else
        [angs,rgs] = meshgrid(anggrid,rnggrid);
        [x,y] = pol2cart(deg2rad(angs),rgs);
    
        loc = rtxrx.MountingLocation;
        x = x+loc(1);
        y = y+loc(2);
    
        himg = mesh(ax,x,y,Xdb);
        himg.FaceColor = 'interp';
        view(ax,-90,90);
        xlabel(ax,'X (m)');
        ylabel(ax,'Y (m)');
        axis(ax,'equal');
        axis(ax,[0 60 20*[-1 1]]);
    
        hold(ax,'on');
        [x,y] = pol2cart(deg2rad(az(:)),rg(:));
        x = x+loc(1);
        y = y+loc(2);
        h = plot3(ax,x,y,110*ones(size(x)),'ko','MarkerFaceColor','w');
        hold(ax,'off');
    
        set(get(colorbar(ax),'Ylabel'),'String','SNR (dB)');
        caxis(ax,[0 snrMax]);
        legend(h,'Local maxima');
    end
    title(ax,sprintf('Beamformed, Range, and Doppler'));
    if true || isNew
        fig.Visible = 'on';
    else
        fig.Visible = visState;
    end
    end
    