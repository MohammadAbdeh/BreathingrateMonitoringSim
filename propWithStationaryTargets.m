rng(2015);

if isa(waveform,'phased.MFSKWaveform')
    sweeptime = waveform.StepTime*waveform.StepsPerSweep;
else
    sweeptime = waveform.SweepTime;
end
Nsamp = round(waveform.SampleRate*sweeptime);

xr = complex(zeros(Nsamp,Nsweep));
xr_unmixed = xr;

Ntgt = numel(target.MeanRCS);
for m = 1:Nsweep
    % Update radar and target positions
    [radar_pos,radar_vel] = radarmotion(sweeptime);
    [tgt_pos,tgt_vel] = tgtmotion(sweeptime);

    % Transmit FMCW waveform
    sig = waveform();
    txsig = transmitter(sig);

    % Propagate the signal and reflect off the target
    rxsig = complex(zeros(Nsamp,Ntgt));
    
    for n = 1:Ntgt
        rxsig(:,n) = txchannel(txsig,radar_pos,tgt_pos(:,n),radar_vel,tgt_vel(:,n));
        
    end
        rxsig = target(rxsig);
    
    for n = 1:Ntgt
            
        rxsig(:,n) = rxchannel(rxsig,radar_pos,tgt_pos(:,n),radar_vel,tgt_vel(:,n));
    end
    
    % Dechirp the received radar return
    rxsig = receiver(sum(rxsig,2));
    xd = dechirp(rxsig,sig);
    
    % Visualize the spectrum
    specanalyzer([rxsig xd]);  
     
    xr_unmixed(:,m) = rxsig;
    xr(:,m) = xd;
end

dfactor = ceil(fs/fb_max)/2;
fs_d = fs/dfactor;
% 
% %%
% % To estimate the range, firstly,the beat frequency is estimated using the
% % coherently integrated sweeps and then converted to the range.
% 

fbc_rng = rootmusic(decimate(xr(:,1),dfactor),2,fs_d);
fbd_rng = rootmusic(decimate(xr(:,2),dfactor),2,fs_d);

rng_est = beat2range([fbc_rng fbd_rng;fbc_rng flipud(fbd_rng)],...
    sweep_slope,c) % note that R=c*delta_t/2


 % Second, the Doppler shift is estimated across the sweeps at the range
 % where the target is present.

%  peak_loc = val2ind(rng_est,c/(fs_d*2));
%  fd = -rootmusic(xr_d(peak_loc,:),1,1/tm);
%  v_est = dop2speed(fd,lambda)/2

% % Note that both range and Doppler estimation are quite accurate.
%% after that we store the range value into a the rangeEstimation buffer
% rangeEstimations(1,i)=rng_est;
% rangeEstimations(2,i)=i*interval;



