%% Two-ray Propagation
% in reality, the actual signal propagation
% between the radar and the target vehicle is more complicated than what is
% modeled so far. For example, the radio wave can also arrive at the target
% person via reflections. A simple yet widely used model to describe such
% a multipath scenario is a two-ray model, where the signal propagates from
% the radar to the target person via two paths, one direct path and one
% reflected path off the ground.

% The reflection off the road impacts the phase of the signal and the
% receiving signal at the target person is a coherent combination of the
% signals via the two paths. Same thing happens on the return trip too
% where the reflected signal off the target vehicle travels back to the
% radar. Hence depending on the distance between the radar and the target,the signals
% from different paths may add constructively or destructively, making
% signal strength fluctuating over time. Such fluctuation can pose some
% challenge in the successive detection stage.
%
% To showcase the multipath effect, next section uses the two ray channel
% model to propagate the signal between the radar and the target person.

% the transmitting 2 ray chanell
txchannel = phased.TwoRayChannel('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs);

% the recieving 2 ray chanell
rxchannel = phased.TwoRayChannel('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs);
Nsweep = 16;


if isa(waveform,'phased.MFSKWaveform')
    sweeptime = waveform.StepTime*waveform.StepsPerSweep;
else
    sweeptime = waveform.SweepTime;
end

Nsamp = round(waveform.SampleRate*sweeptime);

xr = complex(zeros(Nsamp,Nsweep));
xr_unmixed = xr;

for m = 1:Nsweep

    % Update radar and target positions
    [radar_pos,radar_vel] = radarmotion(sweeptime);
    [tgt_pos,tgt_vel] = chestmotion(sweeptime);

    % Transmit FMCW waveform
    sig = waveform();
    txsig = transmitter(sig);
    
    % propagate the signal
    txsig = txchannel(txsig,radar_pos,tgt_pos,radar_vel,tgt_vel);  
    
    % reflect off the target
    rxsig = chestTarget(txsig);
    
    % propagate the signal
    rxsig = rxchannel(rxsig,radar_pos,tgt_pos,radar_vel,tgt_vel);    
    
    % Dechirp the received radar return
    rxsig = receiver(rxsig);
    xd = dechirp(rxsig,sig);
    xr_unmixed(:,m) = rxsig;
    specanalyzer([txsig xd]);  
    xr(:,m) = xd;    
end

% % Before estimating the value of the range and Doppler, it may be a good
% % idea to take a look at the zoomed range Doppler response of 
% % 64 sweeps in an iteration
% 
 rngdopresp = phased.RangeDopplerResponse('PropagationSpeed',c,...
     'DopplerOutput','Speed','OperatingFrequency',fc,'SampleRate',fs,...
     'RangeMethod','FFT','SweepSlope',sweep_slope,...
     'RangeFFTLengthSource','Property','RangeFFTLength',2048,...
     'DopplerFFTLengthSource','Property','DopplerFFTLength',256);

 clf;
plotResponse(rngdopresp,xr);                     % Plot range Doppler map
axis([-v_max v_max 0 range_max]);
% caxis(clim);

% % There are many ways to estimate the range and speed of the target .
% % For example, one can choose almost any spectral analysis method to
% % extract both the beat frequency and the Doppler shift. This example uses
% % the root MUSIC algorithm to extract both the beat frequency and the
% % Doppler shift.
% %
% % As a side note, although the received signal is sampled at 1.5 GHz so the
% % system can achieve the required range resolution, after the dechirp, one
% % only needs to sample it at a rate that corresponds to the maximum beat
% % frequency. Since the maximum beat frequency is in general less than the
% % required sweeping bandwidth, the signal can be decimated to alleviate the
% % hardware cost. The following code snippet shows the decimation process.
% 
 Dn = fix(fs/(2*fb_max));
 for m = size(xr,2):-1:1
     xr_d(:,m) = decimate(xr(:,m),Dn,'FIR');
 end
 fs_d = fs/Dn;
% 
% %%
% % To estimate the range, firstly, the beat frequency is estimated using the
% % coherently integrated sweeps and then converted to the range.
% 
 fb_rng = rootmusic(pulsint(xr_d,'coherent'),1,fs_d);
 rng_est = beat2range(fb_rng,sweep_slope,c)  % note that R=c*delta_t/2

 % Second, the Doppler shift is estimated across the sweeps at the range
 % where the target is present.

 peak_loc = val2ind(rng_est,c/(fs_d*2));
 fd = -rootmusic(xr_d(peak_loc,:),1,1/tm);
 v_est = dop2speed(fd,lambda)/2

% % Note that both range and Doppler estimation are quite accurate.
%% after that we store the range value into a the rangeEstimation buffer
rangeEstimations(1,i)=rng_est;
rangeEstimations(2,i)=i*interval;





%%
% With all settings remaining same, the comparison of the resulting
% range-Doppler map with two-ray propagation and the range-Doppler map
% obtained before with a line of sight (LOS) propagation channel suggests
% that the signal strength dropped almost 40 dB, which is significant.
% Therefore, such effect must be considered during the design. One possible
% choice is to form a very sharp beam on the vertical direction to null out
% the reflections.

%% Summary
% This example shows how to use FMCW signal to perform range and Doppler
% estimation in an automotive automatic cruise control application. The
% example also shows how to generate a range Doppler map from the received
% signal and discussed how to use triangle sweep to compensate for the
% range Doppler coupling effect for the FMCW signal. Finally, the effect on
% the signal level due to the multipath propagation is discussed.