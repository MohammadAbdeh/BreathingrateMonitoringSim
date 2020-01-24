% Next, run the simulation loop. 

 rng(2012);
 Nsweep = 64;
 xr = complex(zeros(waveform.SampleRate*waveform.SweepTime,Nsweep));
 n=0;
 for m = 1:Nsweep
     n=n+1;
     % Update radar and target positions
     [radar_pos,radar_vel] = radarmotion(waveform.SweepTime);
     [tgt_pos,tgt_vel] = chestmotion(waveform.SweepTime);
  
     % Transmit FMCW waveform
     sig = waveform();
     txsig = transmitter(sig);
     
     % Propagate the signal and reflect off the target
     txsig = channel(txsig,radar_pos,tgt_pos,radar_vel,tgt_vel);

     txsig = chestTarget(txsig);

     % Dechirp the received radar return
     txsig = receiver(txsig);   
     dechirpsig = dechirp(txsig,sig);
     
     % Visualize the spectrum
     specanalyzer([txsig dechirpsig]);  
    xr(:,m) = dechirpsig;
 end
% 
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
 axis([-v_max v_max 0 range_max])
%  clim = caxis;
% 
% %%
% % From the range Doppler response, one can see that the car in front is a
% % bit more than 40 m away and appears almost static. This is expected
% % because the radial speed of the car relative to the radar is only 4 km/h,
% % which translates to a mere 1.11 m/s.
% %
% 
% %%

% % From the spectrum scope, one can see that although the received signal is
% % wideband (channel 1), sweeping through the entire bandwidth, the
% % dechirped signal becomes narrowband (channel 2). 
% 
% %% Range and Doppler Estimation

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
% % To estimate the range, firstly,the beat frequency is estimated using the
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



