%% Automotive Adaptive Cruise Control Using FMCW Technology
% This example shows how to model an automotive adaptive cruise control
% system using the frequency modulated continuous wave (FMCW) technique.
% This example performs range and Doppler estimation of a moving vehicle.
% Unlike pulsed radar systems that are commonly seen in the defense
% industry, automotive radar systems often adopt FMCW technology. Compared
% to pulsed radars, FMCW radars are smaller, use less power, and are much
% cheaper to manufacture. As a consequence, FMCW radars can only monitor a
% much smaller distance.

%   Copyright 2012-2016 The MathWorks, Inc.
 
%% FMCW Waveform 
% Consider an automotive long range radar (LRR) used for adaptive cruise
% control (ACC). This kind of radar usually occupies the band around 77
% GHz, as indicated in [1]. The radar system constantly estimates the
% distance between the vehicle it is mounted on and the vehicle in front of
% it, and alerts the driver when the two become too close. The figure below
% shows a sketch of ACC.
%
% <<../FMCWExample_acc.png>>
%
% A popular waveform used in ACC system is FMCW. The principle of range
% measurement using the FMCW technique can be illustrated using the
% following figure.
%
% <<../FMCWExample_upsweep.png>>
%
% The received signal is a time-delayed copy of the transmitted signal
% where the delay, $\Delta t$, is related to the range. Because the signal
% is always sweeping through a frequency band, at any moment during the
% sweep, the frequency difference, $f_b$, is a constant between the
% transmitted signal and the received signal. $f_b$ is usually called the
% beat frequency. Because the sweep is linear, one can derive the time
% delay from the beat frequency and then translate the delay to the range.
%
% In our example, the maximum range the radar needs to monitor is around
% 20 m and the system needs to be able to distinguish two targets that are
% 10 centimeter apart. From these requirements, one can compute the waveform
% parameters.

fc = 7e9;  %operating frequency
c = 3e8;   %propagation speed(the speed of light)
lambda = c/fc; %the wavelength

%%
% The sweep time can be computed based on the time needed for the signal to
% travel the unambiguous maximum range. In general, for an FMCW radar
% system, the sweep time should be at least 5 to 6 times the round trip
% time. This example uses a factor of 5.5.

range_max = 20;
tm = 5.5*range2time(range_max,c);

%%
% The sweep bandwidth can be determined according to the range resolution
% and the sweep slope is calculated using both sweep bandwidth and sweep
% time.

range_res = 0.1;
bw = range2bw(range_res,c);
sweep_slope = bw/tm;

%%
% Because an FMCW signal often occupies a huge bandwidth, setting the
% sample rate blindly to twice the bandwidth often stresses the capability
% of A/D converter hardware. To address this issue, one can often choose a
% lower sample rate. Two things can be considered here:

% # For a complex sampled signal, the sample rate can be set to the same as
% the bandwidth.
% # FMCW radars estimate the target range using the beat frequency embedded
% in the dechirped signal. The maximum beat frequency the radar needs to
% detect is the sum of the beat frequency corresponding to the maximum
% range and the maximum Doppler frequency. Hence, the sample rate only
% needs to be twice the maximum beat frequency.
%
% In this example, the beat frequency corresponding to the maximum range is
% given by

fr_max = range2beat(range_max,sweep_slope,c);

%%
% In addition, the maximum speed of a traveling car is about 230 km/h.
% Hence the maximum Doppler shift and the maximum beat frequency can be
% computed as

maxbreathingRate=40;
forwordExpantion=2;
maxDisInMinute=2*maxbreathingRate*forwordExpantion;

v_max =maxDisInMinute/(100*60) ;
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;


%%
% This example adopts a sample rate of the larger of twice the maximum beat
% frequency and the bandwidth.

fs = max(2*fb_max,bw);

%%
% The following table summarizes the radar parameters.
% 
%  System parameters            Value
%  ----------------------------------
%  Operating frequency (GHz)    77
%  Maximum target range (m)     200
%  Range resolution (m)         1
%  Maximum target speed (km/h)  230
%  Sweep time (microseconds)    7.33
%  Sweep bandwidth (MHz)        150
%  Maximum beat frequency (MHz) 27.30
%  Sample rate (MHz)            150


%%
% With all the information above, one can set up the FMCW waveform used
% in the radar system.

waveform = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,...
    'SampleRate',fs);

%%
% This is a up-sweep linear FMCW signal, often referred to as sawtooth
% shape. One can examine the time-frequency plot of the generated signal.

sig = waveform();
subplot(211); plot(0:1/fs:tm-1/fs,real(sig));
xlabel('Time (s)'); ylabel('Amplitude (v)');
title('FMCW signal'); axis tight;
subplot(212); spectrogram(sig,32,16,32,fs,'yaxis');
title('FMCW signal spectrogram');

%% Target Model
% The target of our radar is a man chest in front of it. This example
% assumes the man is breath?ng 16 breath per m?nute 
%

format long     %to take ?nto consederat?on more acurate measurments 
 senarioBreathingRate=16;
 chest_dist = 9;
 chest_speed = 2*2*senarioBreathingRate/(60*100);
 chest_rcs = 1.8;
 
 chestTarget = phased.RadarTarget('MeanRCS',chest_rcs,'PropagationSpeed',c...
     ,'OperatingFrequency',fc);
 

 chestmotion = phased.Platform('InitialPosition',[chest_dist;0;0.5],...
    'Velocity',[chest_speed;0;0]);



%%
% The propagation model is assumed to be free space for now.
% 

 channel = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);

% the transmitting 2 ray chanell
% txchannel = phased.TwoRayChannel('PropagationSpeed',c,...
%     'OperatingFrequency',fc,'SampleRate',fs);
% 
% % the recieving 2 ray chanell
% rxchannel = phased.TwoRayChannel('PropagationSpeed',c,...
%     'OperatingFrequency',fc,'SampleRate',fs);

%% Radar System Setup
% The rest of the radar system includes the transmitter, the receiver, and
% the antenna. This example uses the parameters presented in [4]. Note that
% this example models only main components and omits the effect from other
% components, such as coupler and mixer. In addition, for the sake of
% simplicity, the antenna is assumed to be isotropic and the gain of the
% antenna is included in the transmitter and the receiver.

 ant_aperture = 6.06e-4;                         % in square meter
 ant_gain = aperture2gain(ant_aperture,lambda);  % in dB
 
 tx_ppower = db2pow(5)*1e-3;                     % in watts
 tx_gain = 9+ant_gain;                           % in dB
 
 rx_gain = 15+ant_gain;                          % in dB
 rx_nf = 4.5;                                    % in dB
  
 transmitter = phased.Transmitter('PeakPower',tx_ppower,'Gain',tx_gain);
 receiver = phased.ReceiverPreamp('Gain',rx_gain,'NoiseFigure',rx_nf,...
     'SampleRate',fs);

%%
% Automotive radars are generally mounted on vehicles, so they are often in
% motion. This example assumes the radar is traveling at a speed of 100
% km/h along x-axis. So the target car is approaching the radar at a
% relative speed of 4 km/h.

 radar_speed = 0;
 radarmotion = phased.Platform('InitialPosition',[0;0;0.5],...
    'Velocity',[radar_speed;0;0]);

%% Radar Signal Simulation
% As briefly mentioned in earlier sections, an FMCW radar measures the
% range by examining the beat frequency in the dechirped signal. To extract
% this frequency, a dechirp operation is performed by mixing the received
% signal with the transmitted signal. After the mixing, the dechirped
% signal contains only individual frequency components that correspond to
% the target range.
% 
% In addition, even though it is possible to extract the Doppler
% information from a single sweep, the Doppler shift is often extracted
% among several sweeps because within one pulse, the Doppler frequency is
% indistinguishable from the beat frequency. To measure the range and
% Doppler, an FMCW radar typically performs the following operations:
%
% # The waveform generator generates the FMCW signal.
% # The transmitter and the antenna amplify the signal and radiate the
% signal into space.
% # The signal propagates to the target, gets reflected by the target, and
% travels back to the radar.
% # The receiving antenna collects the signal.
% # The received signal is dechirped and saved in a buffer.
% # Once a certain number of sweeps fill the buffer, the Fourier transform
% is performed in both range and Doppler to extract the beat frequency as
% well as the Doppler shift. One can then estimate the range and speed of
% the target using these results. Range and Doppler can also be shown as an
% image and give an intuitive indication of where the target is in the
% range and speed domain.
%
% The next section simulates the process outlined above. A total of 64
% sweeps are simulated and a range Doppler response is generated at the
% end.
%
% During the simulation, a spectrum analyzer is used to show the spectrum
% of each received sweep as well as its dechirped counterpart.
% 
 specanalyzer = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
     'ShowLegend',true);
%     

%%
%run the simulation
interval=0.01; %time interval in minutes between measurments.
iteration=400;
rangeEstimations=zeros(2,iteration);
% timeOfFlights=zeros(2,iteration);
% phases=zeros(2,iteration*64);


% fft parameters
samplingFreq=1/interval      % Sampling frequency 
T = interval;             % Sampling period       
L = iteration;               % Length of signal
k = (0:T:L-1);        % Time vector

% counter used to traverse the direction of the velocity due to inhaling
% exhaling
velocityChangingCount=0; 
%% the simlation loop
 Nsweep = 2;
 
for i = 1:iteration
    
    velocityChangingCount= velocityChangingCount+1;      
    %traversing the direction of velocity(changing the breathing state)
    if velocityChangingCount==100
         chest_speed = -1 * chest_speed;
         velocityChangingCount=0;
    end   
    
    %%updating the chest and radar positions 
    chestmotion.Velocity = [chest_speed;0;0];
    [radar_pos,radar_vel] = radarmotion(interval);
    [tgt_pos,tgt_vel] = chestmotion(interval);

    %timer used to run the simulation script every interval.
    t = timer('TimerFcn', 'stat=false; run("towRayPropSimulation")',... 
                     'StartDelay',interval);

    % starring the timer             
    start(t)
    %waiting untill the process is done
    wait(t)
end

%% spoting the range estimations 
figure(1)
plot(rangeEstimations(2,:),rangeEstimations(1,:))
title('Range Esimations while breathing')
xlabel('Time (s)')
ylabel('|Range(m)|')

% rangeEstimations=rangeEstimations-mean(rangeEstimations);
rangeEstimations(1,:) = detrend(rangeEstimations(1,:))
Y=fft(rangeEstimations(1,:));
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = samplingFreq*(0:(L/2))/L;  %the frequency vector
figure(2)
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)\|')

index=find(f==1);
peak=max(P1(:));
peakIndex=find(P1==peak);
s1=((k(end)+1)*interval); % simulation duration
s2=abs(peakIndex-index); % number of breaths;
breathingRate=s2*60/s1; % breathing rate in minutes;

display(breathingRate);





 
 
 %% Range Doppler Coupling Effect
 % One issue associated with linear FM signals, such as an FMCW signal, is
 % the range Doppler coupling effect. As discussed earlier, the target range
 % corresponds to the beat frequency. Hence, an accurate range estimation
 % depends on an accurate estimate of beat frequency. However, the presence
 % of Doppler shift changes the beat frequency, resulting in a biased range
% % estimation. 
% %
% % For the situation outlined in this example, the range error caused by the
% % relative speed between the target and the radar is
% 
% deltaR = rdcoupling(fd,sweep_slope,c)
% 
% %%
% % This error is so small that we can safely ignore it. 
% % 
% % Even though the current design is achieving the desired performance, one
% % parameter warrants further attention. In the current configuration, the
% % sweep time is about 7 microseconds. Therefore, the system needs to sweep
% % a 150 MHz band within a very short period. Such an automotive radar may
% % not be able to meet the cost requirement. Besides, given the velocity of
% % a car, there is no need to make measurements every 7 microseconds. Hence,
% % automotive radars often use a longer sweep time. For example, the
% % waveform used in [2] has the same parameters as the waveform designed in
% % this example except a sweep time of 2 ms.
% %
% % A longer sweep time makes the range Doppler coupling more prominent. To
% % see this effect, first reconfigure the waveform to use 2 ms as the sweep
% % time.
% 
% waveform_tr = clone(waveform);
% release(waveform_tr);
% tm = 2e-3;
% waveform_tr.SweepTime = tm;
% sweep_slope = bw/tm;
% 
% %%
% % Now calculate the range Doppler coupling. 
% 
% deltaR = rdcoupling(fd,sweep_slope,c)
% 
% %%
% % A range error of 1.14 m can no longer be ignored and needs to be
% % compensated. Naturally, one may think of doing so following the same
% % procedure outlined in earlier sections, estimating both range and
% % Doppler, figuring out the range Doppler coupling from the Doppler shift,
% % and then remove the error from the estimate.
% %
% % Unfortunately this process doesn't work very well with the long sweep
% % time. The longer sweep time results in a lower sampling rate across the
% % sweeps, thus reducing the radar's capability of unambiguously detecting
% % high speed vehicles. For instance, using a sweep time of 2 ms, the
% % maximum unambiguous speed the radar system can detect using the
% % traditional Doppler processing is
% 
% v_unambiguous = dop2speed(1/(2*tm),lambda)/2
% 
% %%
% % The unambiguous speed is only 0.48 m/s, which means that the relative
% % speed, 1.11 m/s, cannot be unambiguously detected. This means that not
% % only the target car will appear slower in Doppler processing, the range
% % Doppler coupling also cannot be correctly compensated.
% %
% % One way to resolve such ambiguity without Doppler processing is to adopt
% % a triangle sweep pattern. Next section shows how the triangle sweep
% % addresses the issue.
% 
% %% Triangular Sweep
% % In a triangular sweep, there are one up sweep and one down sweep to form
% % one period, as shown in the following figure.
% %
% % <<../FMCWExample_trisweep.png>>
% %
% % The two sweeps have the same slope except different signs. From the
% % figure, one can see that the presence of Doppler frequency, $f_d$,
% % affects the beat frequencies ($f_{bu}$ and $f_{bd}$) differently in up
% % and down sweeps. Hence by combining the beat frequencies from both up and
% % down sweep, the coupling effect from the Doppler can be averaged out and
% % the range estimate can be obtained without ambiguity.
% 
% %%
% % First, set the waveform to use triangular sweep.
% 
% waveform_tr.SweepDirection = 'Triangle';
% 
% %%
% % Now simulate the signal return. Because of the longer sweep time,
% % fewer sweeps (16 vs. 64) are collected before processing.
% 
% Nsweep = 16;
% xr = helperFMCWSimulate(Nsweep,waveform_tr,radarmotion,carmotion,...
%     transmitter,channel,cartarget,receiver);
% 
% %%
% % The up sweep and down sweep are processed separately to obtain the beat
% % frequencies corresponding to both up and down sweep.
% 
% fbu_rng = rootmusic(pulsint(xr(:,1:2:end),'coherent'),1,fs);
% fbd_rng = rootmusic(pulsint(xr(:,2:2:end),'coherent'),1,fs);
% 
% %%
% % Using both up sweep and down sweep beat frequencies simultaneously, the
% % correct range estimate is obtained.
% 
% rng_est = beat2range([fbu_rng fbd_rng],sweep_slope,c)
% 
% %%
% % Moreover, the Doppler shift and the velocity can also be recovered in a
% % similar fashion.
% 
% fd = -(fbu_rng+fbd_rng)/2;
% v_est = dop2speed(fd,lambda)/2
% 
% %%
% % The estimated range and velocity match the true values, 43 m and 1.11
% % m/s, very well.

%% Summary
% This example shows how to use FMCW signal to perform range and Doppler
% estimation in an automotive automatic cruise control application. The
% example also shows how to generate a range Doppler map from the received
% signal and discussed how to use triangle sweep to compensate for the
% range Doppler coupling effect for the FMCW signal. Finally, the effect on
% the signal level due to the multipath propagation is discussed.

%% References
%
% [1] Karnfelt, C. et al.. _77 GHz ACC Radar Simulation Platform_, IEEE
% International Conferences on Intelligent Transport Systems
% Telecommunications (ITST), 2009.
%
% [2] Rohling, H. and M. Meinecke. _Waveform Design Principle for
% Automotive Radar Systems_, Proceedings of CIE International Conference on
% Radar, 2001.
