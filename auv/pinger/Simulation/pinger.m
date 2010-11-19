% Testing for the idea of hydrophone bearing measurement by correlation.
% Uses simulated data with "plausible" parameters.
% 
% Acoustic delays are generated in 2048x upsampled domain, so that delays
% are well modelled to within 0.1%. Simulated pinger output is then down-
% sampled to normal sampling frequency, noise added according to prescribed
% SNR, and quantized to 8-bits. This is the input to the algorithm.
% The correlation is performed after first upsampling 8x to get subsample
% precision in the output. 
%
% The signal is filtered in the frequency domain before the correlation so
% as not to cause unwanted phase/lag problems and to reduce CPU load. 
%
% TODO: 
%   * streaming processing -> add chirp detector and control logic
%   * simulate multipath (reflections off tank walls)
%   * investigate different angle representation (quaternions) esp wrt RMSE
%
% NOTES regarding real-world implementation:
%   * system gain will have to be calibrated. Probably best to do this
%     manually (rather than online-adaptively) with recorded data
%   * need to consider the effect of range-to-pinger on SNR and magnitude
%     of the received ping. Can we estimate distance from magnitude?
%   * will probably take the lags directly into the SLAM filter rather than
%     doing the bearing/elevation estimation here. This is known as a
%     tightly coupled filter. 
%   * there are optimizations that could be made wrt converting lags
%     to/from samples to seconds (may also help avoid numerical problems)
%
% Hugo Vincent, 16-27 Jan 2010

%--- Parameters -----------------------------------------------------------

% Pinger Output specs
pinger_freq = 12e3; % 12 kHz
chirp_time = 10e-3; % 10 msec
det_filter_width = 50; % width of peak filter in Hz for detector

% Sound card specs
sample_freq = 96e3; % 96 kHz
snr = 12; % signal-to-noise ratio in decibels

% Speed of sound in sea water (from wikipedia)
speed_of_sound = 1550; % m/sec

% Radius of circle that hydrophones lie on (each 120 degrees)
baseline = sin(deg2rad(60)) * 0.28; % measured dimensions of hydrophone mounts

% Processing upsample ratio
proc_rs = 8; % if CPU load is tight, this could be dropped to 4.

% FFT length to use for correlation
fftlen = 8192;
% fftlen = 2^nextpow2(2 * length(data) - 1); % Strictly correct version

% Hydrophone positions
mics = zeros(3,2);
[mics(1,1) mics(1,2)] = pol2cart(0,      baseline);
[mics(2,1) mics(2,2)] = pol2cart(2*pi/3, baseline);
[mics(3,1) mics(3,2)] = pol2cart(4*pi/3, baseline);
%printf('Distance along edge of hydrophone triangle = %g cm', ...
%    norm(mics(1,:) - mics(2,:)) * 100);

% SIMULATION PARAMETERS:
mcsize = 1; % number of particles for Monte Carlo 
gen_rs = 2048; % signal generation oversampling ratio

%--- Setup ----------------------------------------------------------------

% Filter that is used to BPF the data for the detector
[peakfilt_num, peakfilt_den] = iirpeak(pinger_freq/(sample_freq/2), ...
    det_filter_width/(sample_freq/2));

% Window for filtering upsampled FFT'd data.... FIXME could this be better?
center_freq_dft = pinger_freq/(sample_freq * proc_rs) * fftlen;
window = zeros(1, fftlen);
window((-50:50) + fftlen/2 + center_freq_dft) = triang(101);
window = [window(fftlen/2:end) window(1:(fftlen/2-1))];
window = window + window(end:-1:1);

% SIMULATION ONLY:
% Generate the reference pinger waveform, at Fs and upsampled 720x
waveform_rs = cos(2 * pi * pinger_freq * (0:1/(sample_freq*gen_rs):chirp_time));
lw = ceil(length(waveform_rs) / 2);

% Minimum amount of waveform on each side of ping for correlation
min_pad = (fftlen / proc_rs) * (gen_rs / 2); % 1 meter

%--- Run simulation -------------------------------------------------------

% Test pinger at different bearings
angles = 0:2:360;
elevs = 0:2:90;

performance_bearing = zeros(length(angles), length(elevs));
performance_elevation = zeros(length(angles), length(elevs));
for j = 1:length(angles)
    this_ang = angles(j);
    %matlabpool local 2
    for k = 1:length(elevs)
        this_elev = elevs(k);

        %--- Generate "truth" signals from pinger -------------------------
        % Find perpendicular distance to tangent plane (assumes incoming rays are parallel)
        mics_rot = mics * rotation_matrix(deg2rad(this_ang));
        distances = baseline + mics_rot(:,2);
        distances = distances * cos(deg2rad(this_elev));

        % Delay for each hydrophone hearing the signal
        delay_samples = ceil(sample_freq * (distances / speed_of_sound) * gen_rs);
        assert(max(delay_samples) <= min_pad, ...
            'Baseline distance too high for chosen fftlen and proc_rs');

        % Construct delayed waveform and downsample to sampling frequency
        waveform1 = downsample([zeros(1, min_pad + delay_samples(1) - lw) ...
            waveform_rs zeros(1, min_pad - delay_samples(1) - lw)], gen_rs);
        waveform2 = downsample([zeros(1, min_pad + delay_samples(2) - lw) ...
            waveform_rs zeros(1, min_pad - delay_samples(2) - lw)], gen_rs);
        waveform3 = downsample([zeros(1, min_pad + delay_samples(3) - lw) ...
            waveform_rs zeros(1, min_pad - delay_samples(3) - lw)], gen_rs);

        perftime = 0;
        decoded_bearings = zeros(1, mcsize);
        decoded_elevations = zeros(1, mcsize);
        for i = 1:mcsize
            %--- Generate noisy input to algorithm ------------------------

            % Add white Gaussian noise and random gain (+/- 1 decade)
            y1 = awgn((0.1 + 10*rand())*waveform1, snr, 'measured');
            y2 = awgn((0.1 + 10*rand())*waveform2, snr, 'measured');
            y3 = awgn((0.1 + 10*rand())*waveform3, snr, 'measured');

            % Add some quantization noise (55 dB SNR ~ 8-bit ADC)
            y1 = awgn(y1, 55, 'measured');
            y2 = awgn(y2, 55, 'measured');
            y3 = awgn(y3, 55, 'measured');

            % Quantize with 8-bit ADC
            y1 = double(int8(y1*128));
            y2 = double(int8(y2*128));
            y3 = double(int8(y3*128));

            % y1, y2, y3 now represent the three noisy, distorted microphone inputs

            %--- Detect/correlate pinger waveform delays ------------------
            tic;
            
            % FIXME streaming detector algorithm + control logic
            % FIXME extraction and centering on ping from stream
            %
            % Has to sample all three channels into a circular buffer in 
            % lock-step and then run the IIR filter on a copy of one channel.
            % Envelope detect output of the IIR filter to detect the ping, 
            % then position a window of fftlen/proc_fs such that it covers
            % the ping in all three channels, roughly centered. 
            
            % Check input window length is as assumed (see above comment)
            assert(length(y1) == fftlen/proc_rs && ...
                length(y2) == fftlen/proc_rs && ...
                length(y3) == fftlen/proc_rs, 'Input window is wrong length');
            
            % (a) Up-sample 8x to get subsample-precision time delay output
            % (i.e. insert 7 zeros between each sample)            
            y1 = upsample(y1, proc_rs);
            y2 = upsample(y2, proc_rs);
            y3 = upsample(y3, proc_rs);
            
            % (b) FFT the signals for use in correlation (and because we
            % upsampled without LPFing, we've got lots of symmetric copies
            % of the signal in our spectrum after the DFT).
            % NOTE: this is a real->complex FFT, so yfX is complex-valued,
            % not real after the transform
			yf1 = fft(y1, fftlen);
			yf2 = fft(y2, fftlen);
			yf3 = fft(y3, fftlen);
            
            % (c) Filter the signals (we do this by windowing the DFT)
            yf1 = yf1 .* window;
            yf2 = yf2 .* window;
            yf3 = yf3 .* window;

            % (d) Find each of the 3-way cross-correlation power maximums
            delta_lags = zeros(3,1);
            delta_lags(1) = max_xcorr_power(yf1, yf2);
            delta_lags(2) = max_xcorr_power(yf1, yf3);
            delta_lags(3) = max_xcorr_power(yf2, yf3);
            
            % (e) Convert to microseconds
            delta_lags = delta_lags .* (1e6 / (sample_freq * proc_rs));
            
            % (f) Decode angle (Trilateration)
            angs = trilaterate(delta_lags, baseline, speed_of_sound);
            % printf('Bearing %g Elev %g', angs(1), angs(2));
            
            % All Done!! Now store angles for analysis later
            decoded_bearings(i) = angs(1);
            decoded_elevations(i) = angs(2);
            
            perftime = perftime + toc;
        end
        
        % Summarize performance
        performance_bearing(j, k) = sqrt(mean(degrees_difference(...
            decoded_bearings, this_ang).^2));
        performance_elevation(j, k) = sqrt(mean(degrees_difference(...
            decoded_elevations, this_elev).^2));

        printf(['Angle: %g RMSE: %g -- Elev: %g ' ...
            'RMSE: %g :: Time %g ms\n'], this_ang, performance_bearing(j, k), ...
            this_elev, performance_elevation(j, k), perftime / mcsize * 1e3);
    end
end

% Plot results
figure(1);
subplot(2,1,1);
plot(angles, mean(performance_bearing(:,1:20), 2));
ylabel('RMSE (degrees) on Bearing');
xlabel('Bearing to pinger (degrees)');
xlim([0 360]);
title('Three-hydrophone Bearing performance at 18 dB SNR');
subplot(2,1,2);
plot(elevs, mean(performance_bearing, 1));
ylabel('RMSE (degrees) on Bearing');
xlabel('Declination to pinger (degrees)');
xlim([0 90]);
ylim([0 5]);

figure(2);
subplot(2,1,1);
plot(angles, mean(performance_elevation(:,end-20:end), 2));
ylabel('RMSE (degrees) on Elevation');
xlabel('Bearing to pinger (degrees)');
xlim([0 360]);
title('Three-hydrophone Elevation performance at 18 dB SNR');
subplot(2,1,2);
plot(elevs, mean(performance_elevation, 1));
ylabel('RMSE (degrees) on Elevation');
xlabel('Declination to pinger (degrees)');
xlim([0 90]);
