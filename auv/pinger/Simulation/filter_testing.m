% Pinger Output specs
pinger_freq = 12e3; % 12 kHz
det_filter_width = 50; % width of peak filter in Hz for detector

% Sound card specs
sample_freq = 96e3; % 96 kHz
snr = 12; % signal-to-noise ratio in decibels

% Filter Testing
pinger_freq = 12e3;
%[data, Fs] = wavread('/Users/hugo/hydrophones_signal_noise.wav');
[data, Fs] = wavread('/Users/hugo/hydrophones_long_pings.wav');
%[data, Fs] = wavread('/Users/hugo/Downloads/hydro3.wav');
%soundsc(data, Fs);
sample_freq = Fs;
[peakfilt_num, peakfilt_den] = iirpeak(pinger_freq/(sample_freq/2), ...
    det_filter_width/(sample_freq/2));

% Run IIR filter
data2 = filter(peakfilt_num, peakfilt_den, data);
%soundsc(data2, Fs);

% Power
data2 = data2.^2;
figure(1); plot(data2);

% Envelope detector
threshold = 1e-5;
data3 = data2 > threshold;
figure(2); plot(data3); ylim([-0.1 1.1]);

