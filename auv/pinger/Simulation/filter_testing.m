% Filter Testing
pinger_freq = 12e3;
%[data, Fs] = wavread('/Users/hugo/hydrophones_signal_noise.wav');
[data, Fs] = wavread('/Users/hugo/hydrophones_long_pings.wav');
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

