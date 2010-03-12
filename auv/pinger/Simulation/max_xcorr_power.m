function lag = max_xcorr_power(x1_fft, x2_fft)
    % Find lag (in samples) of maximum cross-correlation between x1 and x2.
    % Inputs should be FFT'd already.
    %
    % Minimal, naive implementation of cross-correlation for this
    % application
    %
    % Hugo Vincent, 19 Jan 2010
    
    % Compute cross-correlation and transform back into time domain
    y = ifft(x1_fft .* conj(x2_fft));
    
    % Compute power
    y = y.^2;
    
    % Munge array index into time lag in samples
    lags = [1:length(x1_fft)/2 -length(x1_fft)/2:-1];
    
    % Find maximum
    [~, ind] = max(y);
    lag = lags(ind);
    
    % FIXME could do something smarter here (e.g. piecewise linear fit ->
    % interpolate maximum with sub-sample precision)
end
