function angs = trilaterate2(m, bl, SoS)
    % angs = trilaterate(measured_delta_lags, baseline, speed_of_sound)
    %
    % Computes bearing and elevation to pinger for a given set of measured
    % lag times (equitriangular hydrophone array is assumed).
    %
    % See attached Mathematica notebook for derivation. 
    %
    % Hugo Vincent 25 January 2010.
    
    angs = zeros(2,1);
    angs(1) = fminbnd(@(x) lag_error(      x, 0, m, bl, SoS), 0, 2*pi);
    angs(2) = fminbnd(@(x) lag_error(angs(1), x, m, bl, SoS), 0, pi/2);
    angs = rad2deg(angs);
end

function err = lag_error(bearing, elevation, m, bl, SoS)
    a = sqrt(3) * cos(bearing);
    b =      3  * sin(bearing);
    est_lags = [-0.5 * (a + b); 0.5 * (a - b); a];
    est_lags = est_lags * 1e6 * cos(elevation) * bl / SoS;
    err = sum((m - est_lags).^2);
end
