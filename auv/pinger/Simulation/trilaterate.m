function angs = trilaterate(m, bl, SoS)
    % angs = trilaterate(measured_delta_lags, baseline, speed_of_sound)
    %
    % Computes bearing and elevation to pinger for a given set of measured
    % lag times (equitriangular hydrophone array is assumed).
    %
    % See attached Mathematica notebook for derivation. 
    %
    % Hugo Vincent 25 January 2010.
    
    angs = zeros(2,1);
    
    % Compute bearing:
    rt3 = sqrt(3);
    angs(1) = 2 * atan2(m(2) - m(1) + 2 * m(3) - 2 * sqrt(...
        sum(m.^2) + m(1) * m(2) + m(2) * m(3) - m(1) * m(3)), rt3 * (m(1) + m(2)));
    % Note: make sure error handling is correct in the case that the sqrt
    % returns a complex number and atan2 is thus undefined.
    
    % Compute elevation:
    angs(2) = acos(min(1, -SoS / (9e6 * bl) * (rt3 * (m(1) - m(2) - 2 * m(3)) ...
        * cos(angs(1)) + 3 * (m(1) + m(2)) * sin(angs(1)))));
    % ^^^ this min(1, ...) is to prevent complex numbers being returned when
    % noise gives rise to an inconsistent set of lags.
        
    % FIXME could do better here from a numerical precision sensitivity
    % point of view...
    
    % Normalize and sanity-check angles
    angs = rad2deg(angs);
    if angs(1) < 0
        angs(1) = angs(1) + 360;
    end
    % FIXME what to do about normalizing elevation angle?
end
