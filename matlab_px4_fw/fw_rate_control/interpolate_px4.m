function y = interpolate_px4(x, x0, x1, y0, y1)
%INTERPOLATE_PX4 Linear interpolation helper used by PX4 trim scheduling.
if abs(x1 - x0) < eps
    y = y0;
else
    t = (x - x0) / (x1 - x0);
    y = y0 + t * (y1 - y0);
end
end
