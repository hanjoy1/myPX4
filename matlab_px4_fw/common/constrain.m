function y = constrain(x, lo, hi)
%CONSTRAIN PX4-style scalar/vector saturation.
y = min(max(x, lo), hi);
end
