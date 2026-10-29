function s = uwb_nansum(x)
%UWB_NANSUM  Sum ignoring NaN (works in MATLAB and Octave).
s = sum(x(isfinite(x)));
end
