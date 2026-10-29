function m = uwb_nanmean(x)
%UWB_NANMEAN  Mean ignoring NaN (works in MATLAB and Octave).
m = mean(x(isfinite(x)));
end
