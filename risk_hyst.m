function [on, mem] = risk_hyst(cond, t, condRelease, dwell, mem)
%RISK_HYST  Hysteresis + dwell for one warning flag (E5). One sample per call.
%   [on, mem] = risk_hyst(cond, t, condRelease, dwell, mem)
%     cond        true -> switch ON immediately (stricter = instant)
%     t           time (s)
%     condRelease true -> allowed to switch OFF (e.g. distance > threshold + gap)
%     dwell       release only after condRelease has held for this long (s)
%     mem         [] on first call
%   Switching on is immediate; switching off needs the release condition to hold
%   continuously for 'dwell' seconds. This stops flickering at a boundary.

if isempty(mem), mem = struct('on', false, 'tRelease', NaN); end
if cond
    mem.on = true; mem.tRelease = NaN;
elseif mem.on
    if condRelease
        if isnan(mem.tRelease), mem.tRelease = t; end
        if t - mem.tRelease >= dwell, mem.on = false; mem.tRelease = NaN; end
    else
        mem.tRelease = NaN;
    end
end
on = mem.on;
end
