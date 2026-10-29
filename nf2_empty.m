function F = nf2_empty(nAnch, nRows)
%NF2_EMPTY  Create a frame batch: the common data format used everywhere.
%   F = nf2_empty(nAnch, nRows)   (nRows default 0)
%   Every field has one row per frame:
%     t       time (s), from the tag's system_time
%     sysTime same as t (kept for reference)
%     D       ranges to A0..A(n-1) (m), NaN if that anchor is missing
%     fp, rx  first-path / total received power per anchor (dB)
%     posNA   NAssistant's own position [x y z] (m)
%     velNA   NAssistant's own velocity [vx vy vz] (m/s)
%     eop     NAssistant's position error estimate [x y z] (m)
%     voltage tag supply voltage (V)
%     nNodes  number of anchors in the frame
if nargin < 1, nAnch = 4; end
if nargin < 2, nRows = 0; end
F.t       = nan(nRows, 1);
F.sysTime = nan(nRows, 1);
F.D       = nan(nRows, nAnch);
F.fp      = nan(nRows, nAnch);
F.rx      = nan(nRows, nAnch);
F.posNA   = nan(nRows, 3);
F.velNA   = nan(nRows, 3);
F.eop     = nan(nRows, 3);
F.voltage = nan(nRows, 1);
F.nNodes  = zeros(nRows, 1);
end
