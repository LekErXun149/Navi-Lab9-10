function [F, rest, stats] = nf2_parse(buf, nAnch)
%NF2_PARSE  Decode NLink_LinkTrack_Node_Frame2 frames from a raw byte stream.
%   [F, rest, stats] = nf2_parse(buf, nAnch)
%     buf   : uint8 vector of received bytes (may start/end mid-frame)
%     nAnch : number of anchor columns to fill (default 4; anchor id k -> column k+1)
%   Returns
%     F     : frame batch (see nf2_empty) with one row per valid frame
%     rest  : unused trailing bytes - prepend them to the next read
%     stats : .good, .badChecksum, .skipped (bytes discarded while syncing)
%
%   Layout (LinkTrack User Manual V2.1, section 8.1.2.5, little-endian):
%     0 header 0x55 | 1 mark 0x04 | 2-3 frame length | 4 role | 5 id
%     6-9 system_time (ms) | 10-12 eop*100 | 13-21 pos*1000 int24 x3
%     22-30 vel*10000 int24 x3 | ... | 116-117 voltage*1000 | 118 node count N
%     119 + 13k : role, id, dis*1000 int24, fp_rssi*(-2), rx_rssi*(-2), 6 reserved
%     last byte : checksum = sum of all previous bytes, low 8 bits

if nargin < 2, nAnch = 4; end
buf = uint8(buf(:)');
F = nf2_empty(nAnch);
stats = struct('good', 0, 'badChecksum', 0, 'skipped', 0);
HDR = uint8(85); MARK = uint8(4);          % 0x55, 0x04
i = 1; n = numel(buf);
rows = {};

while i + 3 <= n
    if buf(i) ~= HDR || buf(i+1) ~= MARK
        i = i + 1; stats.skipped = stats.skipped + 1; continue;
    end
    L = double(buf(i+2)) + 256*double(buf(i+3));
    if L < 120 || L > 1200                       % not a plausible Node_Frame2 length
        i = i + 1; stats.skipped = stats.skipped + 1; continue;
    end
    if i + L - 1 > n, break; end                 % incomplete frame: wait for more bytes
    fr = double(buf(i:i+L-1));
    if mod(sum(fr(1:end-1)), 256) ~= fr(end)
        stats.badChecksum = stats.badChecksum + 1;
        i = i + 1; continue;
    end
    rows{end+1} = decode_one(fr, nAnch); %#ok<AGROW>
    stats.good = stats.good + 1;
    i = i + L;
end
rest = buf(i:end);
if ~isempty(rows), F = nf2_concat(F, rows{:}); end
end

% -------------------------------------------------------------------------
function r = decode_one(fr, nAnch)
% fr is 1-based: byte offset k in the manual is fr(k+1)
u32 = @(k) fr(k+1) + 256*fr(k+2) + 65536*fr(k+3) + 16777216*fr(k+4);
r = nf2_empty(nAnch, 1);
r.sysTime  = u32(6) / 1000;                             % s
r.t        = r.sysTime;
r.posNA    = [i24(fr,13) i24(fr,16) i24(fr,19)] / 1000; % m (NAssistant's own solution)
r.velNA    = [i24(fr,22) i24(fr,25) i24(fr,28)] / 10000;% m/s
r.eop      = fr(11:13) / 100;
r.voltage  = (fr(117) + 256*fr(118)) / 1000;
N          = fr(119);
r.nNodes   = N;
for k = 0:N-1
    b = 120 + 13*k;                                     % 1-based start of block k
    if b + 12 > numel(fr) - 1, break; end
    id = fr(b+1);
    if id < nAnch
        c = id + 1;
        r.D(c)  = i24(fr, b+1) / 1000;                  % dis starts at 0-based offset b+1
        r.fp(c) = -fr(b+5) / 2;
        r.rx(c) = -fr(b+6) / 2;
    end
end
end

function v = i24(fr, k)
% signed 24-bit little-endian integer starting at 0-based byte offset k
v = fr(k+1) + 256*fr(k+2) + 65536*fr(k+3);
if v >= 8388608, v = v - 16777216; end
end
