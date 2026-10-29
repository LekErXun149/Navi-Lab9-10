function F = nf2_load(file, nAnch)
%NF2_LOAD  Load a recording into the common frame-batch format.
%   F = nf2_load(file)          nAnch defaults to 4
%   Supported files:
%     .xlsx / .csv : NAssistant "Export" of NLink_LinkTrack_Node_Frame2
%     .bin         : raw serial bytes (saved by UwbSource in live mode)
%
%   NAssistant export columns: 2 systemTime(ms), 3-5 eop, 6-8 pos, 9-11 vel,
%   25 voltage, 26 nodeCount, then per node 5 columns [role id dis fpRssi rxRssi]
%   starting at column 27. Node blocks are matched to anchors by their id.

if nargin < 2, nAnch = 4; end
[~, ~, ext] = fileparts(char(file));
switch lower(ext)
    case '.bin'
        fid = fopen(file, 'r'); assert(fid > 0, 'Cannot open %s', file);
        raw = fread(fid, Inf, 'uint8=>uint8'); fclose(fid);
        [F, ~, st] = nf2_parse(raw, nAnch);
        fprintf('%s: %d frames, %d bad checksums\n', file, st.good, st.badChecksum);
        return
    case {'.xlsx', '.xls', '.csv'}
        M = read_numeric(file);
    otherwise
        error('Unsupported file type: %s', ext);
end

M = M(any(~isnan(M), 2), :);                 % drop empty rows
n = size(M, 1);
F = nf2_empty(nAnch, n);
F.sysTime = M(:, 2) / 1000;
F.t       = F.sysTime;
F.eop     = M(:, 3:5);
F.posNA   = M(:, 6:8);
F.velNA   = M(:, 9:11);
F.voltage = M(:, 25);
F.nNodes  = M(:, 26);
nBlk = floor((size(M, 2) - 26) / 5);
for k = 0:nBlk-1
    c   = 27 + 5*k;
    ids = M(:, c+1);
    for r = find(~isnan(ids) & ids >= 0 & ids < nAnch)'
        a = ids(r) + 1;
        F.D(r, a)  = M(r, c+2);
        F.fp(r, a) = M(r, c+3);
        F.rx(r, a) = M(r, c+4);
    end
end
end

function M = read_numeric(file)
if exist('readmatrix', 'file') || exist('readmatrix', 'builtin')
    M = readmatrix(file);                      % MATLAB R2019a+
else                                           % Octave fallback
    [~, ~, ext] = fileparts(char(file));
    if strcmpi(ext, '.csv')
        M = dlmread(file, ',', 1, 0, 'emptyvalue', NaN);
    else
        pkg load io; M = xlsread(file);
    end
end
end
