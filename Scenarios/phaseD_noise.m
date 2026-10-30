function err = phaseD_noise(profile, n, root)
%PHASED_NOISE  Real UWB range errors from the 26-Oct recordings, n frames x 4 anchors.
%   err = phaseD_noise('typical', n, root)   noise of the 15:33 recording (tag still):
%         each range minus its own median -> real noise and NLOS spikes, no bias
%   err = phaseD_noise('worst', n, root)     the tag-under-A3 recording (16:20) with tape
%         ground truth -> real errors INCLUDING bias (tag 0.74 m below anchors, NLOS)
%   Long sequences are built from random chunks (up to 20 s) of the recording,
%   blended over 1 s at each join, so the errors keep their real time
%   correlation. NaN = anchor missing in that frame.
%   Uses the global random generator: set the seed before calling.
persistent cache
if isempty(cache), cache = struct(); end
if ~isfield(cache, profile)
    ext = '.xlsx';
    if ~(exist('readmatrix', 'file') || exist('readmatrix', 'builtin')), ext = '.csv'; end
    switch profile
        case 'typical'
            F = nf2_load(fullfile(root, 'data', ['static_1533' ext]), 4);
            med = zeros(1, 4);
            for a = 1:4, d = F.D(isfinite(F.D(:, a)), a); med(a) = median(d); end
            E = F.D - repmat(med, size(F.D, 1), 1);
        case 'worst'
            F = nf2_load(fullfile(root, 'data', ['underA3_1620' ext]), 4);
            A = [0 0; 0.681 7.003; 11.958 5.357; 12.093 0];     % 26-Oct anchor layout
            dh = 1.30 - 0.56;                                     % anchor - tag height
            tru = sqrt(sum((A - [12.093 0]).^2, 2) + dh^2)';      % tag under A3 (tape)
            E = F.D - repmat(tru, size(F.D, 1), 1);
        otherwise
            error('profile must be ''typical'' or ''worst''');
    end
    cache.(profile) = E;
end
E = cache.(profile);
% Build a long sequence from random chunks of the recording. Joins are blended
% over 1 s so the slowly changing bias does not jump (a jump looks like movement).
blk = min(size(E, 1) - 1, 500);             % up to 20 s chunks at 25 Hz
fade = 25;
o = randi(size(E, 1) - blk);
err = E(o + (1:blk), :);
while size(err, 1) < n
    o = randi(size(E, 1) - blk);
    nxt = E(o + (1:blk), :);
    a = repmat(linspace(0, 1, fade)', 1, 4);
    j = size(err, 1) - fade + (1:fade);
    mix = (1 - a) .* err(j, :) + a .* nxt(1:fade, :);
    mix(isnan(err(j, :)) | isnan(nxt(1:fade, :))) = NaN;
    err = [err(1:end - fade, :); mix; nxt(fade + 1:end, :)];
end
err = err(1:n, :);
end
