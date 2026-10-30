function F = phaseD_frames(P, sc, profile, cfg, root)
%PHASED_FRAMES  Turn a true path into the frames the tag would have sent.
%   F = phaseD_frames(P, sc, profile, cfg, root)
%   For every time step: true 3D range to each anchor (with the anchor/tag height
%   difference) + real range error (phaseD_noise) + scenario disturbances:
%     sc.dropout = [t0 t1; ...]           no frames at all (signal lost)
%     sc.nlos    = [anchor t0 t1 bias]    extra range error on one anchor (0-based id)
%   The result is a normal frame batch, processed exactly like live data.
n = numel(P.t); nA = cfg.anchor.n;
dh = cfg.anchor.h(:)' - cfg.tag.h;
R = zeros(n, nA);
for a = 1:nA
    R(:, a) = sqrt((P.xy(:, 1) - cfg.anchor.xy(a, 1)).^2 + (P.xy(:, 2) - cfg.anchor.xy(a, 2)).^2 + dh(a)^2);
end
D = R + phaseD_noise(profile, n, root);
if isfield(sc, 'nlos')
    for k = 1:size(sc.nlos, 1)
        m = P.t >= sc.nlos(k, 2) & P.t < sc.nlos(k, 3);
        D(m, sc.nlos(k, 1) + 1) = D(m, sc.nlos(k, 1) + 1) + sc.nlos(k, 4);
    end
end
keep = true(n, 1);
if isfield(sc, 'dropout')
    for k = 1:size(sc.dropout, 1)
        keep(P.t >= sc.dropout(k, 1) & P.t < sc.dropout(k, 2)) = false;
    end
end
F = nf2_empty(nA, nnz(keep));
F.t = P.t(keep); F.sysTime = F.t;
F.D = D(keep, :);
F.nNodes = sum(isfinite(F.D), 2);
end
