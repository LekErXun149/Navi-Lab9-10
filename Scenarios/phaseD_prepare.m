function [F, E, P] = phaseD_prepare(sc, profile, seed, cfg, root, fileIdx)
%PHASED_PREPARE  Frames, localisation output and (true) path for one scenario run.
%   [F, E, P] = phaseD_prepare(sc, profile, seed, cfg, root)        simulated scenario
%   [F, E, P] = phaseD_prepare(sc, '', 0, cfg, root, fileIdx)       S17 replay of a real walk:
%       no truth exists, so P.xy is the ESTIMATED position (metrics are relative to it)
dt = 0.04;
if strcmp(sc.type, 'sim')
    phaseD_seed(seed);
    P = sc.P;
    F = phaseD_frames(P, sc, profile, cfg, root);
    [E, ~] = uwb_process(F, [], cfg);
else
    ext = '.xlsx';
    if ~(exist('readmatrix', 'file') || exist('readmatrix', 'builtin')), ext = '.csv'; end
    F = nf2_load(fullfile(root, 'data', [sc.files{fileIdx} ext]), cfg.anchor.n);
    t = F.t;
    for k = 2:numel(t)                                % remove time jumps, start at 0
        if t(k) <= t(k-1), t(k:end) = t(k:end) + (t(k-1) - t(k)) + dt; end
    end
    F.t = round((t - t(1)) / dt) * dt;
    [~, keep] = unique(F.t); F = nf2_slice(F, keep);
    [E, ~] = uwb_process(F, [], cfg);
    P.t = (0:dt:F.t(end))';
    ok = isfinite(E.pos(:, 1));
    P.xy = [interp1(F.t(ok), E.pos(ok, 1), P.t, 'previous', 'extrap') ...
            interp1(F.t(ok), E.pos(ok, 2), P.t, 'previous', 'extrap')];
    P.mark = struct();
end
end
