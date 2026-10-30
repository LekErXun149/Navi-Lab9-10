function Lg = safety_run_offline(F, cfg, dtStep)
%SAFETY_RUN_OFFLINE  Run localisation + safety logic + AGV simulation on a recording.
%   Lg = safety_run_offline(F, cfg)          F = frame batch (nf2_load)
%   Lg = safety_run_offline(F, cfg, dtStep)  simulation step, default 0.04 s
%   Deterministic (no wall clock): used by phaseC_offline and the Phase D tests.
%   Time jumps backwards in a recording are removed; long gaps stay (-> stale).
%   Lg has one row per simulation step:
%     t, pos, sigmaPos, speed, stale, fenceOn, B, laneD (nL), laneTTL, laneLevel,
%     desired (nA), applied, v, s, overall, plus agvNames, laneNames.
if nargin < 3, dtStep = 0.04; end

% monotonic time
t = F.t;
for k = 2:numel(t)
    if t(k) <= t(k-1), t(k:end) = t(k:end) + (t(k-1) - t(k)) + dtStep; end
end
F.t = t;

S = safety_init(cfg); st = [];
nL = numel(S.lane); nA = numel(S.agv);
grid = (t(1):dtStep:t(end) + dtStep)';
n = numel(grid);
Lg.t = grid;
Lg.pos = nan(n, 2); Lg.sigmaPos = nan(n, 1); Lg.speed = nan(n, 1);
Lg.stale = false(n, 1); Lg.fenceOn = false(n, 1); Lg.B = nan(n, 1);
Lg.laneD = nan(n, nL); Lg.laneTTL = inf(n, nL); Lg.laneLevel = ones(n, nL);
Lg.desired = ones(n, nA); Lg.applied = ones(n, nA); Lg.v = nan(n, nA); Lg.s = nan(n, nA);
Lg.overall = ones(n, 1);
Lg.agvNames = {S.agv.name}; Lg.laneNames = {cfg.site.lanes.name};

j = 1; nF = numel(t);
for k = 1:n
    i0 = j;
    while j <= nF && t(j) <= grid(k), j = j + 1; end
    est = [];
    if j > i0
        [E, st] = uwb_process(nf2_slice(F, i0:j-1), st, cfg);
        est = struct('pos', E.pos(end, :), 'vel', E.vel(end, :), ...
                     'sigmaPos', E.sigmaPos(end), 'stale', E.stale(end), 'speed', E.speed(end));
    end
    [S, o] = safety_step(S, grid(k), est, cfg);
    if ~isempty(S.est)
        Lg.pos(k, :) = S.est.pos; Lg.sigmaPos(k) = S.est.sigmaPos; Lg.speed(k) = S.est.speed;
    end
    Lg.stale(k) = o.stale; Lg.fenceOn(k) = o.fenceOn; Lg.B(k) = o.B;
    Lg.laneD(k, :) = [o.lane.d]; Lg.laneTTL(k, :) = [o.lane.ttl]; Lg.laneLevel(k, :) = [o.lane.level];
    Lg.desired(k, :) = o.desired; Lg.applied(k, :) = o.applied;
    Lg.v(k, :) = o.v; Lg.s(k, :) = o.s; Lg.overall(k) = o.overall;
end
end
