function [m, rec, EV] = phaseD_simulate(F, E, P, sc, cfg)
%PHASED_SIMULATE  Run the safety logic + AGVs over a scenario and measure against truth.
%   [m, rec, EV] = phaseD_simulate(F, E, P, sc, cfg)
%     F, E  frames and their uwb_process output (computed once, shared by all versions)
%     P     true path (phaseD_path), sc scenario, cfg with cfg.policy = version to test
%   The first sc.warm seconds are a warm-up (person outside the fence, AGVs at full
%   speed); metrics only count after it. sc.agv sets AGV positions at the moment the
%   person first enters the fence (to stage a situation, e.g. an AGV just passing).

dt = P.t(2) - P.t(1); n = numel(P.t);
fIdx = zeros(n, 1);                                   % grid step -> frame index (0 = none)
fIdx(round(F.t / dt) + 1) = 1:numel(F.t);

S = safety_init(cfg); nA = numel(S.agv);
for k = 1:nA                                          % production running at the start
    S.agv(k).v = cfg.agv.vMax; S.agv(k).desired = 1; S.agv(k).applied = 1; S.agv(k).queue = [-Inf 1];
end
names = {S.agv.name};
tFence = phaseD_tfence(P, cfg);
kSet = find(P.t >= tFence - sc.agvLead, 1);           % when to stage the AGVs

rec.t = P.t; rec.desired = ones(n, nA); rec.applied = ones(n, nA);
rec.v = zeros(n, nA); rec.s = zeros(n, nA); rec.stale = false(n, 1);
rec.fenceOn = false(n, 1); rec.laneLevel = ones(n, numel(S.lane)); rec.pos = nan(n, 2); rec.B = nan(n, 1);
EV = [];
for k = 1:n
    if ~isempty(kSet) && k == kSet && isfield(sc, 'agv')
        for j = 1:size(sc.agv, 1)                     % {name, s at fence entry}
            i = find(strcmp(names, sc.agv{j, 1}));
            S.agv(i).s = sc.agv{j, 2} - cfg.agv.vMax * sc.agvLead;
            S.agv(i).v = cfg.agv.vMax;
        end
    end
    est = [];
    if fIdx(k) > 0
        i = fIdx(k);
        est = struct('pos', E.pos(i, :), 'vel', E.vel(i, :), 'sigmaPos', E.sigmaPos(i), ...
                     'stale', E.stale(i), 'speed', E.speed(i));
    end
    [S, o] = safety_step(S, P.t(k), est, cfg);
    if nargout > 2, EV = safety_events(EV, S, o, cfg); end
    rec.desired(k, :) = o.desired; rec.applied(k, :) = o.applied;
    rec.v(k, :) = o.v; rec.s(k, :) = o.s; rec.stale(k) = o.stale;
    rec.fenceOn(k) = o.fenceOn; rec.laneLevel(k, :) = [o.lane.level]; rec.pos(k, :) = o.pos; rec.B(k) = o.B;
end
rec.tFence = tFence;
m = phaseD_metrics(rec, P, sc, cfg, S);
end
